function u = MPC_QP(y, r, model, N1, N2, Nu, alpha, lam, reset, cfg)
%MPC_QP Constrained MPC for an incremental discrete state-space model.
%   u = MPC_QP(y, r, model, N1, N2, Nu, alpha, lam, reset, cfg)
%
%   Inputs
%     y     - absolute measured output, 3x1
%     r     - setpoint, 3x1
%     model - discrete state-space model or struct with fields A,B,C,D
%     N1    - first prediction step used in the cost
%     N2    - last prediction step used in the cost
%     Nu    - control horizon
%     alpha - scalar or 3x1 reference smoothing factor
%     lam   - scalar or 3x1 move suppression weights
%     reset - logical reset flag
%     cfg   - optional struct
%
%   Optional cfg fields
%     wy          - 3x1 output weights, default [1;1;1]
%     umin        - 3x1 lower bounds on u
%     umax        - 3x1 upper bounds on u
%     dumin       - 3x1 lower bounds on du
%     dumax       - 3x1 upper bounds on du
%     x0          - nx x 1 initial state estimate
%     u0          - 3x1 initial absolute input
%     du0         - 3x1 previous input increment
%     y0          - 3x1 previous absolute output used to form dy
%     L           - nx x 3 observer gain
%     use_quadprog - use quadprog when available, default true if present
%
%   Notes
%     - The given model is assumed to already be incremental:
%           x(k+1) = A x(k) + B du(k)
%          dy(k)   = C x(k) + D du(k)
%     - The controller still tracks the absolute input u internally so that
%       absolute-input constraints can be enforced.
%     - The measured input y is absolute. The controller computes dy
%       internally and integrates predicted dy to build absolute y
%       predictions for tracking.
%     - The optimized variable is du, but the function output is the
%       absolute control signal u(k).
%     - If cfg.L is omitted, a simple output-injection gain pinv(C) is used
%       as a toolbox-free default observer correction.

    persistent xhat u_prev du_prev y_prev A B C D ...
               PhiY_sel GammaY_sel ...
               last_model_sig last_N1 last_N2 last_Nu last_wy last_lam ...
               last_L init_flag

    if nargin < 10 || isempty(cfg)
        cfg = struct();
    end
    cfg = fill_cfg_defaults(cfg);

    if isempty(init_flag)
        init_flag = true;
    end

    if reset
        init_flag = true;
        u = cfg.u0(:);
        return;
    end

    y = y(:);
    r = r(:);
    if numel(y) ~= 3 || numel(r) ~= 3
        error('y and r must be 3x1 vectors.');
    end

    if isscalar(alpha)
        alpha_vec = repmat(alpha, 3, 1);
    elseif numel(alpha) == 3
        alpha_vec = alpha(:);
    else
        error('alpha must be a scalar or a 3-element vector.');
    end

    [Ain, Bin, Cin, Din] = unpack_model(model);
    nx = size(Ain, 1);
    nu = size(Bin, 2);
    ny = size(Cin, 1);

    if nu ~= 3 || ny ~= 3
        error('MPC_QP currently expects a 3-input, 3-output model.');
    end

    if init_flag || isempty(xhat)
        u_prev = cfg.u0(:);
        du_prev = cfg.du0(:);
        if isempty(cfg.y0)
            y_prev = y;
        else
            y_prev = cfg.y0(:);
        end
        xhat = initialize_state(cfg, Cin, Din, y - y_prev, du_prev, nx);
    end

    L = resolve_observer_gain(cfg, Cin, nx, ny);
    model_sig = [Ain(:); Bin(:); Cin(:); Din(:)];

    if init_flag || isempty(PhiY_sel) || ...
            isempty(last_model_sig) || ~isequal(model_sig, last_model_sig) || ...
            N1 ~= last_N1 || N2 ~= last_N2 || Nu ~= last_Nu || ...
            ~isequal(cfg.wy, last_wy) || ~isequal(lam(:), last_lam) || ...
            ~isequal(L, last_L)

        validate_horizons(N1, N2, Nu);

        A = Ain;
        B = Bin;
        C = Cin;
        D = Din;

        [PhiY_sel, GammaY_sel] = build_prediction_mats(A, B, C, D, N1, N2, Nu);

        last_model_sig = model_sig;
        last_N1 = N1;
        last_N2 = N2;
        last_Nu = Nu;
        last_wy = cfg.wy;
        last_lam = lam(:);
        last_L = L;
    end

    dy_meas = y - y_prev;

    % Prediction-correction state observer:
    % x(k|k-1) = A x(k-1|k-1) + B du(k-1)
    % dy(k|k-1)= C x(k|k-1) + D du(k-1)
    % x(k|k)   = x(k|k-1) + L (dy(k) - dyhat(k|k-1))
    if ~init_flag
        xhat = A * xhat + B * du_prev;
    end
    dyhat = C * xhat + D * du_prev;
    xhat = xhat + L * (dy_meas - dyhat);

    F = PhiY_sel * xhat + repmat(y, N2 - N1 + 1, 1);
    R = build_reference(y, r, N1, N2, alpha_vec);

    Qy = build_output_weight(cfg.wy, N2 - N1 + 1);
    Ru = build_move_weight(lam, Nu, nu);
    H = GammaY_sel' * Qy * GammaY_sel + Ru;
    f = -GammaY_sel' * Qy * (R - F);

    [lb, ub, Aineq, bineq] = build_constraints(cfg, Nu, u_prev, nu);
    dU = solve_qp(H, f, lb, ub, Aineq, bineq, u_prev, cfg, nu);

    du = dU(1:nu);
    u_prev = u_prev + du;
    du_prev = du;
    y_prev = y;
    u = u_prev;
    init_flag = false;
end

function cfg = fill_cfg_defaults(cfg)
    if ~isfield(cfg, 'wy'), cfg.wy = ones(3, 1); end
    if ~isfield(cfg, 'umin'), cfg.umin = -inf(3, 1); end
    if ~isfield(cfg, 'umax'), cfg.umax = inf(3, 1); end
    if ~isfield(cfg, 'dumin'), cfg.dumin = -inf(3, 1); end
    if ~isfield(cfg, 'dumax'), cfg.dumax = inf(3, 1); end
    if ~isfield(cfg, 'u0'), cfg.u0 = zeros(3, 1); end
    if ~isfield(cfg, 'du0'), cfg.du0 = zeros(3, 1); end
    if ~isfield(cfg, 'y0'), cfg.y0 = []; end
    if ~isfield(cfg, 'x0'), cfg.x0 = []; end
    if ~isfield(cfg, 'L'), cfg.L = []; end
    if ~isfield(cfg, 'use_quadprog')
        cfg.use_quadprog = exist('quadprog', 'file') == 2;
    end

    cfg.wy = cfg.wy(:);
    cfg.umin = cfg.umin(:);
    cfg.umax = cfg.umax(:);
    cfg.dumin = cfg.dumin(:);
    cfg.dumax = cfg.dumax(:);
    cfg.u0 = cfg.u0(:);
    cfg.du0 = cfg.du0(:);
    if ~isempty(cfg.y0)
        cfg.y0 = cfg.y0(:);
    end
end

function [A, B, C, D] = unpack_model(model)
    if isstruct(model)
        A = model.A;
        B = model.B;
        C = model.C;
        if isfield(model, 'D') && ~isempty(model.D)
            D = model.D;
        else
            D = zeros(size(C, 1), size(B, 2));
        end
        return;
    end

    try
        A = model.A;
        B = model.B;
        C = model.C;
        D = model.D;
    catch
        error('model must be a struct or discrete state-space object with A,B,C,D.');
    end

    if isempty(D)
        D = zeros(size(C, 1), size(B, 2));
    end
end

function x0 = initialize_state(cfg, C, D, dy, du_prev, nx)
    if ~isempty(cfg.x0)
        x0 = cfg.x0(:);
        if numel(x0) ~= nx
            error('cfg.x0 must have length equal to the number of states.');
        end
        return;
    end

    % Least-squares output matching is a simple default when no observer
    % design is provided. Replace with a better x0 if you already have one.
    x0 = pinv(C) * (dy - D * du_prev);
    if numel(x0) ~= nx
        x0 = [x0; zeros(nx - numel(x0), 1)];
    end
end

function L = resolve_observer_gain(cfg, C, nx, ny)
    if ~isempty(cfg.L)
        L = cfg.L;
        if ~isequal(size(L), [nx, ny])
            error('cfg.L must be sized nx-by-ny.');
        end
        return;
    end

    % Toolbox-free default observer injection.
    L = pinv(C);
    if ~isequal(size(L), [nx, ny])
        tmp = zeros(nx, ny);
        rows = min(nx, size(L, 1));
        cols = min(ny, size(L, 2));
        tmp(1:rows, 1:cols) = L(1:rows, 1:cols);
        L = tmp;
    end
end

function validate_horizons(N1, N2, Nu)
    validateattributes(N1, {'numeric'}, {'scalar', 'integer', '>=', 1}, mfilename, 'N1');
    validateattributes(N2, {'numeric'}, {'scalar', 'integer', '>=', N1}, mfilename, 'N2');
    validateattributes(Nu, {'numeric'}, {'scalar', 'integer', '>=', 1}, mfilename, 'Nu');
end

function [PhiY_sel, GammaY_sel] = build_prediction_mats(A, B, C, D, N1, N2, Nu)
    nx = size(A, 1);
    ny = size(C, 1);
    nu = size(B, 2);

    PhiDy = zeros(ny * N2, nx);
    GammaDy = zeros(ny * N2, nu * Nu);

    A_pow = eye(nx);
    for j = 1:N2
        A_pow = A_pow * A;
        row_idx = (j - 1) * ny + (1:ny);
        PhiDy(row_idx, :) = C * A_pow;

        for i = 1:min(j, Nu)
            col_idx = (i - 1) * nu + (1:nu);
            if j == i
                GammaDy(row_idx, col_idx) = C * B + D;
            else
                GammaDy(row_idx, col_idx) = C * (A ^ (j - i)) * B;
            end
        end
    end

    T = kron(tril(ones(N2)), eye(ny));
    PhiY = T * PhiDy;
    GammaY = T * GammaDy;

    select = false(ny * N2, 1);
    for j = N1:N2
        row_idx = (j - 1) * ny + (1:ny);
        select(row_idx) = true;
    end

    PhiY_sel = PhiY(select, :);
    GammaY_sel = GammaY(select, :);
end

function R = build_reference(y, r, N1, N2, alpha)
    ny = numel(y);
    ref = zeros(ny, N2);
    ref(:, 1) = alpha(:) .* y(:) + (1 - alpha(:)) .* r(:);

    for k = 2:N2
        ref(:, k) = alpha(:) .* ref(:, k - 1) + (1 - alpha(:)) .* r(:);
    end

    rows = N2 - N1 + 1;
    R = zeros(ny * rows, 1);
    for j = N1:N2
        block = (j - N1) * ny + (1:ny);
        R(block) = ref(:, j);
    end
end

function W = build_output_weight(wy, rows)
    W = kron(eye(rows), diag(wy(:)));
end

function Lam = build_move_weight(lam, Nu, nu)
    if isscalar(lam)
        lam_vec = repmat(lam, nu, 1);
    elseif numel(lam) == nu
        lam_vec = lam(:);
    else
        error('lam must be a scalar or a vector with one entry per input.');
    end

    Lam = kron(eye(Nu), diag(lam_vec));
end

function [lb, ub, Aineq, bineq] = build_constraints(cfg, Nu, u_prev, nu)
    lb = repmat(cfg.dumin(:), Nu, 1);
    ub = repmat(cfg.dumax(:), Nu, 1);

    S = kron(tril(ones(Nu)), eye(nu));
    u_stack = repmat(u_prev(:), Nu, 1);
    umin_stack = repmat(cfg.umin(:), Nu, 1);
    umax_stack = repmat(cfg.umax(:), Nu, 1);

    Aineq = [S; -S];
    bineq = [umax_stack - u_stack;
            -(umin_stack - u_stack)];
end

function dU = solve_qp(H, f, lb, ub, Aineq, bineq, u_prev, cfg, nu)
    H = (H + H') / 2;

    if cfg.use_quadprog
        opts = optimoptions('quadprog', 'Display', 'off');
        dU = quadprog(2 * H, 2 * f, Aineq, bineq, [], [], lb, ub, [], opts);
        if ~isempty(dU)
            return;
        end
    end

    dU = -H \ f;
    dU = min(max(dU, lb), ub);

    Nu = numel(dU) / nu;
    S = kron(tril(ones(Nu)), eye(nu));
    u_seq = S * dU + repmat(u_prev(:), Nu, 1);
    u_seq = min(max(u_seq, repmat(cfg.umin(:), Nu, 1)), repmat(cfg.umax(:), Nu, 1));

    for k = 1:Nu
        idx = (k - 1) * nu + (1:nu);
        if k == 1
            dU(idx) = u_seq(idx) - u_prev(:);
        else
            dU(idx) = u_seq(idx) - u_seq(idx - nu);
        end
        dU(idx) = min(max(dU(idx), lb(idx)), ub(idx));
    end
end
