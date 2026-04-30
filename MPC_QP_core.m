function du = MPC_QP_core(y, r, A, B, N1, N2, Nu, alpha, lam, reset, ...
                    C, D, wy, umin, umax, dumin, dumax, u_init, du_init, ...
                    Qkf, Rkf, P0, use_quadprog, is_active, u_applied, solve_when_inactive, ...
                    use_steady_state_kf, L)
%MPC_QP_CORE Core constrained MPC implementation with activity-aware estimation.
%   The estimator can use either a standard recursive Kalman filter or a
%   fixed steady-state Kalman gain supplied by the caller.
%
%   Qkf : process noise covariance for the incremental state model
%   Rkf : measurement noise covariance for dy_meas
%   P0  : initial state estimation covariance
%   use_steady_state_kf : true to use fixed Kalman gain L
%   L   : steady-state Kalman gain with size nx-by-ny

    persistent xhat P y_prev y_filt u_applied_prev ...
        du_prev_assumed init_flag was_active_prev

    if nargin < 24 || isempty(is_active)
        is_active = true;
    end
    has_u_applied = (nargin >= 25) && ~isempty(u_applied);
    if nargin < 26 || isempty(solve_when_inactive)
        solve_when_inactive = false;
    end
    if nargin < 27 || isempty(use_steady_state_kf)
        use_steady_state_kf = false;
    end
    if nargin < 28
        L = [];
    end

    y = y(:);
    r = r(:);
    wy = wy(:);
    umin = umin(:);
    umax = umax(:);
    dumin = dumin(:);
    dumax = dumax(:);
    u_init = u_init(:);
    du_init = du_init(:);

    nx = size(A, 1);
    nu = size(B, 2);
    ny = size(C, 1);

    use_steady_state_kf = normalize_logical_scalar(use_steady_state_kf, 'use_steady_state_kf');

    if use_steady_state_kf
        beta = 0; % No filtering when using steady-state KF
        L = validate_steady_state_gain(L, nx, ny);
        if isempty(P0)
            P0 = zeros(nx);
        else
            P0 = expand_covariance(P0, nx, 'P0');
        end
    else
        beta = 0.5; % Smoothing factor for y_filt
        Qkf = expand_covariance(Qkf, nx, 'Qkf');
        Rkf = expand_covariance(Rkf, ny, 'Rkf');
        P0  = expand_covariance(P0,  nx, 'P0');
    end

    du = zeros(nu, 1);

    if has_u_applied
        u_applied = u_applied(:);
    end

    if isempty(init_flag)
        init_flag = true;
    end

    is_active = normalize_is_active(is_active);

    if reset
        xhat = zeros(nx, 1);
        P = P0;

        y_prev = y;
        y_filt = y;

        u_applied_prev = u_init;
        du_prev_assumed = du_init;

        init_flag = true;
        was_active_prev = is_active;

        return;
    end

    if numel(y) ~= 3 || numel(r) ~= 3
        error('y and r must be 3x1 vectors.');
    end
    if nu ~= 3 || ny ~= 3
        error('MPC_QP expects a 3-input, 3-output model.');
    end
    if numel(u_init) ~= nu || numel(du_init) ~= nu
        error('u_init and du_init must have one entry per input.');
    end
    if has_u_applied && numel(u_applied) ~= nu
        error('u_applied must have one entry per input.');
    end

    if isscalar(alpha)
        alpha_vec = repmat(alpha, 3, 1);
    elseif numel(alpha) == 3
        alpha_vec = alpha(:);
    else
        error('alpha must be a scalar or a 3-element vector.');
    end

    if isempty(xhat)
        xhat = zeros(nx, 1);
    end
    if isempty(P)
        P = P0;
    end
    if isempty(y_prev)
        y_prev = y;
    end
    if isempty(y_filt)
        y_filt = y;
    end
    if isempty(u_applied_prev)
        u_applied_prev = u_init;
    end
    if isempty(du_prev_assumed)
        du_prev_assumed = du_init;
    end
    if isempty(was_active_prev)
        was_active_prev = true;
    end

    if ~has_u_applied
        u_applied = u_applied_prev + assumed_increment(was_active_prev, du_prev_assumed, nu);
    end

    if init_flag
        xhat = zeros(nx, 1);
        P = P0;
        y_prev = y;
        y_filt = y;
        u_applied_prev = u_applied;
    end

    validate_horizons(N1, N2, Nu);

    % -------------------------------------------------------------
    % 1. Filter measured output
    % -------------------------------------------------------------
    y_filt = beta * y_filt + (1 - beta) * y;
    y_used = y_filt;

    % -------------------------------------------------------------
    % 2. Build measured output increment
    % -------------------------------------------------------------
    dy_meas = y_used - y_prev;

    if init_flag
        du_applied = zeros(nu, 1);

        x_pred = xhat;
        P_pred = P;
    else
        du_applied = u_applied - u_applied_prev;

        % ---------------------------------------------------------
        % 3. Kalman time update
        % ---------------------------------------------------------
        x_pred = A * xhat + B * du_applied;
        if use_steady_state_kf
            P_pred = P;
        else
            P_pred = A * P * A' + Qkf;
            P_pred = (P_pred + P_pred') / 2;
        end
    end

    % -------------------------------------------------------------
    % 4. Kalman measurement update
    %
    % Measurement equation:
    %   dy_meas = C*x_pred + D*du_applied + measurement noise
    % -------------------------------------------------------------
    dyhat = C * x_pred + D * du_applied;
    innovation = dy_meas - dyhat;

    if use_steady_state_kf
        Kkf = L;
    else
        S = C * P_pred * C' + Rkf;
        S = (S + S') / 2 + 1e-12 * eye(ny);

        Kkf = (P_pred * C') / S;
    end
    xhat = x_pred + Kkf * innovation;

    if use_steady_state_kf
        P = P_pred;
    else
        % Joseph covariance update, numerically safer than P=(I-KC)P
        I = eye(nx);
        P = (I - Kkf * C) * P_pred * (I - Kkf * C)' + Kkf * Rkf * Kkf';
        P = (P + P') / 2;
    end

    % -------------------------------------------------------------
    % 5. MPC prediction and QP
    % -------------------------------------------------------------
    [PhiY, GammaY] = build_prediction_mats(A, B, C, D, N1, N2, Nu);

    F = PhiY * xhat + repmat(y_used, N2 - N1 + 1, 1);
    R = build_reference(y_used, r, N1, N2, alpha_vec);

    Qy = build_output_weight(wy, N2 - N1 + 1);
    Ru = build_move_weight(lam, Nu, nu);

    H = GammaY' * Qy * GammaY + Ru;
    f = -GammaY' * Qy * (R - F);

    qp_solved = is_active || logical(solve_when_inactive);

    if qp_solved
        [lb, ub, Aineq, bineq] = build_constraints(dumin, dumax, umin, umax, Nu, u_applied, nu);
        dU = solve_qp(H, f, lb, ub, Aineq, bineq, u_applied, umin, umax, use_quadprog, nu);
        du_candidate = dU(1:nu);
    else
        du_candidate = zeros(nu, 1);
    end

    if is_active
        du = du_candidate;
    end

    % -------------------------------------------------------------
    % 6. Update persistent variables
    % -------------------------------------------------------------
    y_prev = y_used;
    u_applied_prev = u_applied;
    du_prev_assumed = du;
    init_flag = false;
    was_active_prev = is_active;
end

function tf = normalize_logical_scalar(value, name)
    if islogical(value) && isscalar(value)
        tf = value;
        return;
    end

    if isnumeric(value) && isscalar(value) && isfinite(value) && any(value == [0, 1])
        tf = logical(value);
        return;
    end

    error('%s must be a logical scalar or numeric 0/1 scalar.', name);
end

function L = validate_steady_state_gain(L, nx, ny)
    if isempty(L)
        error('L must be provided when use_steady_state_kf is true.');
    end

    if ~isequal(size(L), [nx, ny])
        error('L must be an %dx%d steady-state Kalman gain matrix.', nx, ny);
    end
end

function M = expand_covariance(M, n, name)
    if isempty(M)
        error('%s must not be empty.', name);
    end

    if isscalar(M)
        M = M * eye(n);
    elseif isvector(M) && numel(M) == n
        M = diag(M(:));
    elseif isequal(size(M), [n, n])
        M = (M + M') / 2;
    else
        error('%s must be a scalar, an %d-element vector, or a %dx%d matrix.', ...
              name, n, n, n);
    end
end

function validate_horizons(N1, N2, Nu)
    validateattributes(N1, {'numeric'}, {'scalar', 'integer', '>=', 1}, mfilename, 'N1');
    validateattributes(N2, {'numeric'}, {'scalar', 'integer', '>=', N1}, mfilename, 'N2');
    validateattributes(Nu, {'numeric'}, {'scalar', 'integer', '>=', 1}, mfilename, 'Nu');
end

function is_active = normalize_is_active(is_active)
    is_active = normalize_logical_scalar(is_active, 'is_active');
end

function du_assumed = assumed_increment(was_active_prev, du_prev_assumed, nu)
    if was_active_prev
        du_assumed = du_prev_assumed(:);
    else
        du_assumed = zeros(nu, 1);
    end
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

function [lb, ub, Aineq, bineq] = build_constraints(dumin, dumax, umin, umax, Nu, u_base, nu)
    lb = repmat(dumin(:), Nu, 1);
    ub = repmat(dumax(:), Nu, 1);

    S = kron(tril(ones(Nu)), eye(nu));
    u_stack = repmat(u_base(:), Nu, 1);
    umin_stack = repmat(umin(:), Nu, 1);
    umax_stack = repmat(umax(:), Nu, 1);

    Aineq = [S; -S];
    bineq = [umax_stack - u_stack;
            -(umin_stack - u_stack)];
end

function dU = solve_qp(H, f, lb, ub, Aineq, bineq, u_base, umin, umax, use_quadprog, nu)
    H = (H + H') / 2;

    if use_quadprog
        opts = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off');
        x0 = zeros(size(f));
        dU = quadprog(2 * H, 2 * f, Aineq, bineq, [], [], lb, ub, x0, opts);
        if ~isempty(dU)
            return;
        end
    end

    dU = -H \ f;
    dU = min(max(dU, lb), ub);

    Nu = numel(dU) / nu;
    S = kron(tril(ones(Nu)), eye(nu));
    u_seq = S * dU + repmat(u_base(:), Nu, 1);
    u_seq = min(max(u_seq, repmat(umin(:), Nu, 1)), repmat(umax(:), Nu, 1));

    for k = 1:Nu
        idx = (k - 1) * nu + (1:nu);
        if k == 1
            dU(idx) = u_seq(idx) - u_base(:);
        else
            dU(idx) = u_seq(idx) - u_seq(idx - nu);
        end
        dU(idx) = min(max(dU(idx), lb(idx)), ub(idx));
    end
end

