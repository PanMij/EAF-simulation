function [du, info] = MPC_QP_core(y, r, A, B, N1, N2, Nu, alpha, lam, reset, ...
                    C, D, wy, umin, umax, dumin, dumax, u_init, du_init, ...
                    L, use_quadprog, mode, u_applied, solve_when_inactive)
%MPC_QP_CORE Core constrained MPC implementation with mode-aware estimation.

    persistent xhat y_prev u_applied_prev du_prev_assumed init_flag last_mode

    modes = controller_modes();

    if nargin < 22 || isempty(mode)
        mode = modes.MPC_ACTIVE;
    end
    has_u_applied = (nargin >= 23) && ~isempty(u_applied);
    if nargin < 24 || isempty(solve_when_inactive)
        solve_when_inactive = false;
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

    du = zeros(nu, 1);

    if has_u_applied
        u_applied = u_applied(:);
    end

    if isempty(init_flag)
        init_flag = true;
    end

    if reset
        xhat = zeros(nx, 1);
        y_prev = y;
        u_applied_prev = u_init;
        du_prev_assumed = du_init;
        init_flag = true;
        last_mode = mode;
        if nargout > 1
            info = build_info_struct(mode, false, has_u_applied, u_init, zeros(nu, 1), ...
                                     zeros(nu, 1), xhat, C * xhat, modes);
        end
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

    mode = validate_mode(mode, modes);

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
    if isempty(y_prev)
        y_prev = y;
    end
    if isempty(u_applied_prev)
        u_applied_prev = u_init;
    end
    if isempty(du_prev_assumed)
        du_prev_assumed = du_init;
    end
    if isempty(last_mode)
        last_mode = modes.MPC_ACTIVE;
    end

    if ~has_u_applied
        u_applied = u_applied_prev + assumed_increment(last_mode, du_prev_assumed, modes, nu);
    end

    if init_flag
        xhat = zeros(nx, 1);
        y_prev = y;
        u_applied_prev = u_applied;
    end

    if all(L(:) == 0)
        L_use = pinv(C);
    else
        L_use = L;
    end

    validate_horizons(N1, N2, Nu);

    dy_meas = y - y_prev;
    if init_flag
        du_applied = zeros(nu, 1);
    else
        du_applied = u_applied - u_applied_prev;
        xhat = A * xhat + B * du_applied;
    end

    dyhat = C * xhat + D * du_applied;
    xhat = xhat + L_use * (dy_meas - dyhat);

    [PhiY, GammaY] = build_prediction_mats(A, B, C, D, N1, N2, Nu);
    F = PhiY * xhat + repmat(y, N2 - N1 + 1, 1);
    R = build_reference(y, r, N1, N2, alpha_vec);

    Qy = build_output_weight(wy, N2 - N1 + 1);
    Ru = build_move_weight(lam, Nu, nu);
    H = GammaY' * Qy * GammaY + Ru;
    f = -GammaY' * Qy * (R - F);

    qp_solved = (mode == modes.MPC_ACTIVE) || logical(solve_when_inactive);
    if qp_solved
        [lb, ub, Aineq, bineq] = build_constraints(dumin, dumax, umin, umax, Nu, u_applied, nu);
        dU = solve_qp(H, f, lb, ub, Aineq, bineq, u_applied, umin, umax, use_quadprog, nu);
        du_candidate = dU(1:nu);
    else
        du_candidate = zeros(nu, 1);
    end

    if mode == modes.MPC_ACTIVE
        du = du_candidate;
    end

    y_prev = y;
    u_applied_prev = u_applied;
    du_prev_assumed = du;
    init_flag = false;
    last_mode = mode;

    if nargout > 1
        info = build_info_struct(mode, qp_solved, has_u_applied, u_applied, du_applied, ...
                                 du_candidate, xhat, dyhat, modes);
    end
end

function validate_horizons(N1, N2, Nu)
    validateattributes(N1, {'numeric'}, {'scalar', 'integer', '>=', 1}, mfilename, 'N1');
    validateattributes(N2, {'numeric'}, {'scalar', 'integer', '>=', N1}, mfilename, 'N2');
    validateattributes(Nu, {'numeric'}, {'scalar', 'integer', '>=', 1}, mfilename, 'Nu');
end

function mode = validate_mode(mode, modes)
    validateattributes(mode, {'numeric'}, {'scalar', 'integer'}, mfilename, 'mode');
    valid_modes = [modes.MPC_ACTIVE, modes.MANUAL_ACTIVE, modes.OTHER_CONTROLLER_ACTIVE];
    if ~any(mode == valid_modes)
        error('mode must be 1 (MPC_ACTIVE), 2 (MANUAL_ACTIVE), or 3 (OTHER_CONTROLLER_ACTIVE).');
    end
end

function du_assumed = assumed_increment(mode, du_prev_assumed, modes, nu)
    if mode == modes.MPC_ACTIVE
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

function info = build_info_struct(mode, qp_solved, has_u_applied, u_applied, du_applied, ...
                                  du_candidate, xhat, dyhat, modes)
    info = struct( ...
        'mode', mode, ...
        'mode_name', mode_name(mode, modes), ...
        'qp_solved', logical(qp_solved), ...
        'using_u_applied_feedback', logical(has_u_applied), ...
        'u_applied', u_applied(:), ...
        'du_applied', du_applied(:), ...
        'du_mpc', du_candidate(:), ...
        'u_mpc', u_applied(:) + du_candidate(:), ...
        'xhat', xhat(:), ...
        'dyhat', dyhat(:));
end

function name = mode_name(mode, modes)
    if mode == modes.MPC_ACTIVE
        name = 'MPC_ACTIVE';
    elseif mode == modes.MANUAL_ACTIVE
        name = 'MANUAL_ACTIVE';
    else
        name = 'OTHER_CONTROLLER_ACTIVE';
    end
end

function modes = controller_modes()
    modes = struct( ...
        'MPC_ACTIVE', 1, ...
        'MANUAL_ACTIVE', 2, ...
        'OTHER_CONTROLLER_ACTIVE', 3);
end
