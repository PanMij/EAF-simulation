function u = MPC_QP(y, r, A, B, C, D, N1, N2, Nu, alpha, lam, reset, ...
                    wy, umin, umax, dumin, dumax, u_init, du_init, L, use_quadprog)
%MPC_QP Constrained MPC for an incremental discrete state-space model.
%   u = MPC_QP(y, r, A, B, C, D, N1, N2, Nu, alpha, lam, reset, ...
%              wy, umin, umax, dumin, dumax, u_init, du_init, L, use_quadprog)
%
%   Model:
%       x(k+1) = A x(k) + B du(k)
%      dy(k)   = C x(k) + D du(k)
%
%   Inputs:
%     y            absolute measured output, 3x1
%     r            reference, 3x1
%     A,B,C,D      numeric state-space matrices
%     N1,N2,Nu     MPC horizons
%     alpha        scalar or 3x1 reference smoothing factor
%     lam          scalar or 3x1 move suppression weights
%     reset        reset flag
%     wy           3x1 output weights
%     umin,umax    3x1 absolute input bounds
%     dumin,dumax  3x1 input increment bounds
%     u_init       3x1 absolute input used on reset
%     du_init      3x1 previous input increment used on reset
%     L            observer gain, nx-by-3. Pass zeros(nx,3) to use pinv(C)
%     use_quadprog logical scalar, use quadprog if available
%
%   Output:
%     u            absolute control signal, 3x1

    persistent xhat u_prev du_prev y_prev init_flag

    if isempty(init_flag)
        init_flag = true;
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

    if reset
        init_flag = true;
        xhat = zeros(nx, 1);
        u_prev = u_init;
        du_prev = du_init;
        y_prev = y;
        u = u_prev;
        return;
    end

    if numel(y) ~= 3 || numel(r) ~= 3
        error('y and r must be 3x1 vectors.');
    end
    if nu ~= 3 || ny ~= 3
        error('MPC_QP expects a 3-input, 3-output model.');
    end

    if isscalar(alpha)
        alpha_vec = repmat(alpha, 3, 1);
    elseif numel(alpha) == 3
        alpha_vec = alpha(:);
    else
        error('alpha must be a scalar or a 3-element vector.');
    end

    if isempty(y_prev)
        y_prev = y;
    end
    if isempty(xhat)
        xhat = zeros(nx, 1);
    end
    if isempty(u_prev)
        u_prev = u_init;
    end
    if isempty(du_prev)
        du_prev = du_init;
    end

    if init_flag
        xhat = zeros(nx, 1);
        u_prev = u_init;
        du_prev = du_init;
        y_prev = y;
    end

    if all(L(:) == 0)
        L_use = pinv(C);
    else
        L_use = L;
    end

    validate_horizons(N1, N2, Nu);

    dy_meas = y - y_prev;

    if ~init_flag
        xhat = A * xhat + B * du_prev;
    end
    dyhat = C * xhat + D * du_prev;
    xhat = xhat + L_use * (dy_meas - dyhat);

    [PhiY, GammaY] = build_prediction_mats(A, B, C, D, N1, N2, Nu);
    F = PhiY * xhat + repmat(y, N2 - N1 + 1, 1);
    R = build_reference(y, r, N1, N2, alpha_vec);

    Qy = build_output_weight(wy, N2 - N1 + 1);
    Ru = build_move_weight(lam, Nu, nu);
    H = GammaY' * Qy * GammaY + Ru;
    f = -GammaY' * Qy * (R - F);

    [lb, ub, Aineq, bineq] = build_constraints(dumin, dumax, umin, umax, Nu, u_prev, nu);
    dU = solve_qp(H, f, lb, ub, Aineq, bineq, u_prev, umin, umax, use_quadprog, nu);

    du = dU(1:nu);
    u_prev = u_prev + du;
    du_prev = du;
    y_prev = y;
    u = u_prev;
    init_flag = false;
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

function [lb, ub, Aineq, bineq] = build_constraints(dumin, dumax, umin, umax, Nu, u_prev, nu)
    lb = repmat(dumin(:), Nu, 1);
    ub = repmat(dumax(:), Nu, 1);

    S = kron(tril(ones(Nu)), eye(nu));
    u_stack = repmat(u_prev(:), Nu, 1);
    umin_stack = repmat(umin(:), Nu, 1);
    umax_stack = repmat(umax(:), Nu, 1);

    Aineq = [S; -S];
    bineq = [umax_stack - u_stack;
            -(umin_stack - u_stack)];
end

function dU = solve_qp(H, f, lb, ub, Aineq, bineq, u_prev, umin, umax, use_quadprog, nu)
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
    u_seq = S * dU + repmat(u_prev(:), Nu, 1);
    u_seq = min(max(u_seq, repmat(umin(:), Nu, 1)), repmat(umax(:), Nu, 1));

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
