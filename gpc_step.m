function du = gpc_step(y, r, A, B, N1, N2, Nu, alpha, lam, reset, C, mode, u_applied, solve_when_inactive)
%GPC_STEP  One-step 3x3 GPC move for a CARIMA model with mode-aware tracking.
%   du = gpc_step(y, r, A, B, N1, N2, Nu, alpha, lam, reset, C)
%   du = gpc_step(..., mode, u_applied)
%   du = gpc_step(..., mode, u_applied, solve_when_inactive)
%
%   This implementation keeps the CARIMA predictor synchronized even when
%   GPC is inactive. The internal histories are always updated from the
%   measured output and the actual actuator signal applied to the plant.
%
%   Inputs:
%     y, r      - 3x1 output and reference vectors
%     A, B, C   - CARIMA polynomial matrices
%     mode      - 1: GPC active, 2: manual active, 3: other controller active
%     u_applied - actual absolute actuator input currently applied to plant
%     solve_when_inactive - if true, still compute GPC candidate while inactive
%
%   Notes:
%     - du_hist always stores actual applied input increments, never an
%       unused GPC move.
%     - If u_applied is omitted, the function falls back to legacy behavior:
%       when GPC was previously active, the last GPC move is assumed applied;
%       otherwise zero increment is assumed.
%     - The returned du is only nonzero when mode == GPC_ACTIVE.

    persistent K y_prev dy_hist du_hist e_hist ...
               u_prev_applied du_prev_assumed ...
               last_A last_B last_C last_N1 last_N2 last_Nu last_lam init_flag

    modes = gpc_controller_modes();

    if nargin < 12 || isempty(mode)
        mode = modes.GPC_ACTIVE;
    end
    has_u_applied = (nargin >= 13) && ~isempty(u_applied);
    if nargin < 14 || isempty(solve_when_inactive)
        solve_when_inactive = false;
    end

    % Reset handling
    if isempty(init_flag)
        init_flag = true;
    end
    if reset
        na_reset = size(A, 3) - 1;
        nb_reset = size(B, 3) - 1;
        nc_reset = size(C, 3) - 1;

        y_prev = zeros(3, 1);
        dy_hist = zeros(3, max(na_reset, 1));
        du_hist = zeros(3, nb_reset + 1);
        e_hist = zeros(3, max(nc_reset, 1));
        u_prev_applied = zeros(3, 1);
        du_prev_assumed = zeros(3, 1);
        init_flag = true;
        du = zeros(3,1);
        return;
    end

    % Basic checks
    y = y(:);
    r = r(:);
    assert(numel(y) == 3 && numel(r) == 3, 'y and r must be 3x1 vectors.');

    if has_u_applied
        u_applied = u_applied(:);
        assert(numel(u_applied) == 3, 'u_applied must be a 3x1 vector.');
    end

    mode = validate_gpc_mode(mode, modes);

    if isscalar(alpha)
        alpha_vec = repmat(alpha, 3, 1);
    elseif numel(alpha) == 3
        alpha_vec = alpha(:);
    else
        assert(false, 'alpha must be a scalar or a 3-element vector.');
    end

    if isscalar(lam)
        lam_vec = repmat(lam, 3, 1);
    elseif numel(lam) == 3
        lam_vec = lam(:);
    else
        assert(false, 'lam must be a scalar or a 3-element vector.');
    end

    % Orders
    na = size(A, 3) - 1;
    nb = size(B, 3) - 1;
    nc = size(C, 3) - 1;

    % Initialize cache tracking
    if isempty(last_A)
        last_A = A;
        last_B = B;
        last_C = C;
        last_N1 = N1;
        last_N2 = N2;
        last_Nu = Nu;
        last_lam = lam_vec;
    end

    % Recompute gain matrix only if A/B/horizons/lambda changed.
    if init_flag || isempty(K) || ...
            ~isequal(A, last_A) || ~isequal(B, last_B) || ...
            N1 ~= last_N1 || N2 ~= last_N2 || Nu ~= last_Nu || ~isequal(lam_vec, last_lam)

        G = gmat(A, B, N1, N2, Nu);

        Lam = kron(eye(Nu), diag(lam_vec));
        K = (G' * G + Lam) \ G';

        last_A = A;
        last_B = B;
        last_N1 = N1;
        last_N2 = N2;
        last_Nu = Nu;
        last_lam = lam_vec;
    end

    % Track C too (not used for K cache, but used in predictor/F)
    if ~isequal(C, last_C)
        last_C = C;
    end

    % Initialize histories on first call
    if init_flag || isempty(y_prev)
        y_prev = y;
    end
    if init_flag || isempty(dy_hist)
        dy_hist = zeros(3, max(na, 1));      % [dy(k-1), dy(k-2), ...]
    end
    if init_flag || isempty(du_hist)
        du_hist = zeros(3, nb + 1);          % [du(k-1), du(k-2), ...]
    end
    if init_flag || isempty(e_hist)
        e_hist = zeros(3, max(nc, 1));       % [e(k-1), e(k-2), ...]
    end
    if init_flag || isempty(u_prev_applied)
        if has_u_applied
            u_prev_applied = u_applied;
        else
            u_prev_applied = zeros(3, 1);
        end
    end
    if init_flag || isempty(du_prev_assumed)
        du_prev_assumed = zeros(3, 1);
    end

    if ~has_u_applied
        if mode == modes.GPC_ACTIVE
            u_applied = u_prev_applied + du_prev_assumed;
        else
            u_applied = u_prev_applied;
        end
    end

    % ----- Measurement update -----
    dy_k = y - y_prev;

    % Predictor histories always represent what actually reached the plant
    % on previous intervals. The current applied increment is saved only for
    % the next call, once it becomes past information.
    if init_flag
        du_applied_k = zeros(3, 1);
    else
        du_applied_k = u_applied - u_prev_applied;
    end

    % ----- Innovation update -----
    % Histories used here correspond to:
    %   dy_hist = [dy(k-1), dy(k-2), ...]
    %   du_hist = [du(k-1), du(k-2), ...]
    %   e_hist  = [e(k-1),  e(k-2),  ...]
    dyhat_k = predict_dy1_carima(A, B, C, dy_hist, du_hist, e_hist);
    e_k = dy_k - dyhat_k;

    % ----- Free response prediction -----
    % For the optimization at sample k, the current control increment is not
    % yet applied, so the free response uses du(k)=0.
    if na > 0
        dy_hist_use = [dy_k, dy_hist(:, 1:end-1)];
    else
        dy_hist_use = dy_hist(:, 1:1);  % dummy, ignored when na == 0
    end

    du_hist_use = [zeros(3, 1), du_hist(:, 1:end-1)];

    if nc > 0
        e_hist_use = [e_k, e_hist(:, 1:end-1)];
    else
        e_hist_use = e_hist(:, 1:1);    % dummy, ignored when nc == 0
    end

    F = fmat(A, B, C, N1, N2, y, dy_hist_use, du_hist_use, e_hist_use);

    % ----- Reference trajectory -----
    Rfull = zeros(N2 + 1, 3);
    Rfull(1, :) = y.';
    alpha_row = alpha_vec.';
    r_row = r.';

    for t = 2:N2 + 1
        Rfull(t, :) = alpha_row .* Rfull(t - 1, :) + (1 - alpha_row) .* r_row;
    end

    rows = N2 - N1 + 1;
    R = zeros(3 * rows, 1);
    for i = 1:3
        r_i = Rfull(N1 + 1:N2 + 1, i);
        r0 = (i - 1) * rows + 1;
        R(r0:r0 + rows - 1) = r_i;
    end

    % ----- Optional control optimization -----
    solve_gpc = (mode == modes.GPC_ACTIVE) || logical(solve_when_inactive);
    du_candidate = zeros(3, 1);
    if solve_gpc
        dU_star = K * (R - F);
        du_candidate(1) = dU_star(1);
        du_candidate(2) = dU_star(Nu + 1);
        du_candidate(3) = dU_star(2 * Nu + 1);
    end

    % ----- Output selection -----
    du = zeros(3, 1);
    if mode == modes.GPC_ACTIVE
        du = du_candidate;
    end

    % ----- Predictor/history update using actual plant input -----
    if nb >= 0
        du_hist = [du_applied_k, du_hist(:, 1:end-1)];
    end
    if na > 0
        dy_hist = [dy_k, dy_hist(:, 1:end-1)];
    end
    if nc > 0
        e_hist = [e_k, e_hist(:, 1:end-1)];
    end

    y_prev = y;
    u_prev_applied = u_applied;
    du_prev_assumed = du;
    init_flag = false;
end


function dyhat = predict_dy1_carima(A, B, C, dy_hist, du_hist, e_hist)
%PREDICT_DY1_CARIMA  One-step predictor for dy(k|k-1) in a MIMO CARIMA model.
%
%   Model:
%     A(z^-1)dy(k) = z^-1B(z^-1)du(k) + C(z^-1)e(k)
%
%   The predictor uses only past information:
%     dyhat(k|k-1) = -A1dy(k-1)-... + B0du(k-1)+... + C1e(k-1)+...
%
%   Histories expected:
%     dy_hist = [dy(k-1), dy(k-2), ...]
%     du_hist = [du(k-1), du(k-2), ...]   actual applied increments only
%     e_hist  = [e(k-1),  e(k-2),  ...]

    na = size(A,3) - 1;
    nb = size(B,3) - 1;
    nc = size(C,3) - 1;

    dyhat = zeros(3,1);

    for i = 1:3
        acc = 0;

        for a = 1:na
            acc = acc - squeeze(A(i,:,a+1)) * dy_hist(:,a);
        end

        for b = 0:nb
            acc = acc + squeeze(B(i,:,b+1)) * du_hist(:,b+1);
        end

        for c = 1:nc
            acc = acc + squeeze(C(i,:,c+1)) * e_hist(:,c);
        end

        dyhat(i) = acc;
    end
end


function mode = validate_gpc_mode(mode, modes)
    assert(isnumeric(mode) && isscalar(mode) && isfinite(mode) && mode == floor(mode), ...
        'mode must be an integer scalar.');
    assert(any(mode == [modes.GPC_ACTIVE, modes.MANUAL_ACTIVE, modes.OTHER_CONTROLLER_ACTIVE]), ...
        'mode must be 1 (GPC_ACTIVE), 2 (MANUAL_ACTIVE), or 3 (OTHER_CONTROLLER_ACTIVE).');
end


function modes = gpc_controller_modes()
    modes = struct( ...
        'GPC_ACTIVE', 1, ...
        'MANUAL_ACTIVE', 2, ...
        'OTHER_CONTROLLER_ACTIVE', 3);
end
