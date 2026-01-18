function du = gpc_step(y, r, A, B, N1, N2, Nu, alpha, lam, reset)
%GPC_STEP  One-step 3x3 GPC move for a CARIMA/ARIMA model with incremental control.
%   Codegen-friendly version for MATLAB Function blocks (no cell arrays).
%
%   A and B must be numeric arrays sized:
%     A: 3x3x(na+1) with A(:,:,1) == 1
%     B: 3x3x(nb+1)
%
%   Computes free response F and reference trajectory R, then applies
%   ΔU* = (G'G+λI)^{-1} G' (R-F) and outputs Δu(k) for each input.
%   Maintains histories of Δy and Δu between calls for the predictor.
    persistent K y_prev dy_hist du_hist last_A last_B last_N1 last_N2 last_Nu last_lam init_flag

    % Reset handling for MATLAB Function block.
    if isempty(init_flag)
        init_flag = true;
    end
    if reset
        init_flag = true;
        du = zeros(3, 1);
        return;
    end

    % Initialize cached parameters to avoid use-before-assign errors.
    if isempty(last_A)
        last_A = A;
        last_B = B;
        last_N1 = N1;
        last_N2 = N2;
        last_Nu = Nu;
        if isscalar(lam)
            lam = repmat(lam, [1 3]);
        end
        lam = lam(:).';
        last_lam = lam;
    end

    % Cache the gain matrix unless model/horizons/weight change.
    if init_flag || isempty(K) || isempty(last_A) || isempty(last_B) || ...
            ~isequal(A, last_A) || ~isequal(B, last_B) || ...
            N1 ~= last_N1 || N2 ~= last_N2 || Nu ~= last_Nu || lam ~= last_lam
        G = gmat(A, B, N1, N2, Nu);
        Lam = eye(3 * Nu);
        for i = 1:3
            r0 = (i - 1) * Nu + 1;
            Lam(r0:r0+Nu-1, r0:r0+Nu-1) = lam(i) * Lam(r0:r0+Nu-1, r0:r0+Nu-1);
        end
        K = (G' * G + Lam) \ G';
        last_A = A;
        last_B = B;
        last_N1 = N1;
        last_N2 = N2;
        last_Nu = Nu;
        last_lam = lam;
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

    % Initialize state history on first call.
    if init_flag || isempty(y_prev)
        y_prev = y;
    end
    na = size(A, 3) - 1;
    nb = size(B, 3) - 1;
    if init_flag || isempty(dy_hist)
        dy_hist = zeros(3, max(na, 1));
    end
    if init_flag || isempty(du_hist)
        du_hist = zeros(3, nb + 1);
    end
    init_flag = false;

    % Current output increment.
    dy_k = y - y_prev;

    % Build histories that include the current sample for free response.
    if na > 0
        dy_hist_use = [dy_k, dy_hist(:, 1:end-1)];
    else
        dy_hist_use = dy_hist(:, 1:1);
    end
    % Free response assumes Δu(k) = 0 (future moves handled by G*ΔU).
    du_hist_use = [zeros(3, 1), du_hist(:, 1:end-1)];

    % Free response over N1..N2.
    F = fmat(A, B, N1, N2, y, dy_hist_use, du_hist_use);

    % Reference trajectory over N1..N2 with exponential smoothing.
    Rfull = zeros(N2+1, 3);
    Rfull(1, :) = y.';
    for i = 2:N2+1
        alpha_row = alpha_vec.';
        Rfull(i, :) = alpha_row .* Rfull(i - 1, :) + (1 - alpha_row) .* r.';
    end
    rows = N2 - N1 + 1;
    R = zeros(3 * rows, 1);
    for i = 1:3
        r_i = Rfull(N1+1:N2+1, i);
        r0 = (i - 1) * rows + 1;
        R(r0:r0+rows-1) = r_i;
    end

    % Optimal move sequence and current move for each input.
    dU_star = K * (R - F);
    du = zeros(3, 1);
    du(1) = dU_star(1);
    du(2) = dU_star(Nu + 1);
    du(3) = dU_star(2 * Nu + 1);

    % Update histories for next call.
    du_hist = [du, du_hist(:, 1:end-1)];
    if na > 0
        dy_hist = [dy_k, dy_hist(:, 1:end-1)];
    end
    y_prev = y;
end
