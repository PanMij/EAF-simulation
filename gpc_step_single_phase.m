function du = gpc_step_single_phase(y, r, A, B, N1, N2, Nu, alpha, lam, reset)
%GPC_STEP  One-step GPC move for a CARIMA/ARIMA model with incremental control.
%   Computes the free response F and reference trajectory R, then applies the
%   standard GPC law ΔU* = (G'G+λI)^{-1} G' (R-F) and outputs Δu(k).
%   Maintains histories of Δy and Δu between calls for the predictor.
    persistent K y_prev dy_hist du_hist last_key init_flag

    % Reset handling for MATLAB Function block.
    if isempty(init_flag)
        init_flag = true;
    end
    if reset
        init_flag = true;
        du = 0;
        return;
    end

    % Cache the gain matrix unless model/horizons/weight change.
    key = {A, B, N1, N2, Nu, lam};
    if isempty(last_key)
        last_key = key;
    end
    if init_flag || isempty(K) || ~isequal(key, last_key)
        G = gmat_single_phase(A, B, N1, N2, Nu);
        K = (G' * G + lam * eye(Nu)) \ G';
        last_key = key;
    end

    na = numel(A) - 1;
    nb = numel(B) - 1;

    % Initialize state history on first call.
    if init_flag || isempty(y_prev)
        y_prev = y;
    end
    if init_flag || isempty(dy_hist)
        dy_hist = zeros(1, max(na, 0));
    end
    if init_flag || isempty(du_hist)
        du_hist = zeros(1, nb + 1);
    end
    init_flag = false;

    % Current output increment.
    dy_k = y - y_prev;

    % Build histories that include the current sample for free response.
    if na > 0
        dy_hist_use = [dy_k, dy_hist(1:end-1)];
    else
        dy_hist_use = [];
    end
    if nb >= 0
        % Free response assumes Δu(k) = 0 (future moves handled by G*ΔU).
        du_hist_use = [0, du_hist(1:end-1)];
    end

    % Free response over N1..N2.
    F = fmat_single_phase(A, B, N1, N2, y, dy_hist_use, du_hist_use);

    % Reference trajectory over N1..N2 with exponential smoothing.
    Rfull = zeros(N2, 1);
    Rfull(1) = y;
    for i = 2:N2
        Rfull(i) = alpha * Rfull(i - 1) + (1 - alpha) * r;
    end
    R = Rfull(N1:N2);

    % Optimal move sequence and current move.
    dU_star = K * (R - F);
    du = dU_star(1);

    % Update histories for next call.
    du_hist = [du, du_hist(1:end-1)];
    if na > 0
        dy_hist = [dy_k, dy_hist(1:end-1)];
    end
    y_prev = y;
end
