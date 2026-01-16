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
        last_lam = lam;
    end

    % Cache the gain matrix unless model/horizons/weight change.
    if init_flag || isempty(K) || isempty(last_A) || isempty(last_B) || ...
            ~isequal(A, last_A) || ~isequal(B, last_B) || ...
            N1 ~= last_N1 || N2 ~= last_N2 || Nu ~= last_Nu || lam ~= last_lam
        G = gmat_mimo(A, B, N1, N2, Nu);
        K = (G' * G + lam * eye(3 * Nu)) \ G';
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
    F = fmat_mimo(A, B, N1, N2, y, dy_hist_use, du_hist_use);

    % Reference trajectory over N1..N2 with exponential smoothing.
    Rfull = zeros(N2, 3);
    Rfull(1, :) = y.';
    for i = 2:N2
        alpha_row = alpha_vec.';
        Rfull(i, :) = alpha_row .* Rfull(i - 1, :) + (1 - alpha_row) .* r.';
    end
    rows = N2 - N1 + 1;
    R = zeros(3 * rows, 1);
    for i = 1:3
        r_i = Rfull(N1:N2, i);
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

function G = gmat_mimo(A, B, N1, N2, Nu)
    rows = N2 - N1 + 1;
    G = zeros(3 * rows, 3 * Nu);
    for i = 1:3
        for j = 1:3
            Aij = squeeze(A(i, j, :)).';
            Bij = squeeze(B(i, j, :)).';
            Gij = siso_gmat(Aij, Bij, N1, N2, Nu);
            r0 = (i - 1) * rows + 1;
            c0 = (j - 1) * Nu + 1;
            G(r0:r0+rows-1, c0:c0+Nu-1) = Gij;
        end
    end
end

function G = siso_gmat(A, B, N1, N2, Nu)
    na = numel(A) - 1;
    nb = numel(B) - 1;

    dy = zeros(1, N2 + na + 2);
    y  = zeros(1, N2 + 2);
    du_at = @(t) double(t == 0);

    for t = 1:N2
        acc = 0;
        for i = 1:na
            if (t - i) + 1 >= 1
                acc = acc - A(i+1) * dy((t - i) + 1);
            end
        end
        for j = 0:nb
            acc = acc + B(j+1) * du_at(t - 1 - j);
        end
        dy(t + 1) = acc;
        y(t + 1)  = y(t) + dy(t + 1);
    end

    g = y(2:N2+1);

    rows = N2 - N1 + 1;
    G = zeros(rows, Nu);
    for r = 1:rows
        for c = 1:Nu
            idx = N1 + r - c;
            if idx >= 1
                G(r, c) = g(idx);
            end
        end
    end
end

function F = fmat_mimo(A, B, N1, N2, yk, dy_hist, du_hist)
    rows = N2 - N1 + 1;
    F = zeros(3 * rows, 1);

    for i = 1:3
        dy_sim = zeros(3, N2);
        y_sim  = zeros(1, N2);
        y_prev = yk(i);

        for t = 1:N2
            dy_sum = 0;
            for j = 1:3
                Aij = squeeze(A(i, j, :)).';
                Bij = squeeze(B(i, j, :)).';
                na = numel(Aij) - 1;
                nb = numel(Bij) - 1;

                acc = 0;
                for a = 1:na
                    m = t - a;
                    if m <= 0
                        acc = acc - Aij(a+1) * dy_hist(i, -m + 1);
                    else
                        acc = acc - Aij(a+1) * dy_sim(j, m);
                    end
                end

                for b = 0:nb
                    n = t - 1 - b;
                    if n <= 0
                        acc = acc + Bij(b+1) * du_hist(j, -n + 1);
                    end
                end

                dy_sim(j, t) = acc;
                dy_sum = dy_sum + acc;
            end

            y_prev   = y_prev + dy_sum;
            y_sim(t) = y_prev;
        end

        Fi = y_sim(N1:N2).';
        r0 = (i - 1) * rows + 1;
        F(r0:r0+rows-1) = Fi;
    end
end
