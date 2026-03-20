clc;clear;close all;

%% 1. Load + format data
data_path_est = fullfile("data", "noise-free", "IdMPC_est.mat");
data_path_val = fullfile("data", "noise-free", "IdMPC_val.mat");

Ts = 0.02;
load(data_path_est);
% Drop the first sample
startIdx = 20;
t = t(startIdx:end, :);
u = u(startIdx:end, :);
y = y(startIdx:end, :);
% y_real = y_real(startIdx:end, :);
l_hy = l_hy(startIdx:end, :);
figure;
subplot(4, 1, 1);
plot(t, u);
title('Excitation Signal'); xlabel('t'); ylabel('u(t)');
subplot(4, 1, 2);
plot(t, y);
title('Plant Output'); xlabel('t'); ylabel('y(t)');
legend("y1", "y2", "y3");
% subplot(4, 1, 3);
% plot(t, y_real);
% title('Plant Output (Noise-free)'); xlabel('t'); ylabel('y(t)');
% legend("y1", "y2", "y3");
subplot(4, 1, 4);
plot(t, l_hy);
title('Hydraulic Output'); xlabel('t'); ylabel('h');
legend("h1", "h2", "h3");

%% 2. Create `iddata` + detrend
y = lowpass(y, 40, 1000);
y = y(startIdx:end-startIdx, :);
u = u(startIdx:end-startIdx, :);
y = diff(y);
% y = diff(y_real);
u = diff(u);
data = iddata(y, u, Ts, "TimeUnit", "s");
data.InputName = {"v_a", "v_b", "v_c"};
data.OutputName = {"Z_a", "Z_b", "Z_c"};
% remove mean
% data = detrend(data, 0);
figure;
idplot(data);

%% 3. Split estimation/validation
% N = size(data.OutputData, 1);
% Ne = floor(0.7*N);
% 
% data_est = data(1:Ne);
% data_val = data(Ne+1:end);
data_est = data;
load(data_path_val);
t = t(startIdx:end, :);
u = u(startIdx:end, :);
y = y(startIdx:end, :);
% y = y_real(startIdx:end, :);
l_hy = l_hy(startIdx:end, :);
u = diff(u);
y = diff(y);
data_val = iddata(y, u, Ts, "TimeUnit", "s");
data_val.InputName = {"v_a", "v_b", "v_c"};
data_val.OutputName = {"Z_a", "Z_b", "Z_c"};
figure;
idplot(data_val);

%% 4. Estimate CARIMA model
y_est = data_est.y;   % these are already diff(y)
u_est = data_est.u;   % these are already diff(u)
y_val = data_val.y;
u_val = data_val.u;
n_a = 1:10;
n_b = 1:10;
n_c = 0:2;    % <-- noise polynomial order C(q^-1) = 1 + c1 q^-1 + ... + cnc q^-nc
% n_c = 0;
% n_a = 2;
% n_b = 2;
d   = 0;      % input delay
maxELSIter = 20;
tolELS = 1e-4;
fit_best = -Inf;
theta_best = [];
na_best = 0; nb_best = 0; nc_best = 0;
for na = n_a
    for nb = n_b
        for nc = n_c
            try
                arx_na = diag([na na na]);
                % arx_na = na * ones(3, 3);
                arx_nb = nb * ones(3, 3);
                arx_nk = ones(3, 3);
                armax_nc = nc * ones(3, 1);
                % sys_arx = arx(data_est, [arx_na arx_nb arx_nk]);
                % [~, fit, ~] = compare(data_val, sys_arx);
                sys_armax = armax(data_est, [arx_na, arx_nb, armax_nc, arx_nk]);
                [~, fit, ~] = compare(data_val, sys_armax);
                if fit > fit_best
                    fit_best = fit;
                    % bestSys = sys_arx;
                    bestSys = sys_armax;
                    na_best = na;
                    nb_best = nb;
                    nc_best = nc;
                    % info_best = info;
                end
                fprintf("\nOrders: na=%d, nb=%d, nc=%d, fit=%.2f%%\n", ...
                    na, nb, nc, fit);
            catch ME
                fprintf("na=%d, nb=%d, nc=%d failed: %s\n", na, nb, nc, ME.message);
            end
        end
    end
end
fprintf("\nBest orders: na=%d, nb=%d, nc=%d, fit=%.2f%%\n", ...
    na_best, nb_best, nc_best, fit_best);

A = zeros([3, 3, na_best + 1]);
for i = 1:3
    for j = 1:3
        A(i, j, :) = bestSys.A{i, j};
    end
end
B = zeros([3, 3, nb_best]);
for i = 1:3
    for j = 1:3
        B(i, j, :) = bestSys.B{i, j}(2:end);
    end
end
C = zeros([3, 3, nc_best + 1]);
for i = 1:3
    C(i, i, :) = bestSys.C{i};
end

%% 5. Validate the model
y = data_val.y;   % diff(y)
u = data_val.u;   % diff(u)
[y_hat, e_hat, k0] = carima_osa_predict_mimo(y, u, theta_best, na_best, nb_best, nc_best, d);
figure;
for i = 1:3
    fit_i = 100 * (1 - norm(y(k0:end,i) - y_hat(k0:end,i)) / ...
        norm(y(k0:end,i) - mean(y(k0:end,i))));
    subplot(3,1,i);
    plot(y(:,i)); hold on;
    plot(y_hat(:,i)); hold off;
    grid on;
    legend("\Delta y", sprintf("\\Delta y_{1-step}, fit=%.2f%%", fit_i));
    ylabel(sprintf("\\Delta y_%d",i));
end
xlabel("sample");
% reconstruct original y from predicted delta-y (one-step style)
load(data_path_val, "y");
y_raw = y_real(startIdx:end, :);
y_model = nan(size(y_raw));
% Reconstruct from k0 onward using cumulative sum of predicted delta-y
y_model(k0,:) = y_raw(k0,:);
for k = k0+1:size(y_hat,1)
    y_model(k,:) = y_model(k-1,:) + y_hat(k,:);
end
figure;
for i = 1:3
    idx = k0:size(y_hat,1);
    fit_i = 100 * (1 - norm(y_raw(idx,i) - y_model(idx,i)) / ...
        norm(y_raw(idx,i) - mean(y_raw(idx,i))));
    subplot(3,1,i);
    plot(y_raw(:,i)); hold on;
    plot(y_model(:,i)); hold off;
    grid on;
    legend("y", sprintf("y_{model}, fit=%.2f%%", fit_i));
    ylabel(sprintf("y_%d",i));
end
xlabel("sample");

%% Free run test
% [ysim, esim, k0] = carima_free_run_predict_mimo(y_val, u_val, theta_best, na, nb, nc, d);


%% 6. GPC parameters
% theta rows are ordered in 3x3 blocks:
% [A1 ... Ana, B0 ... Bnb, C1 ... Cnc]
ThetaBlk = reshape(theta_best', 3, 3, []);
A = cat(3, eye(3), ThetaBlk(:,:,1:na_best));
B = ThetaBlk(:,:,na_best+1 : na_best+nb_best+1);
if nc_best > 0
    C = cat(3, eye(3), ThetaBlk(:,:,na_best+nb_best+2 : na_best+nb_best+1+nc_best));
else
    C = eye(3);   % C(q^-1) = I
end
% save("data/GPC_params.mat", "A", "B", "C", "na_best", "nb_best", "nc_best", "d", "Ts");


%% Helper functions for MIMO CARIMA/ARMAX estimation and prediction
function [theta, yhat, ehat, info] = carima_els_mimo(y, u, na, nb, nc, d, maxIter, tol)
% Extended Least Squares for MIMO incremental CARIMA/ARMAX:
% y(k) = -A1 y(k-1)-...-Ana y(k-na) + B0 u(k-1-d)+...+Bnb u(k-1-d-nb)
%        + C1 e(k-1)+...+Cnc e(k-nc) + e(k)
    [N, ny] = size(y);
    nu = size(u,2);
    nAy = ny * na;
    nBu = nu * (nb + 1);
    nCe = ny * nc;
    nTheta = nAy + nBu + nCe;
    k0 = max([na, nb + d + 1, nc]) + 1;
    % --- ARX initialization (C = I, i.e., no MA terms) ---
    theta = zeros(nTheta, ny);
    theta_arx = arx_ls_mimo(y, u, na, nb, d);
    theta(1:nAy+nBu, :) = theta_arx;
    % Initial residuals from ARX predictor
    [yhat, ehat] = carima_osa_predict_mimo(y, u, theta, na, nb, nc, d);
    info.cost = nan(maxIter,1);
    info.relchg = nan(maxIter,1);
    for iter = 1:maxIter
        S = zeros(nTheta, nTheta);
        rhs = zeros(nTheta, ny);
        for k = k0:N
            phi_y = reshape(-y(k-1:-1:k-na, :).', [], 1);  % ny*na x 1
            phi_u = reshape( u(k-1-d:-1:k-1-d-nb, :).', [], 1); % nu*(nb+1) x 1
            if nc > 0
                phi_e = reshape(ehat(k-1:-1:k-nc, :).', [], 1); % ny*nc x 1
                phi = [phi_y; phi_u; phi_e];
            else
                phi = [phi_y; phi_u];
            end
            S = S + phi * phi.';
            rhs = rhs + phi * y(k,:);
        end
        % small regularization for numerical robustness
        % theta_new = (S + 1e-6 * eye(nTheta)) \ rhs;
        theta_new = S \ rhs;
        [yhat_new, ehat_new] = carima_osa_predict_mimo(y, u, theta_new, na, nb, nc, d);
        idx = k0:N;
        cost = norm(ehat_new(idx,:), "fro")^2;
        relchg = norm(theta_new - theta, "fro") / max(1, norm(theta, "fro"));
        info.cost(iter) = cost;
        info.relchg(iter) = relchg;
        theta = theta_new;
        yhat = yhat_new;
        ehat = ehat_new;
        if relchg < tol
            break;
        end
    end
    info.nIter = iter;
    info.cost = info.cost(1:iter);
    info.relchg = info.relchg(1:iter);
end

function theta_arx = arx_ls_mimo(y, u, na, nb, d)
% Least-squares ARX initializer (no noise polynomial terms)
% Solves Y = Phi * Theta by MATLAB backslash (more stable than normal equations)

    [N, ny] = size(y);
    nu = size(u,2);

    nAy = ny * na;
    nBu = nu * (nb + 1);
    nTheta = nAy + nBu;

    k0 = max([na, nb + d + 1]) + 1;
    nRow = N - k0 + 1;

    Phi = zeros(nRow, nTheta);
    Y   = zeros(nRow, ny);

    row = 0;
    for k = k0:N
        row = row + 1;

        phi_y = reshape(-y(k-1:-1:k-na, :).', [], 1);
        phi_u = reshape( u(k-1-d:-1:k-1-d-nb, :).', [], 1);
        phi   = [phi_y; phi_u];

        Phi(row, :) = phi.';
        Y(row, :)   = y(k, :);
    end

    % Solve min ||Phi * Theta - Y||_F
    theta_arx = Phi \ Y;

    % fprintf('ARX: size(Phi) = [%d, %d], cond(Phi) = %.3e\n', ...
    % size(Phi,1), size(Phi,2), cond(Phi));
end

function [yhat, ehat, k0] = carima_osa_predict_mimo(y, u, theta, na, nb, nc, d)
% One-step-ahead predictor for MIMO ARMAX/CARIMA incremental model
% Uses recursively computed innovations ehat in the C(q^-1) term.
    [N, ny] = size(y);
    nu = size(u,2);
    nAy = ny * na;
    nBu = nu * (nb + 1);
    nCe = ny * nc;
    nTheta = nAy + nBu + nCe;
    if size(theta,1) ~= nTheta || size(theta,2) ~= ny
        error("theta size mismatch. Expected [%d x %d], got [%d x %d].", ...
            nTheta, ny, size(theta,1), size(theta,2));
    end
    yhat = nan(N, ny);
    ehat = zeros(N, ny);
    k0 = max([na, nb + d + 1, nc]) + 1;
    for k = k0:N
        phi_y = reshape(-y(k-1:-1:k-na, :).', [], 1);
        phi_u = reshape( u(k-1-d:-1:k-1-d-nb, :).', [], 1);
        if nc > 0
            phi_e = reshape(ehat(k-1:-1:k-nc, :).', [], 1);
            phi = [phi_y; phi_u; phi_e];
        else
            phi = [phi_y; phi_u];
        end
        yhat(k,:) = (phi.' * theta);   % 1 x ny
        ehat(k,:) = y(k,:) - yhat(k,:);
    end
end

function [ysim, esim, k0] = carima_free_run_predict_mimo(y, u, theta, na, nb, nc, d, kStart)
% Free-run simulation for MIMO incremental CARIMA/ARMAX model
%
% Model form (here y and u are already differenced signals):
%   y(k) = -A1 y(k-1) - ... - Ana y(k-na) ...
%          + B0 u(k-1-d) + ... + Bnb u(k-1-d-nb) ...
%          + C1 e(k-1) + ... + Cnc e(k-nc) + e(k)
%
% Free-run idea:
%   - Before kStart: use measured y as warm-up
%   - From kStart onward: use simulated ysim recursively
%   - Future innovations are set to zero, i.e. e(k)=0 for k >= kStart
%   - For the C(q^-1) term, only pre-kStart residuals are used as warm-up;
%     after kStart, future residuals are taken as zero
%
% Inputs:
%   y      : N x ny, measured differenced outputs (e.g. diff(y_raw))
%   u      : N x nu, measured differenced inputs  (e.g. diff(u_raw))
%   theta  : nTheta x ny
%   na,nb,nc,d : model orders
%   kStart : simulation start index; if omitted, use k0
%
% Outputs:
%   ysim   : N x ny, free-run simulated differenced outputs
%   esim   : N x ny, simulation error y - ysim (for evaluation only)
%   k0     : minimum valid start index
%
% Notes:
%   1) This is free-run on the differenced signal y = diff(y_raw).
%   2) If nc > 0, the noise-model part is only used for warm-up via OSA residuals
%      before kStart. After kStart, innovations are set to zero.
%   3) For deterministic control prediction, this is usually more meaningful than OSA.

    [N, ny] = size(y);
    nu = size(u, 2);

    nAy = ny * na;
    nBu = nu * (nb + 1);
    nCe = ny * nc;
    nTheta = nAy + nBu + nCe;

    if size(theta,1) ~= nTheta || size(theta,2) ~= ny
        error("theta size mismatch. Expected [%d x %d], got [%d x %d].", ...
            nTheta, ny, size(theta,1), size(theta,2));
    end

    if any(~isfinite(y), 'all')
        error("Input y contains NaN or Inf.");
    end
    if any(~isfinite(u), 'all')
        error("Input u contains NaN or Inf.");
    end
    if any(~isfinite(theta), 'all')
        error("theta contains NaN or Inf.");
    end

    % Minimum valid index
    k0 = max([na, nb + d + 1, nc]) + 1;

    if nargin < 8 || isempty(kStart)
        kStart = k0;
    end

    if kStart < k0
        error("kStart must be >= k0 = %d.", k0);
    end
    if kStart > N
        error("kStart exceeds data length.");
    end

    % ---- Warm-up residuals from OSA predictor ----
    % These are only used for indices before kStart
    if nc > 0
        [~, ehat_osa, ~] = carima_osa_predict_mimo(y, u, theta, na, nb, nc, d);
    else
        ehat_osa = zeros(N, ny);
    end

    % ---- Initialize free-run output ----
    ysim = nan(N, ny);

    % Before kStart, copy measured output as warm-up
    % ysim(1:kStart-1, :) = y(1:kStart-1, :);
    % ysim(1:kStart-1, :) = 0;

    % ---- Free-run recursion ----
    for k = 1:N
        % Past outputs:
        % use simulated outputs in free-run region
        phi_y = zeros(nAy, 1);
        if k - na > 0
            phi_y = reshape(-ysim(k-1:-1:k-na, :).', [], 1);
        else
            phi_y(1:(k-1)*ny) = reshape(-ysim(k-1:-1:1, :).', [], 1);
        end

        % Past inputs:
        % still use measured input sequence
        phi_u = zeros(nBu, 1);
        if k - 1 - d - nb > 0
            phi_u = reshape(u(k-1-d:-1:k-1-d-nb, :).', [], 1);
        else
            phi_u(1:(k-1-d)*nu) = reshape(u(k-1-d:-1:1, :).', [], 1);
        end

        % Past residuals for C(q^-1):
        % before kStart -> use OSA residual warm-up
        % from kStart onward -> set future innovations to zero
        if nc > 0
            phi_e_mat = zeros(nc, ny);
            for ell = 1:nc
                kk = k - ell;
                if kk < kStart
                    phi_e_mat(ell, :) = ehat_osa(kk, :);
                else
                    phi_e_mat(ell, :) = zeros(1, ny);
                end
            end
            phi_e = reshape(phi_e_mat.', [], 1);
            phi = [phi_y; phi_u; phi_e];
        else
            phi = [phi_y; phi_u];
        end

        if any(~isfinite(phi))
            error("phi contains NaN/Inf at k = %d.", k);
        end

        ysim(k, :) = phi.' * theta;

        if any(~isfinite(ysim(k,:)))
            error("ysim becomes NaN/Inf at k = %d.", k);
        end
    end

    % Simulation error (only for evaluation)
    esim = y - ysim;
end