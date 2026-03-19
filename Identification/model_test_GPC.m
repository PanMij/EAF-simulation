clc;clear;
close all;

load("data/noise_40/GPC_params_ss2car.mat");
load("data/noise_40/IdMPC_val.mat");
startIdx = 20;
N1 = 0;
Np = 2000;
u = u(startIdx + N1:startIdx + N1 + Np - 1, :);
y = y_real(startIdx + N1:startIdx + N1 + Np - 1, :);
y = diff(y);
u = diff(u);

A_flat = reshape(A(:,:,2:end), 3, []);
B_flat = reshape(B, 3, []);
theta = [A_flat.'; B_flat.'];
na = size(A, 3) - 1;
nb = size(B, 3) - 1;
nc = 0;
d = 0;

[ysim, esim, k0] = carima_free_run_predict_mimo(y, u, theta, na, nb, nc, d);
for k = 1 : 3
    subplot(3, 1, k);
    plot([ysim(:, k) y(:, k)]);
    legend("ysim", "y");
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