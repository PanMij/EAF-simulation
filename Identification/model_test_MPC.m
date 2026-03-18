clc;clear;close all;

load("data/noise_40/MPC_params.mat");
load("data/noise_40/IdMPC_val.mat");
startIdx = 20;
N1 = 1;
Np = 500;
u = u(startIdx + N1:startIdx + N1 + Np - 1, :);
y = y_real(startIdx + N1:startIdx + N1 + Np - 1, :);

% y_smooth = smoothdata(y, "sgolay");
% figure;
% subplot(3, 1, 1);
% plot([y(:, 1) y_smooth(:, 1)]);
% subplot(3, 1, 2);
% plot([y(:, 2) y_smooth(:, 2)]);
% subplot(3, 1, 3);
% plot([y(:, 3) y_smooth(:, 3)]);

% y_med = medfilt1(y, 5);
% figure;
% subplot(3, 1, 1);
% plot([y(:, 1) y_med(:, 1)]);
% subplot(3, 1, 2);
% plot([y(:, 2) y_med(:, 2)]);
% subplot(3, 1, 3);
% plot([y(:, 3) y_med(:, 3)]);

% fs = 1000;          % sampling frequency
% fpass = 40;         % keep content below 40 Hz
% y_lp = lowpass(y, fpass, fs);
% figure;
% subplot(3, 1, 1);
% plot([y(:, 1) y_lp(:, 1)]);
% subplot(3, 1, 2);
% plot([y(:, 2) y_lp(:, 2)]);
% subplot(3, 1, 3);
% plot([y(:, 3) y_lp(:, 3)]);

nx = size(C, 2);
nu = size(C, 1);

du = diff(u);
dy = diff(y);

dyhat = zeros(size(dy));
yhat = zeros(size(y));
yhat(1, :) = y(1, :);

X = zeros(nx, 1);
Y = zeros(nu, 1);
for k = 1 : Np - 1
    Y = C * X;
    X = A * X + B * du(k, :).';
    dyhat(k, :) = Y.';
    yhat(k+1, :) = yhat(k, :) + dyhat(k, :);
end

% subplot(3, 1, 1);
% plot([dy(:, 1) dyhat(:, 1)]);
% subplot(3, 1, 2);
% plot([dy(:, 2) dyhat(:, 2)]);
% subplot(3, 1, 3);
% plot([dy(:, 3) dyhat(:, 3)]);

subplot(3, 1, 1);
plot([y(:, 1) yhat(:, 1)]);
subplot(3, 1, 2);
plot([y(:, 2) yhat(:, 2)]);
subplot(3, 1, 3);
plot([y(:, 3) yhat(:, 3)]);