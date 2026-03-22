clc; clear; 
% close all;

load("data/noise_40/GPC_params_arx_40.mat");
% load("data/noise-free/GPC_params.mat"); C = eye(3);
load("data/noise_40/IdMPC_val.mat");

N1 = 1;
Np = 200;
startIdx = 20;

u = u(startIdx : startIdx + Np - 1, :);
y = y_real(startIdx : startIdx + Np - 1, :);

dy = diff(y);
du = diff(u);

na = size(A,3) - 1;
nb = size(B,3) - 1;

% choose current sample in y
% k0 = na + 1;      % for your case, likely 13 if na = 12
k0 = nb + 1;

% current absolute output
yk = y(k0,:).';

% dy_hist(:,1) must be the most recent increment: y(k0)-y(k0-1)
dy_hist = dy(k0-1:-1:k0-na, :).';
% dy_hist = zeros(12, 3);
% dy_hist(1, :) = [-2.373067707357252e-05 -2.077815170175914e-05 9.153139220138165e-06];
% dy_hist = dy_hist.';

% free response: current move is zero
du_hist = zeros(3, nb+1);
% du_hist = zeros(12, 3);
% du_hist(2, :) = [-3.0000e-2 -3.0000e-2 3.0000e-2];
% du_hist = du_hist.';

% most recent past increments next
if nb > 0
    du_hist(:,2:end) = du(k0-1:-1:k0-nb, :).';
end

e_hist = 0;

F = fmat(A, B, C, N1, Np, yk, dy_hist, du_hist, e_hist);
plot([F(1:Np) F(Np+1:2*Np) F(2*Np+1:3*Np)])