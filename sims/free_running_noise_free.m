%% Free-running test of identified predictive model
clear; clc; close all;

load("data/noise-free/MPC_params_pem.mat");
load("data/noise-free/IdMPC_val.mat");

startIdx = 40;
N1 = 1;
Np = 200;

u = u(startIdx + N1 : startIdx + N1 + Np - 1, :);
y = y(startIdx + N1 : startIdx + N1 + Np - 1, :);

nx = size(C, 2);
ny = size(C, 1);

du = diff(u);
dy = diff(y);

dyhat = zeros(size(du, 1), ny);
yhat = zeros(size(y));
yhat(1, :) = y(1, :);

X = zeros(nx, 1);

for k = 1 : Np - 1
    Y = C * X;
    X = A * X + B * du(k, :).';
    dyhat(k, :) = Y.';
    yhat(k+1, :) = yhat(k, :) + dyhat(k, :);
end

%% Similarity index: FIT (%)
fitVal = zeros(1, ny);
rmseVal = zeros(1, ny);

for i = 1 : ny
    err = y(:, i) - yhat(:, i);
    fitVal(i) = 100 * (1 - norm(err, 2) / norm(y(:, i) - mean(y(:, i)), 2));
    rmseVal(i) = sqrt(mean(err.^2));
end

fitOverall = 100 * (1 - norm(y(:) - yhat(:), 2) / norm(y(:) - mean(y(:)), 2));

fprintf('Free-running similarity indices:\n');
for i = 1 : ny
    fprintf('Phase %d: FIT = %.2f %% , RMSE = %.6f\n', i, fitVal(i), rmseVal(i));
end
fprintf('Overall: FIT = %.2f %%\n', fitOverall);

%% Plot
phaseName = {'A相', 'B相', 'C相'};

fig = figure( ...
    'Name', 'Free-running test of predictive model', ...
    'Color', 'w', ...
    'NumberTitle', 'off');

tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

for k = 1 : ny
    ax = nexttile;
    hold(ax, 'on');

    h(1) = plot(y(:, k), '--', 'LineWidth', 1.5, 'Color', 'black');
    h(2) = plot(yhat(:, k), 'LineWidth', 1.5, 'Color', 'red');

    if k == 1
        hLegend = h;
    end

    grid(ax, 'on');
    ax.Color = 'w';
    ax.XColor = 'k';
    ax.YColor = 'k';
    ax.GridColor = [0.75 0.75 0.75];
    ax.GridAlpha = 0.35;
    ax.FontSize = 12;
    ax.LineWidth = 1;

    ylabel([phaseName{k}, ' 阻抗'], ...
        'Interpreter', 'tex', 'Color', 'k', 'FontSize', 12);
    xlabel('采样点', 'Color', 'k', 'FontSize', 12);

    title(sprintf('%s, FIT = %.2f%%', phaseName{k}, fitVal(k)), ...
        'Color', 'k', 'FontSize', 13);
end

lgd = legend(hLegend, {'原始轨迹', '自由运行轨迹'}, ...
    'Orientation', 'vertical', ...
    'Box', 'off', ...
    'FontSize', 11, ...
    'TextColor', 'k', ...
    'Interpreter', 'none');
lgd.Layout.Tile = 'east';

sgtitle(sprintf('Free-running test, Overall FIT = %.2f%%', fitOverall), ...
    'Interpreter', 'none', 'Color', 'k', 'FontSize', 14);