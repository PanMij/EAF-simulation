%% Run model and plot all logged signals with white background
clear; clc; close all;

model = 'arc_test';

load_system(model);
set_param(model,'SignalLogging','on');

simOut = sim(model, 'ReturnWorkspaceOutputs', 'on');
logs = simOut.logsout;

n = logs.numElements;
if n == 0
    error('No logged signals were found in logsout.');
end

fig = figure( ...
    'Name', ['Logged results from model: ', model], ...
    'Color', 'w', ...
    'NumberTitle', 'off');

tiledlayout(n,1,'TileSpacing','compact','Padding','compact');

signame = ["电弧电流 (A)" "电弧阻抗 (Ω)" "二次侧电压 (V)" "电弧电压 (V)"];
for k = [3 4 1 2]
    sig = logs.getElement(k);
    vals = sig.Values;

    ax = nexttile;
    plot(vals.Time, vals.Data, 'LineWidth', 1.5);

    grid(ax,'on');
    ax.Color = 'w';
    ax.XColor = 'k';
    ax.YColor = 'k';
    ax.GridColor = [0.75 0.75 0.75];
    ax.GridAlpha = 0.35;
    ax.FontSize = 12;
    ax.LineWidth = 1;

    % title(sig.Name, 'Interpreter','none', 'Color','k', 'FontSize',13);
    ylabel(signame(k), 'Interpreter','none', 'Color','k', 'FontSize',12);
    xlabel('时间 (s)', 'Color','k', 'FontSize',12);
end

sgtitle(['Logged results from model: ', model], ...
    'Interpreter','none', 'Color','k', 'FontSize',14);