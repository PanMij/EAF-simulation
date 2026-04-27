%% Run model and plot selected logged signals with white background
clear; clc; close all;

model = 'hydraulic_test';

load_system(model);
set_param(model, 'SignalLogging', 'on');

simIn = Simulink.SimulationInput(model);

% Override variables for this run only
simIn = simIn.setVariable('L_init', 1);
simIn = simIn.setVariable('L_ub', Inf);
simIn = simIn.setVariable('L_lb', 0);

simOut = sim(simIn);
logs = simOut.logsout;

if isempty(logs) || logs.numElements == 0
    error('No logged signals were found in logsout.');
end

siglist = {'vo', 'l_hy'};
signame = ["比例阀开度" "电极位移 (m)"];

fig = figure( ...
    'Name', ['Logged results from model: ', model], ...
    'Color', 'w', ...
    'NumberTitle', 'off');

tiledlayout(numel(siglist), 1, 'TileSpacing', 'compact', 'Padding', 'compact');

for k = 1:numel(siglist)
    sig = logs.getElement(siglist{k});
    if isempty(sig)
        error('Logged signal "%s" was not found in logsout.', siglist{k});
    end

    vals = sig.Values;

    ax = nexttile;
    plot(vals.Time, squeeze(vals.Data), 'LineWidth', 1.5);

    grid(ax, 'on');
    ax.Color = 'w';
    ax.XColor = 'k';
    ax.YColor = 'k';
    ax.GridColor = [0.75 0.75 0.75];
    ax.GridAlpha = 0.35;
    ax.FontSize = 12;
    ax.LineWidth = 1;
    
    if isequal(siglist{k}, 'vo')
        ylim([-5e-3 7e-3]);
    end
    ylabel(signame(k), 'Interpreter', 'none', 'Color', 'k', 'FontSize', 12);
    xlabel('时间 (s)', 'Color', 'k', 'FontSize', 12);
end

sgtitle(['Logged results from model: ', model], ...
    'Interpreter', 'none', 'Color', 'k', 'FontSize', 14);