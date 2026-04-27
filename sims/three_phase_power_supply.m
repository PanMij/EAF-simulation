%% Run model and plot selected logged signals with white background
clear; clc; close all;

model = 'EAF_test';
Tstop = 0.2;

load_system(model);
set_param(model, 'SignalLogging', 'on');
set_param(model, 'StopTime', num2str(Tstop));

% Input l_hy: 3x1 constant signal of 0.2
t = [0; Tstop];
u = repmat([0.2 0.2 0.2], numel(t), 1);
l_hy = timeseries(u, t);

% Run simulation
simOut = sim(model, 'ReturnWorkspaceOutputs', 'on');
logs = simOut.logsout;

if isempty(logs) || logs.numElements == 0
    error('No logged signals were found in logsout.');
end

siglist = {'v_arc', 'i_arc', 'r_arc'};
signame = ["电弧电压 (V)" "电弧电流 (A)" "电弧电阻 (Ω)"];
legtxt = {'A相', 'B相', 'C相'};

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
    time = vals.Time;
    data = vals.Data;

    % Convert data to [Nt x Ny] for plotting
    if isvector(data)
        y = data(:);
    else
        sz = size(data);
        Nt = numel(time);
        if sz(1) == Nt
            y = reshape(data, Nt, []);
        else
            y = reshape(permute(data, [ndims(data), 1:ndims(data)-1]), Nt, []);
        end
    end

    % Drop the first 100 samples
    nDrop = 100;
    if numel(time) <= nDrop
        error('Signal "%s" has only %d samples, which is not enough after dropping the first %d samples.', ...
            siglist{k}, numel(time), nDrop);
    end
    time = time(nDrop+1:end);
    y = y(nDrop+1:end, :);

    ax = nexttile;
    plot(time, y, 'LineWidth', 1.5);

    grid(ax, 'on');
    ax.Color = 'w';
    ax.XColor = 'k';
    ax.YColor = 'k';
    ax.GridColor = [0.75 0.75 0.75];
    ax.GridAlpha = 0.35;
    ax.FontSize = 12;
    ax.LineWidth = 1;

    ylabel(signame(k), 'Interpreter', 'none', 'Color', 'k', 'FontSize', 12);
    xlabel('时间 (s)', 'Color', 'k', 'FontSize', 12);

    nLine = size(y, 2);

    lgd = legend(ax, legtxt(1:min(nLine, numel(legtxt))), ...
        'Location', 'northeastoutside', ...
        'Box', 'off', ...
        'FontSize', 11, ...
        'TextColor', 'k', ...
        'Interpreter', 'none');

    lgd.Color = 'w';
    lgd.EdgeColor = 'k';
end

sgtitle(['Logged results from model: ', model], ...
    'Interpreter', 'none', 'Color', 'k', 'FontSize', 14);