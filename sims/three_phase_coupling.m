%% Run model and plot selected logged signals with white background
clear; clc; close all;

model = 'EAF_test';
Tstep = 1;
Tstop = 3;
nDrop = 100;

load_system(model);
set_param(model, 'SignalLogging', 'on');
set_param(model, 'StopTime', num2str(Tstop));

% Input l_hy: 3x1 step signal
% Initial value: [0.2; 0.2; 0.2]
% Final value:   [0.3; 0.2; 0.2]
% Step time: 5 s
t_c = [0; Tstep];
u_c = [0.2 0.2 0.2; 0.2 0.2 0.2];
t_r = linspace(Tstep+eps, Tstop, 100).';
u_r = (0.25 - 0.2) / (Tstop - Tstep) * (t_r - Tstep) + 0.2;
u_r = [u_r, repmat([0.2 0.2], [numel(u_r) 1])];
t = [t_c; t_r];
u = [u_c; u_r];
l_hy = timeseries(u, t);

% Run simulation
simOut = sim(model, 'ReturnWorkspaceOutputs', 'on');
logs = simOut.logsout;

if isempty(logs) || logs.numElements == 0
    error('No logged signals were found in logsout.');
end

siglist = {'l_hy', 'i_arc_rms', 'r_arc_rms', 'P_arc_rms'};
signame = ["液压位移 (m)" "电弧电流有效值 (A)" "电弧电阻有效值 (\Omega)" "电弧功率有效值 (W)"];
legtxt = {'A相', 'B相', 'C相'};

fig = figure( ...
    'Name', ['Logged results from model: ', model], ...
    'Color', 'w', ...
    'NumberTitle', 'off');

tl = tiledlayout(numel(siglist), 1, 'TileSpacing', 'compact', 'Padding', 'compact');

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

    % Drop the first nDrop samples
    if numel(time) <= nDrop
        error('Signal "%s" has only %d samples, which is not enough after dropping the first %d samples.', ...
            siglist{k}, numel(time), nDrop);
    end
    time = time(nDrop+1:end);
    y = y(nDrop+1:end, :);

    ax = nexttile;
    h = plot(time, y, 'LineWidth', 1.5);

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

    ylabel(signame(k), 'Interpreter', 'tex', 'Color', 'k', 'FontSize', 12);
    xlabel('时间 (s)', 'Color', 'k', 'FontSize', 12);
end

lgd = legend(hLegend, legtxt, ...
    'Orientation', 'vertical', ...
    'Box', 'off', ...
    'FontSize', 11, ...
    'TextColor', 'k', ...
    'Interpreter', 'none');
lgd.Layout.Tile = 'east';

sgtitle(['Logged results from model: ', model], ...
    'Interpreter', 'none', 'Color', 'k', 'FontSize', 14);

% Coupling analysis
i_arc_rms = logs.getElement('i_arc_rms').Values.Data;
i_arc_rms = i_arc_rms(nDrop+1:end, :);
r_arc_rms = logs.getElement('r_arc_rms').Values.Data;
r_arc_rms = r_arc_rms(nDrop+1:end, :);
P_arc_rms = logs.getElement('P_arc_rms').Values.Data;
P_arc_rms = P_arc_rms(nDrop+1:end, :);

i_arc_rms_start = i_arc_rms(1, :);
i_arc_rms_end = i_arc_rms(end, :);
r_arc_rms_start = r_arc_rms(1, :);
r_arc_rms_end = r_arc_rms(end, :);
P_arc_rms_start = P_arc_rms(1, :);
P_arc_rms_end = P_arc_rms(end, :);

fprintf("Coupling analysis results:\n");
i_change_A = (i_arc_rms_end(1) - i_arc_rms_start(1));
r_change_A = (r_arc_rms_end(1) - r_arc_rms_start(1));
P_change_A = (P_arc_rms_end(1) - P_arc_rms_start(1));
fprintf("  Phase A: i_arc_rms change = %.4f A, r_arc_rms change = %.4f Ω, P_arc_rms change = %.4f W\n", i_change_A, r_change_A, P_change_A);
for phase = 2:size(i_arc_rms, 2)
    fprintf("Phase %c:\n", 'A' + phase - 1);
    fprintf("  i_arc_rms: start = %.4f A, end = %.4f A, change = %.4f %%\n", i_arc_rms_start(phase), i_arc_rms_end(phase), (i_arc_rms_end(phase) - i_arc_rms_start(phase)) / i_change_A * 100);
    fprintf("  r_arc_rms: start = %.4f Ω, end = %.4f Ω, change = %.4f %%\n", r_arc_rms_start(phase), r_arc_rms_end(phase), (r_arc_rms_end(phase) - r_arc_rms_start(phase)) / r_change_A * 100);
    fprintf("  P_arc_rms: start = %.4f W, end = %.4f W, change = %.4f %%\n", P_arc_rms_start(phase), P_arc_rms_end(phase), (P_arc_rms_end(phase) - P_arc_rms_start(phase)) / P_change_A * 100);
end