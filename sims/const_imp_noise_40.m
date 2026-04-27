%% Run ctrlSys_mpc with MPC and PID, then compare R_arc and step
clear; clc; close all;

model = 'ctrlSys_mpc';
mpcModel = 'MPC_QP';
matFile = 'MPC_workspace_40.mat';
Tstop = 40;
tMinPlot = 0.5;

load_system(model);
set_param(model, 'SignalLogging', 'on');
set_param(model, 'StopTime', num2str(Tstop));

%% Build external input for the root Inport
% Step settings from the uploaded picture:
% Step time    : [20 10 30]
% Initial value: [0.055 0.055 0.055]
% Final value  : [0.045 0.065 0.045]
%
% Therefore:
% 0   <= t < 10 : [0.055 0.055 0.055]
% 10  <= t < 20 : [0.055 0.065 0.055]
% 20  <= t < 30 : [0.045 0.065 0.055]
% 30  <= t      : [0.045 0.065 0.045]

t_ref = [0; 10; 10+eps; 20; 20+eps; 30; 30+eps; Tstop];
u_ref = [0.055 0.055 0.055; % 0
         0.055 0.055 0.055; % 10
         0.055 0.065 0.055; % 10+eps
         0.055 0.065 0.055; % 20
         0.045 0.065 0.055; % 20+eps
         0.045 0.065 0.055; % 30
         0.045 0.065 0.045; % 30+eps
         0.045 0.065 0.045];% Tstop

ref_ts = timeseries(u_ref, t_ref, 'Name', 'ref_in');
ref_ts = setinterpmethod(ref_ts, 'zoh');   % zero-order hold

%% Run case 1: PID
simIn_pid = Simulink.SimulationInput(model);
simIn_pid = simIn_pid.setModelParameter('SimulationMode', 'rapid-accelerator');
simIn_pid = simIn_pid.setVariable('R_rms', 0.06);
simIn_pid = simIn_pid.setVariable('dist_power', 2e-7);
simIn_pid = simIn_pid.setVariable('CtrlChoice', "PID", 'Workspace', model);
simIn_pid = simIn_pid.setVariable('Kp', 10, 'Workspace', model);
simIn_pid = simIn_pid.setVariable('Ki', 10, 'Workspace', model);
simIn_pid = simIn_pid.setVariable('Kd', [0.01225 0.02 0.01225], 'Workspace', model);
simIn_pid = simIn_pid.setExternalInput(ref_ts);

% simOut_pid = sim(simIn_pid);

%% Run case 2: MPC
simIn_mpc = Simulink.SimulationInput(model);
simIn_mpc = simIn_mpc.setModelParameter('SimulationMode', 'rapid-accelerator');
simIn_mpc = simIn_mpc.setVariable('R_rms', 0.06);
simIn_mpc = simIn_mpc.setVariable('dist_power', 2e-7);
simIn_mpc = simIn_mpc.setVariable('CtrlChoice', "MPC_QP", 'Workspace', model);
simIn_mpc = simIn_mpc.setExternalInput(ref_ts);
% Set MPC parameters
mpcVars = load(matFile);
mpcVarNames = fieldnames(mpcVars);
for i = 1:numel(mpcVarNames)
    varName = mpcVarNames{i};
    simIn_mpc = simIn_mpc.setVariable(varName, mpcVars.(varName), 'Workspace', mpcModel);
end

% simOut_mpc = sim(simIn_mpc);

if isempty(gcp('nocreate'))
    parpool(2);
end
simOut = parsim([simIn_mpc, simIn_pid], 'ShowProgress', 'on');
delete(gcp('nocreate'));

simOut_mpc = simOut(1);
simOut_pid = simOut(2);
logs_mpc = simOut_mpc.logsout;
logs_pid = simOut_pid.logsout;

%% Get logged signals
[R_mpc, t_mpc]     = getLoggedData(logs_mpc, {'R_real'});
[step_mpc, t_step] = getLoggedData(logs_mpc, {'step'});
[R_pid, t_pid]     = getLoggedData(logs_pid, {'R_real'});

% Check time consistency
if numel(t_mpc) ~= numel(t_pid) || any(abs(t_mpc - t_pid) > 1e-12)
    error('The time vectors of MPC and PID simulations are inconsistent.');
end

% Make sure signals are [Nt x Ny]
R_mpc    = reshapeSignal(R_mpc, t_mpc);
R_pid    = reshapeSignal(R_pid, t_pid);
step_mpc = reshapeSignal(step_mpc, t_step);

% If step is scalar, replicate it to three phases
if size(step_mpc, 2) == 1 && size(R_mpc, 2) == 3
    step_mpc = repmat(step_mpc, 1, 3);
end

% Basic checks
if size(R_mpc, 2) < 3 || size(R_pid, 2) < 3 || size(step_mpc, 2) < 3
    error('R_arc and step should contain three phase signals.');
end

% Keep only samples with time > tMinPlot
idx_step = t_step > tMinPlot;
idx_pid  = t_pid  > tMinPlot;
idx_mpc  = t_mpc  > tMinPlot;

if ~any(idx_step) || ~any(idx_pid) || ~any(idx_mpc)
    error('No samples remain after applying the time filter t > %.3f s.', tMinPlot);
end

t_step_plot   = t_step(idx_step);
step_mpc_plot = step_mpc(idx_step, :);

t_pid_plot = t_pid(idx_pid);
R_pid_plot = R_pid(idx_pid, :);

t_mpc_plot = t_mpc(idx_mpc);
R_mpc_plot = R_mpc(idx_mpc, :);

%% Plot
phaseName = {'A相', 'B相', 'C相'};

fig = figure( ...
    'Name', ['Logged results from model: ', model], ...
    'Color', 'w', ...
    'NumberTitle', 'off');

tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

for k = 1:3
    ax = nexttile;
    hold(ax, 'on');

    h(1) = plot(t_step_plot, step_mpc_plot(:,k), '--', 'LineWidth', 1.5, 'Color', 'black');
    h(2) = plot(t_pid_plot,  R_pid_plot(:,k),     'LineWidth', 1.5, 'Color', 'blue');
    h(3) = plot(t_mpc_plot,  R_mpc_plot(:,k),     'LineWidth', 1.5, 'Color', 'red');

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
    xlim(ax, [tMinPlot, Tstop]);

    ylabel([phaseName{k}, ' 电弧电阻 (\Omega)'], ...
        'Interpreter', 'tex', 'Color', 'k', 'FontSize', 12);
    xlabel('时间 (s)', 'Color', 'k', 'FontSize', 12);

    title(phaseName{k}, 'Color', 'k', 'FontSize', 13);
end

lgd = legend(hLegend, {'阻抗设定值', 'PID', 'MPC'}, ...
    'Orientation', 'vertical', ...
    'Box', 'off', ...
    'FontSize', 11, ...
    'TextColor', 'k', ...
    'Interpreter', 'none');
lgd.Layout.Tile = 'east';

sgtitle(['Logged results from model: ', model], ...
    'Interpreter', 'none', 'Color', 'k', 'FontSize', 14);

%% Local functions
function [data, time] = getLoggedData(logs, nameCandidates)
    sig = [];
    for i = 1:numel(nameCandidates)
        sig = logs.getElement(nameCandidates{i});
        if ~isempty(sig)
            break;
        end
    end

    if isempty(sig)
        error('Logged signal was not found. Candidates: %s', strjoin(nameCandidates, ', '));
    end

    vals = sig.Values;
    data = vals.Data;
    time = vals.Time;
end

function y = reshapeSignal(data, time)
    if isvector(data)
        y = data(:);
    else
        sz = size(data);
        Nt = numel(time);

        if sz(1) == Nt
            y = reshape(data, Nt, []);
        else
            order = [ndims(data), 1:ndims(data)-1];
            y = reshape(permute(data, order), Nt, []);
        end
    end
end