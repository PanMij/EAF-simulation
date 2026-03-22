%% MPC-QP parameter sweep for ctrlSys
% This script sweeps N2, Nu, and lam (1x3 vector) and runs ctrlSys.
% Assumes:
%   - Model name: 'ctrlSys'
%   - MPC-QP variables N2, Nu, lam are resolved in workspace 'MPCQP'

clear; clc;

%% 1. Define model and parameter ranges
useParallel = true;  % set true if you want to try parsim
workers = 8;         % valid only if using parsim
stopTime = 70;
save_path = fullfile(pwd, "results", "to_file_data_mpc");
if ~isfolder(save_path)
    mkdir(save_path)
end

mdl = 'ctrlSys_mpc';
controller_workspace = 'MPCQP';

% parameter ranges
N2_vals = [60 80 100];
Nu_vals = [1 10 20];
lam1_vals = 1;
lam2_vals = 1;
lam3_vals = 1;
wy1_vals = [0.1 0.5 1 5 10];
wy2_vals = [0.1 0.5 1 5 10];
wy3_vals = [0.1 0.5 1 5 10];
[N2_grid, Nu_grid, lam1_grid, lam2_grid, lam3_grid, wy1_grid, wy2_grid, wy3_grid] = ndgrid( ...
    N2_vals, Nu_vals, lam1_vals, lam2_vals, lam3_vals, wy1_vals, wy2_vals, wy3_vals);
params = [N2_grid(:), Nu_grid(:), lam1_grid(:), lam2_grid(:), lam3_grid(:), wy1_grid(:), wy2_grid(:), wy3_grid(:)];
params = params(params(:, 6) == params(:, 7) & params(:, 7) == params(:, 8), :);

%% 2. Preallocate result storage
nCases = size(params, 1);

%% 3. Create SimulationInput array for all combinations
in(nCases,1) = Simulink.SimulationInput(mdl);  % preallocate

for i = 1:nCases
    % Set model name
    in(i) = in(i).setModelName(mdl);

    % Set model parameters
    in(i) = in(i).setModelParameter('StopTime', num2str(stopTime));
    in(i) = in(i).setModelParameter('SimulationMode', 'rapid-accelerator');

    % Drop unnecessary data to reduce worker-side memory.
    in(i) = in(i).setModelParameter('SaveTime', 'off');
    in(i) = in(i).setModelParameter('SaveOutput', 'off');
    in(i) = in(i).setModelParameter('SaveState', 'off');
    in(i) = in(i).setModelParameter('SignalLogging', 'off');

    % Set MPC-QP parameters
    N2 = params(i, 1);
    Nu = params(i, 2);
    lam = params(i, 3:5);
    wy = params(i, 6:8);

    % Store parameters in the SimulationInput object
    in(i) = in(i).setVariable('CtrlChoice', "MPC_QP");
    in(i) = in(i).setVariable('N2', N2, 'Workspace', controller_workspace);
    in(i) = in(i).setVariable('Nu', Nu, 'Workspace', controller_workspace);
    in(i) = in(i).setVariable('lam', lam, 'Workspace', controller_workspace);
    in(i) = in(i).setVariable('wy', wy, 'Workspace', controller_workspace);
    in(i) = in(i).setUserString(sprintf("sim_%d", i));

    % Set output locations
    in(i) = in(i).setBlockParameter( ...
        mdl + "/To File step", "Filename", fullfile(save_path, "step.mat"), ...
        mdl + "/To File u", "Filename", fullfile(save_path, "u.mat"), ...
        mdl + "/To File r", "Filename", fullfile(save_path, "r.mat")...
    );
end

%% 4. Run simulations
fprintf("==============SIMULATION START==============\n");
tic;

if useParallel
    if isempty(gcp('nocreate'))
        parpool(workers);
    end
    simOut = parsim(in, 'ShowSimulationManager', 'off', 'ShowProgress', 'on');
    delete(gcp('nocreate'));
else
    simOut = sim(in, 'ShowSimulationManager', 'off', 'ShowProgress', 'on');
end

fprintf("Time Elapsed = %.2fs\n", toc);
fprintf("==============SIMULATION END==============\n");

%% 5. Save the results
fprintf("==============SAVING RESULTS==============\n");
for i = 1 : nCases
    save_worker_output(in, save_path, i);
end
fprintf("==============SAVING RESULTS END==============\n");

%% Local function
function save_worker_output(in, save_path, idx)
    step_path = fullfile(save_path, sprintf("step_%d.mat", idx));
    r_path = fullfile(save_path, sprintf("r_%d.mat", idx));
    u_path = fullfile(save_path, sprintf("u_%d.mat", idx));

    % Extract parameters
    Nu = in(idx).getVariable("Nu");
    N2 = in(idx).getVariable("N2");
    lam = in(idx).getVariable("lam");
    wy = in(idx).getVariable("wy");

    % Extract data
    [t_u, u] = load_log(u_path, 'u');
    [t_r, r] = load_log(r_path, 'r');
    [t_step, step] = load_log(step_path, 'step');

    % Plot the results
    r_ranges = [0.039 0.069; 0.045 0.078; 0.081 0.114];
    fig = figure("Color", "none", "Visible", "off");

    subplot(2, 2, 1);
    plot(t_u, u);
    legend("u1", "u2", "u3");
    % title(sprintf("Controller output (Nu=%d,N2=%d,lam=[%.2f,%.2f,%.2f])", ...
    %     Nu, N2, lam(1), lam(2), lam(3)));
    title(sprintf("Controller output (Nu=%d,N2=%d,wy=[%.2f,%.2f,%.2f])", ...
        Nu, N2, wy(1), wy(2), wy(3)));

    for i = 1 : 3
        subplot(2, 2, i + 1);
        plot(t_step, step(:, i));
        hold on;
        plot(t_r, r(:, i));
        hold off;
        axis([0 t_step(end) r_ranges(i, :)]);
        legend("step", "r")
        title(sprintf("Impedance of phase %d", i));
    end

    % Save the figure
    filename = fullfile(save_path, idx + ".png");
    exportgraphics(fig, filename, 'Resolution', 300);
    close(fig);
end

function [time, data] = load_log(path, var)
    tmp = load(path, var);
    time = tmp.(var).Time;
    data = squeeze(tmp.(var).Data);
    if ismatrix(data) && size(data, 1) < size(data, 2)
        data = data.';
    end
end
