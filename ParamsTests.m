%% GPC parameter sweep for ctrlSys
% This script sweeps N2, Nu, and lam (1x3 vector) and runs ctrlSys.
% Assumes:
%   - Model name: 'ctrlSys'
%   - PID gains are taken from workspace variables: Kp, Ki, Kd
%   - You will fill in how to compute the performance metric from logs/signals.

clear; clc;

%% 1. Define model and parameter ranges
workers = 8;
stopTime = 70;
save_path = fullfile(pwd, "results");

mdl = 'ctrlSys';
A = load("data/GPC_params.mat", "A"); A = A.A;
B = load("data/GPC_params.mat", "B"); B = B.B;

% parameter ranges
N2_vals = 15:3:30;
Nu_vals = 2;
alpha_vals = [0.5 0.6 0.7];
lam1_vals = 0.2;
lam2_vals = 0.1;
lam3_vals = 0.5;
[N2_grid, Nu_grid, alpha_grid, lam1_grid, lam2_grid, lam3_grid] = ndgrid(N2_vals, Nu_vals, alpha_vals, lam1_vals, lam2_vals, lam3_vals);
params = [N2_grid(:), Nu_grid(:), alpha_grid(:), lam1_grid(:), lam2_grid(:), lam3_grid(:)];

%% 2. Preallocate result storage
nCases = size(params, 1);

%% 3. Create SimulationInput array for all combinations
in(nCases,1) = Simulink.SimulationInput(mdl);  % preallocate

for i = 1:nCases
    % Set model name
    in(i) = in(i).setModelName(mdl);

    % Set stop time
    in(i) = in(i).setModelParameter('StopTime', num2str(stopTime));
    
    % Set GPC parameters
    N2 = params(i, 1);
    Nu = params(i, 2);
    alpha = params(i, 3);
    lam = params(i, 4:6);

    % Store parameters in the SimulationInput object
    in(i) = in(i).setVariable('A', A, 'Workspace', 'GPC');
    in(i) = in(i).setVariable('B', B, 'Workspace', 'GPC');
    in(i) = in(i).setVariable('N2', N2, 'Workspace', 'GPC');
    in(i) = in(i).setVariable('Nu', Nu, 'Workspace', 'GPC');
    in(i) = in(i).setVariable('alpha', alpha, 'Workspace', 'GPC');
    in(i) = in(i).setVariable('lam', lam, 'Workspace', 'GPC');
    in(i) = in(i).setUserString(sprintf("sim_%d", i));

    curIdx = i;
    in(i) = setPostSimFcn(in(i), @(out) save_worker_output(out, in(curIdx), save_path));
end

%% 4. Run simulations in batches (use parsim if you have Parallel Computing Toolbox)
useParallel = true;  % set true if you want to try parsim

fprintf("==============SIMULATION START==============\n");
tic;

if useParallel
    % If Parallel Computing Toolbox is available, use parsim for this batch
    if isempty(gcp('nocreate'))
        parpool(workers);  % Set number of workers if necessary
    end
    simOut = parsim(in, 'ShowSimulationManager', 'on');
else
    % Run the batch without parallelization
    simOut = sim(in, 'ShowSimulationManager', 'on');
end

fprintf("Time Elapsed = %.2fs\n", toc);
fprintf("==============SIMULATION END==============\n");

delete(gcp('nocreate'));

%% Local function
function jobOut = save_worker_output(out, in, save_path)
    if ~isempty(out.ErrorMessage)
        jobOut = setUserData(out, struct("tag", in.UserString, "error", out.ErrorMessage));
        return
    end
    
    % Extract parameters
    Nu = in.getVariable("Nu");
    N2 = in.getVariable("N2");
    lam = in.getVariable("lam");
    idx = in.UserString;

    % Extract data
    logs = out.logsout;
    t_u = logs{1}.Values.Time;
    u = squeeze(logs{1}.Values.Data).';
    t_r = logs{2}.Values.Time;
    r = logs{2}.Values.Data;
    t_step = logs{3}.Values.Time;
    step = logs{3}.Values.Data;

    % Plot the results
    r_ranges = [0.02 0.03; 0.006 0.035; 0.065 0.09];
    fig = figure("Color", "none", "Visible", "off");

    subplot(2, 2, 1);
    plot(t_u, u);
    legend("u1", "u2", "u3");
    title(sprintf("Controller output (Nu=%d,N2=%d," + ...
        "lam=[%.2f,%.2f,%.2f])", Nu, N2, lam(1), lam(2), lam(3)));
    
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

    % Remove the simulation data to reduce memory usage
    out.logsout = [];
    out.tout = [];
    meta = struct("Nu", Nu, "N2", N2, "lam", lam, "tag", idx);
    jobOut = setUserData(out, meta);
end