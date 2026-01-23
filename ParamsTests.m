%% GPC parameter sweep for ctrlSys
% This script sweeps N2, Nu, and lam (1x3 vector) and runs ctrlSys.
% Assumes:
%   - Model name: 'ctrlSys'
%   - PID gains are taken from workspace variables: Kp, Ki, Kd
%   - You will fill in how to compute the performance metric from logs/signals.

clear; clc;

%% 1. Define model and parameter ranges
useParallel = true;  % set true if you want to try parsim
workers = 8;          % valid only if using parsim
stopTime = 70;
save_path = fullfile(pwd, "results");

mdl = 'ctrlSys';
A = load("data/GPC_params.mat", "A"); A = A.A;
B = load("data/GPC_params.mat", "B"); B = B.B;

% parameter ranges
N2_vals = 45;
Nu_vals = [2 1];
alpha_vals = 0.5;
lam1_vals = [0.2 0.3];
lam2_vals = 0.1;
lam3_vals = [0.5 0.7];
[N2_grid, Nu_grid, alpha_grid, lam1_grid, lam2_grid, lam3_grid] = ndgrid(N2_vals, Nu_vals, alpha_vals, lam1_vals, lam2_vals, lam3_vals);
params = [N2_grid(:), Nu_grid(:), alpha_grid(:), lam1_grid(:), lam2_grid(:), lam3_grid(:)];

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
    % in(i) = in(i).setModelParameter('SimulationMode', 'accelerator');
    
    % Drop unnecessary data to reduce worker-side memory. 
    in(i) = in(i).setModelParameter('SaveTime', 'off');
    % in(i) = in(i).setModelParameter('SaveOutput', 'off');
    in(i) = in(i).setModelParameter('SaveState', 'off');
    in(i) = in(i).setModelParameter('SignalLogging', 'off');

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

%% 4. Run simulations
fprintf("==============SIMULATION START==============\n");
tic;

if useParallel
    % If Parallel Computing Toolbox is available, use parsim for this batch
    if isempty(gcp('nocreate'))
        parpool(workers);  % Set number of workers if necessary
    end
    simOut = parsim(in, 'ShowSimulationManager', 'off', 'ShowProgress', 'on');
    delete(gcp('nocreate'));
else
    % Run the batch without parallelization
    simOut = sim(in, 'ShowSimulationManager', 'off', 'ShowProgress', 'on');
end

fprintf("Time Elapsed = %.2fs\n", toc);
fprintf("==============SIMULATION END==============\n");

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
    alpha = in.getVariable("alpha");
    idx = in.UserString;

    % Extract data
    logs = out.yout;
    t_u = logs.getElement('u').Values.Time;
    u = squeeze(logs.getElement('u').Values.Data);
    if size(u, 1) < size(u, 2)
        u = u.';
    end
    t_r = logs.getElement('r').Values.Time;
    r = squeeze(logs.getElement('r').Values.Data);
    if size(r, 1) < size(r, 2)
        r = r.';
    end
    t_step = logs.getElement('step').Values.Time;
    step = squeeze(logs.getElement('step').Values.Data);
    if size(step, 1) < size(step, 2)
        step = step.';
    end

    % Plot the results
    r_ranges = [0.02 0.03; 0.006 0.035; 0.065 0.09];
    fig = figure("Color", "none", "Visible", "off");

    subplot(2, 2, 1);
    plot(t_u, u);
    legend("u1", "u2", "u3");
    title(sprintf("Controller output (Nu=%d,N2=%d," + ...
        "lam=[%.2f,%.2f,%.2f]),alpha=%.2f", ...
        Nu, N2, lam(1), lam(2), lam(3), alpha));
    
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
    % out.logsout = [];
    % out.tout = [];

    meta = struct("Nu", Nu, "N2", N2, "lam", lam, "tag", idx);
    jobOut = setUserData(out, meta);
end