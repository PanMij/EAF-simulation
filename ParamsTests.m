%% GPC parameter sweep for ctrlSys
% This script sweeps N2, Nu, and lam (1x3 vector) and runs ctrlSys.
% Assumes:
%   - Model name: 'ctrlSys'
%   - PID gains are taken from workspace variables: Kp, Ki, Kd
%   - You will fill in how to compute the performance metric from logs/signals.

clear; clc;

%% 1. Define model and parameter ranges
workers = 8;
stopTime = 1;
save_path = fullfile(pwd, "results");

mdl = 'ctrlSys';
A = load("data/GPC_params.mat", "A"); A = A.A;
B = load("data/GPC_params.mat", "B"); B = B.B;

% parameter ranges
N2_vals = 15;
Nu_vals = 1;
lam1_vals = 0.2;
% lam2_vals = [0.08 0.1 0.12 0.15];
lam2_vals = [0.08 0.15];
lam3_vals = 0.5;
[lam1_grid, lam2_grid, lam3_grid] = ndgrid(lam1_vals, lam2_vals, lam3_vals);
lam_vals = [lam1_grid(:), lam2_grid(:), lam3_grid(:)];

%% 2. Preallocate result storage
nN2 = numel(N2_vals);
nNu = numel(Nu_vals);
nlambda = size(lam_vals, 1);

nCases = nN2 * nNu * nlambda;

%% 3. Create SimulationInput array for all combinations
in(nCases,1) = Simulink.SimulationInput(mdl);  % preallocate
idx = 0;

for i = 1:nN2
    for j = 1:nNu
        for k = 1:nlambda
            idx = idx + 1;
            
            % Set model name
            in(idx) = in(idx).setModelName(mdl);

            % Set stop time
            in(idx) = in(idx).setModelParameter('StopTime', num2str(stopTime));
            
            % Set GPC parameters
            N2 = N2_vals(i);
            Nu = Nu_vals(j);
            lam = lam_vals(k, :);

            % Store parameters in the SimulationInput object
            in(idx) = in(idx).setVariable('A', A, 'Workspace', 'GPC');
            in(idx) = in(idx).setVariable('B', B, 'Workspace', 'GPC');
            in(idx) = in(idx).setVariable('N2', N2, 'Workspace', 'GPC');
            in(idx) = in(idx).setVariable('Nu', Nu, 'Workspace', 'GPC');
            in(idx) = in(idx).setVariable('lam', lam, 'Workspace', 'GPC');
            in(idx) = in(idx).setUserString(sprintf("sim_%d", idx));
            in(idx) = setPostSimFcn(in(idx), @(out) save_worker_output(out, in(idx), save_path));
        end
    end
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
function out = save_worker_output(out, in, save_path)
    if ~isempty(out.ErrorMessage)
        out = out.setUserData(out.ErrorMessage);
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
    fig = figure("Visible", "off");

    subplot(2, 2, 1);
    plot(t_u, u);
    legend;
    title(sprintf("Controller output (Nu=%d,N2=%d," + ...
        "lam=[%.2f,%.2f,%.2f])", Nu, N2, lam(1), lam(2), lam(3)));
    
    for i = 1 : 3
        subplot(2, 2, i + 1);
        plot(t_step, step(:, i));
        hold on;
        plot(t_r, r(:, i));
        hold off;
        axis([0 t_step(end) r_ranges(i, :)]);
        legend;
        title(sprintf("Impedance of phase %d", i));
    end

    % Save the figure
    filename = fullfile(save_path, idx + ".png");
    exportgraphics(fig, filename, 'Resolution', 300);
    close(fig);
end