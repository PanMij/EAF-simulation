%% PID parameter sweep for ctrlSys
% This script sweeps Kp, Ki, Kd and runs ctrlSys for each combination.
% Assumes:
%   - Model name: 'ctrlSys'
%   - PID gains are taken from workspace variables: Kp, Ki, Kd
%   - You will fill in how to compute the performance metric from logs/signals.

clear; clc;

%% 1. Define model and parameter ranges
mdl = 'ctrlSys_single_phase';

% EDIT THESE RANGES to fit your system
N2_vals = 5:5:30;
Nu_vals = 1:1:3;
lambda_vals = linspace(1e-6, 1e-3, 30);

stopTime = 20;  % EDIT: simulation stop time (seconds)


%% 2. Preallocate result storage
nN2 = numel(N2_vals);
nNu = numel(Nu_vals);
nlambda = numel(lambda_vals);

nCases = nN2 * nNu * nlambda;

results = struct( ...
    'N2',       cell(nCases,1), ...
    'Nu',       cell(nCases,1), ...
    'lambda',   cell(nCases,1), ...
    'SimOK',    cell(nCases,1), ...
    'Metric',   cell(nCases,1) ...
);  % You can rename/add metrics

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

            N2 = N2_vals(i);
            Nu = Nu_vals(j);
            lambda = lambda_vals(k);

            % Store parameters in the SimulationInput object
            in(idx) = in(idx).setVariable('N2', N2);
            in(idx) = in(idx).setVariable('Nu', Nu);
            in(idx) = in(idx).setVariable('lam', lambda);

            % Save for later
            results(idx).N2 = N2;
            results(idx).Nu = Nu;
            results(idx).lambda = lambda;
        end
    end
end

%% 4. Batch size and number of batches
batchSize = 64;  % Adjust this depending on your available memory
nBatches = ceil(nCases / batchSize);  % Number of batches

%% 5. Run simulations in batches (use parsim if you have Parallel Computing Toolbox)
useParallel = true;  % set true if you want to try parsim

for batchIdx = 1:nBatches
    fprintf("==============BATCH START==============\n");
    tic;

    % Get the start and end indices for the current batch
    startIdx = (batchIdx - 1) * batchSize + 1;
    endIdx = min(batchIdx * batchSize, nCases);
    
    % Create the batch of simulation inputs
    batchIn = in(startIdx:endIdx);
    
    if useParallel
        % If Parallel Computing Toolbox is available, use parsim for this batch
        if isempty(gcp('nocreate'))
            parpool(8);  % Set number of workers if necessary
        end
        simOutBatch = parsim(batchIn, 'ShowSimulationManager', 'on');
    else
        % Run the batch without parallelization
        simOutBatch = sim(batchIn, 'ShowSimulationManager', 'on');
    end
    
    %% 6. Extract metrics from simulation outputs for this batch
    for k = 1:numel(simOutBatch)
        try
            % Placeholder: mark simulation as OK, but no metric computed
            idx = startIdx + k - 1;
            results(idx).SimOK = isempty(simOutBatch(k).ErrorMessage);
            results(idx).Metric = NaN;   % Replace with your real metric

            logs = simOutBatch(k).logsout;
            t_u = logs{1}.Values.Time;
            u = logs{1}.Values.Data;
            step = logs{3}.Values.Data;
            t_r = logs{2}.Values.Time;
            r = logs{2}.Values.Data;

            fig = figure('Visible','off');
            subplot(2, 1, 1);
            plot(t_u, u);
            subplot(2, 1, 2);
            plot(t_u, step);
            hold on
            plot(t_r, r);
            hold off
            axis([0 t_u(end) 0.001 0.0025]); % limit the axis ranges
            title(sprintf('N2=%d Nu=%d lam=%.2e', results(idx).N2, results(idx).Nu, results(idx).lambda));
            filename = sprintf('./results/gpc_sweep_OK%d_N2%d_Nu%d_lam%.2e.png', results(idx).SimOK, results(idx).N2, results(idx).Nu, results(idx).lambda);
            exportgraphics(fig, filename, 'Resolution', 300);
            close(fig);

        catch ME
            % In case of simulation failure or missing data
            results(idx).SimOK = false;
            results(idx).Metric = NaN;
            fprintf('Case %d failed: %s\n', idx, ME.message);
        end
    end
    fprintf("BatchIdx = %d, Time Elapsed = %.2fs\n", batchIdx, toc);
    fprintf("==============BATCH END==============\n");
end
