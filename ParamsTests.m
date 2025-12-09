%% PID parameter sweep for ctrlSys
% This script sweeps Kp, Ki, Kd and runs ctrlSys for each combination.
% Assumes:
%   - Model name: 'ctrlSys'
%   - PID gains are taken from workspace variables: Kp, Ki, Kd
%   - You will fill in how to compute the performance metric from logs/signals.

clear; clc;

%% 1. Define model and parameter ranges
mdl = 'ctrlSys';

% EDIT THESE RANGES to fit your system
Kp_vals = 50:10:120;
Ki_vals = 20:10:120;
Kd_vals = 0:5:30;

stopTime = 20;  % EDIT: simulation stop time (seconds)


%% 2. Preallocate result storage
nKp = numel(Kp_vals);
nKi = numel(Ki_vals);
nKd = numel(Kd_vals);

nCases = nKp * nKi * nKd;

results = struct( ...
    'Kp',       cell(nCases,1), ...
    'Ki',       cell(nCases,1), ...
    'Kd',       cell(nCases,1), ...
    'SimOK',    cell(nCases,1), ...
    'Metric',   cell(nCases,1) ...
);  % You can rename/add metrics

%% 3. Create SimulationInput array for all combinations
in(nCases,1) = Simulink.SimulationInput(mdl);  % preallocate
idx = 0;

for iP = 1:nKp
    for iI = 1:nKi
        for iD = 1:nKd
            idx = idx + 1;
            
            % Set model name
            in(idx) = in(idx).setModelName(mdl);

            % Set stop time
            in(idx) = in(idx).setModelParameter('StopTime', num2str(stopTime));

            Kp = Kp_vals(iP);
            Ki = Ki_vals(iI);
            Kd = Kd_vals(iD);

            % Store parameters in the SimulationInput object
            in(idx) = in(idx).setVariable('Kp', Kp);
            in(idx) = in(idx).setVariable('Ki', Ki);
            in(idx) = in(idx).setVariable('Kd', Kd);

            % Save for later
            results(idx).Kp = Kp;
            results(idx).Ki = Ki;
            results(idx).Kd = Kd;
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
            parpool(16);  % Set number of workers if necessary
        end
        simOutBatch = parsim(batchIn, 'ShowProgress', 'on');
    else
        % Run the batch without parallelization
        simOutBatch = sim(batchIn, 'ShowProgress', 'on');
    end
    
    %% 6. Extract metrics from simulation outputs for this batch
    for k = 1:numel(simOutBatch)
        try
            % Placeholder: mark simulation as OK, but no metric computed
            idx = startIdx + k - 1;
            results(idx).SimOK = isempty(simOutBatch(k).ErrorMessage);
            results(idx).Metric = NaN;   % Replace with your real metric

            logs = simOutBatch(k).logsout;
            t = logs{1}.Values.Time;
            R_meas = logs{1}.Values.Data;
            step = logs{2}.Values.Data;

            fig = figure('Visible','off');
            plot(t, [R_meas step(round(linspace(1, numel(step), numel(t))))]);
            title(sprintf('Kp=%.2f Ki=%.2f Kd=%.2f', results(idx).Kp, results(idx).Ki, results(idx).Kd));
            filename = sprintf('./results/pid_sweep_OK%d_Kp%.2f_Ki%.2f_Kd%.2f.png', results(idx).SimOK, results(idx).Kp, results(idx).Ki, results(idx).Kd);
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
