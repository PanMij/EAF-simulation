clear; clc;

%% Parameters
useParallel = true;
workers = 2;
mdl = 'PlantIdentificationMPC';
str = {"est", "val"};
L_init_ls = 0.1728;
% PRBS params
Tg = 20;
Tp = 10;
prbs_seed = {[5 7 3], [7 3 5]};
nSeq = {8, 4};
% Noise params
Rn_seed = {[4949 4511 6499], [2625, 5109, 7021]};
Rn_SNR = 40;
R_rms = 0.06;
dist_seed = {[67905 75798 74338], [ 39283 65582 17201]};
dist_power = 2e-7;
% Use compensator
withComp = true;

%% Generate data
simIn(2, 1) = Simulink.SimulationInput(mdl);
for i = 1:numel(simIn)
    simIn(i) = simIn(i).setModelName(mdl);
    simIn(i) = simIn(i).setVariable('L_init', L_init_ls);
    simIn(i) = simIn(i).setVariable('Tg', Tg, 'Workspace', 'PlantIdentificationMPC');
    simIn(i) = simIn(i).setVariable('Tp', Tp, 'Workspace', 'PlantIdentificationMPC');
    simIn(i) = simIn(i).setVariable('prbs_seed', prbs_seed{i}, 'Workspace', 'PlantIdentificationMPC');
    simIn(i) = simIn(i).setVariable('nSeq', nSeq{i}, 'Workspace', 'PlantIdentificationMPC');
    simIn(i) = simIn(i).setVariable('Rn_seed', Rn_seed{i});
    simIn(i) = simIn(i).setVariable('Rn_SNR', Rn_SNR);
    simIn(i) = simIn(i).setVariable('R_rms', R_rms);
    simIn(i) = simIn(i).setVariable('dist_seed', dist_seed{i});
    simIn(i) = simIn(i).setVariable('dist_power', dist_power);
    simIn(i) = simIn(i).setVariable('withComp', withComp, 'Workspace', 'PlantIdentificationMPC');

    % simIn(i) = simIn(i).setModelParameter('StopTime', '5');
    simIn(i) = simIn(i).setModelParameter('SimulationMode', 'rapid-accelerator');
    simIn(i) = simIn(i).setModelParameter( ...
        'SaveTime', 'off', ...
        'SignalLogging', 'off' ...
    );

    save_path = fullfile(pwd, "data");
    simIn(i) = setPostSimFcn(simIn(i), @(out) save_worker_output(out, save_path, sprintf("IdMPC_%s.mat", str{i})));
end

if useParallel
    if isempty(gcp('nocreate'))
        parpool(workers);  % Set number of workers if necessary
    end
    simOut = parsim(simIn, "ShowProgress", "on");
    delete(gcp('nocreate'));
else
    simOut = sim(simIn, "ShowProgress", "on");
end

%% Local function
function out = save_worker_output(out, save_path, file_name)
    % Save simulation data on the worker to avoid large simOut on client.
    if ~isempty(out.ErrorMessage)
        return;
    end
    if ~exist(save_path, 'dir')
        mkdir(save_path);
    end
    data = out.yout.getElement('data').Values;
    t = data.Time;
    u = data.Data(:, 1:3);
    l_hy = data.Data(:, 4:6);
    y = data.Data(:, 7:9);
    y_real = data.Data(:, 10:12);
    data_path = fullfile(save_path, file_name);
    save(data_path, "t", "u", "l_hy", "y", "y_real","-v7.3");
    
    fprintf("%s\n", data_path);
    out = out.setUserString(data_path);
end
