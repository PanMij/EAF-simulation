clear; clc;

%% Parameters
useParallel = false;
workers = 8;
mdl = 'PlantIdentificationMPC';
% L_init_ls = linspace(0.1025, 0.4187, 10);
% L_init_ls = L_init_ls(2:end-1);
L_init_ls = 0.1728;

%% Generate data
load("data/voltage_speed_lut_0.0001.mat");
simIn(numel(L_init_ls), 1) = Simulink.SimulationInput(mdl);
for i = 1:numel(simIn)
    simIn(i) = simIn(i).setModelName(mdl);
    simIn(i) = simIn(i).setVariable('L_init', L_init_ls(i));

    % simIn(i) = simIn(i).setModelParameter('StopTime', '5');
    simIn(i) = simIn(i).setModelParameter('SimulationMode', 'rapid-accelerator');
    simIn(i) = simIn(i).setModelParameter( ...
        'SaveTime', 'off', ...
        'SignalLogging', 'off' ...
    );

    save_path = fullfile(pwd, "data");
    simIn(i) = setPostSimFcn(simIn(i), @(out) save_worker_output(out, save_path, sprintf("IdMPC%.4f.mat", L_init_ls(i))));
end

if useParallel
    if isempty(gcp('nocreate'))
        parpool(workers);  % Set number of workers if necessary
    end
    simOut = parsim(simIn, "ShowProgress", "on");
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
y = data.Data(:, 4:6);
l_hy = data.Data(:, 7:9);
data_path = fullfile(save_path, file_name);
save(data_path, "t", "y", "u", "l_hy", "-v7.3");

fprintf("%s\n", data_path);
out = out.setUserString(data_path);
end
