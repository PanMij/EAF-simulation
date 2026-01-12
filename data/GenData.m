clear; clc;

%% Parameters
workers = 8;
mdl = 'PlantIdentification';
L_init_ls = linspace(0.1025, 0.4187, 10);
L_init_ls = L_init_ls(2:end-1);

%% Generate data
simIn(numel(L_init_ls), 1) = Simulink.SimulationInput(mdl);
for i = 1:numel(simIn)
    simIn(i) = simIn(i).setModelName(mdl);
    simIn(i) = simIn(i).setVariable('L_init', L_init_ls(i));
    save_path = fullfile(pwd, "data");
    simIn(i) = setPostSimFcn(simIn(i), @(out) save_worker_output(out, save_path, sprintf("Id%.4f.mat", L_init_ls(i))));
end

if isempty(gcp('nocreate'))
    parpool(workers);  % Set number of workers if necessary
end
simOut = parsim(simIn, "ShowSimulationManager", "on");

%% Local function
function out = save_worker_output(out, save_path, file_name)
% Save simulation data on the worker to avoid large simOut on client.
if ~isempty(out.ErrorMessage)
    return;
end
if ~exist(save_path, 'dir')
    mkdir(save_path);
end
l_hy = out.logsout.getElement('l_hy').Values.Data;
t_hy = out.logsout.getElement('l_hy').Values.Time;
t = out.yout{1}.Values.Time;
y = out.yout{1}.Values.Data(:, 1:3);
u = out.yout{1}.Values.Data(:, 4:6);
data = struct('l_hy', l_hy, 't_hy', t_hy, 't', t, 'y', y, 'u', u);
data_path = fullfile(save_path, file_name);
save(data_path, 'data', '-v7.3');

% Trim large fields before returning to client to save memory.
out.logsout = [];
out.tout = [];
out.yout = [];
fprintf("%s\n", data_path);
out = out.setUserString(data_path);
end
