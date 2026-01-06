clear; clc;

%% Parameters
workers = 8;
mdl = 'PlantIdentification';
sample_per_frame = 10;
L_init_ls = linspace(0.1025, 0.4187, 10);
L_init_ls = L_init_ls(2:end-1);

%% Generate data
simIn(numel(L_init_ls), 1) = Simulink.SimulationInput(mdl);
for i = 1:numel(simIn)
    simIn(i) = simIn(i).setModelName(mdl);
    simIn(i) = simIn(i).setVariable('L_init', L_init_ls(i));
end

if isempty(gcp('nocreate'))
    parpool(workers);  % Set number of workers if necessary
end
simOut = parsim(simIn, "ShowSimulationManager", "on");

%% Save the data
% data = struct('t', out.yout{1}.Values.Time, 'y', out.yout{1}.Values.Data(:, 1), 'u', out.yout{1}.Values.Data(:, 2));
% save('data/IdData', "data");
for i = 1:numel(simOut)
    if ~isempty(simOut(i).ErrorMessage)
        continue;
    end
    l_hy = simOut(i).logsout{1}.Values.Data;
    t_hy = simOut(i).logsout{1}.Values.Time;
    t = simOut(i).yout{1}.Values.Time;
    y = simOut(i).yout{1}.Values.Data(:, 1);
    u = simOut(i).yout{1}.Values.Data(:, 2);
    data = struct('l_hy', l_hy, 't_hy', t_hy, 't', t, 'y', y, 'u', u);
    save(sprintf("data/Id%.4f.mat", L_init_ls(i)), "data", "-v7.3");
end
