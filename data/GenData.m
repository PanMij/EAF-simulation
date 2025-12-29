clear; clc;

%% Parameters
workers = 8;
mdl = 'PlantIdentification';
sample_per_frame = [60:10:90];
stop_time = 400;

%% Generate data
simIn(numel(sample_per_frame), 1) = Simulink.SimulationInput(mdl);
for i = 1:numel(sample_per_frame)
    simIn(i) = simIn(i).setModelName(mdl);
    simIn(i) = simIn(i).setModelParameter('StopTime', num2str(stop_time));
    simIn(i) = simIn(i).setVariable('sample_per_frame', sample_per_frame(i));
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
    save(sprintf("data/Id%d", sample_per_frame(i)), "data", "-v7.3");
end
