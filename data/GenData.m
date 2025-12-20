clear; clc;

%% Parameters
mdl = 'PlantIdentification';
sample_per_frame = [100 300 500 800 1000];
stop_time = 400;

%% Generate data
simIn(numel(sample_per_frame), 1) = Simulink.SimulationInput(mdl);
for i = 1:numel(sample_per_frame)
    simIn(i) = simIn(i).setModelName(mdl);
    simIn(i) = simIn(i).setModelParameter('StopTime', num2str(stop_time));
    simIn(i) = simIn(i).setVariable('sample_per_frame', sample_per_frame(i));
end

if isempty(gcp('nocreate'))
    parpool(16);  % Set number of workers if necessary
end
simOut = parsim(simIn, 'ShowProgress', 'on', "ShowSimulationManager", "on");
% out = sim(simIn, "ShowSimulationManager", "on");

%% Save the data
% data = struct('t', out.yout{1}.Values.Time, 'y', out.yout{1}.Values.Data(:, 1), 'u', out.yout{1}.Values.Data(:, 2));
% save('data/IdData', "data");