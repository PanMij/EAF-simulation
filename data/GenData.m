clear; clc;

%% Parameters
mdl = 'PlantIdentification';
sample_per_frame = 1000;
stop_time = 200;

%% Generate data
simIn = Simulink.SimulationInput(mdl);
simIn = simIn.setModelParameter('StopTime', num2str(stop_time));
out = sim(simIn, "ShowSimulationManager", "on");

%% Save the data
data = struct('t', out.yout{1}.Values.Time, 'y', out.yout{1}.Values.Data(:, 1), 'u', out.yout{1}.Values.Data(:, 2));
save('data/IdData', "data");