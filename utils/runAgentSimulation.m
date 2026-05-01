function [experience, env] = runAgentSimulation(agent, Z_init, l_slag, Z_adj_init, maxsteps)
% runAgentSimulation simulates ctrlSys_rl_env_sg.slx with an RL agent.
%
% Inputs:
%   agent   - Trained RL agent
%   Z_init  - 3x1 impedance signal input to root Inport Z_init
%   l_slag  - 3x1 slag height input to root Inport l_slag
%
% Outputs:
%   experience - RL simulation experience returned by sim(env,agent,...)
%   env        - Simulink RL environment object

    %% Validate inputs
    validateattributes(Z_init, {'numeric'}, {'size', [3 1]}, ...
        mfilename, 'Z_init');

    validateattributes(l_slag, {'numeric'}, {'size', [3 1]}, ...
        mfilename, 'l_slag');

    %% Model and RL Agent block
    mdl = "ctrlSys_rl_env_sg";
    agentBlk = mdl + "/RL_sup/RL Agent";

    load_system(mdl);

    %% Create the Simulink RL environment
    obsInfo = getObservationInfo(agent);
    actInfo = getActionInfo(agent);

    env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);

    %% Set external inputs through the environment reset function
    env.ResetFcn = @(in) localResetFcn(in, mdl, Z_init, l_slag, Z_adj_init);

    %% Simulate using environment and agent arguments
    simOpts = rlSimulationOptions(...
        "MaxSteps", maxsteps);

    experience = sim(env, agent, simOpts);

end


function in = localResetFcn(in, mdl, Z_init, l_slag, Z_adj_init)
% localResetFcn sets the root-level Inport data before each simulation.

    inputDataset = localCreateInputDataset(mdl, Z_init, l_slag);

    in = setExternalInput(in, inputDataset);

    in = setVariable(in, "Z_adj_init", Z_adj_init, "Workspace", mdl);
end


function inputDataset = localCreateInputDataset(mdl, Z_init, l_slag)
% localCreateInputDataset creates external input data for root-level ports.

    %% Get model stop time
    stopTimeStr = get_param(mdl, "StopTime");
    stopTime = str2double(stopTimeStr);

    if isnan(stopTime)
        try
            stopTime = evalin("base", stopTimeStr);
        catch
            modelWorkspace = get_param(mdl, "ModelWorkspace");
            stopTime = modelWorkspace.evalin(stopTimeStr);
        end
    end

    if ~isfinite(stopTime) || stopTime <= 0
        stopTime = 1;
    end

    t = [0; stopTime];

    %% Constant 3x1 input signals over the whole simulation
    Z_data = cat(3, Z_init, Z_init);
    l_data = cat(3, l_slag, l_slag);

    Z_ts = timeseries(Z_data, t);
    l_ts = timeseries(l_data, t);

    Z_ts.Name = "Z_init";
    l_ts.Name = "l_slag";

    %% Dataset order must match root Inport order
    inputDataset = Simulink.SimulationData.Dataset;
    inputDataset = inputDataset.addElement(Z_ts, "Z_init");
    inputDataset = inputDataset.addElement(l_ts, "l_slag");

end