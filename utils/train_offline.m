clc;clear;

% Load agent
S1 = load("data/agent_sac_variable_impedance_pc1.mat","agent","trainingStats");
S2 = load("data/agent_sac_variable_impedance_pc2.mat","agent","trainingStats");

buf1 = S1.agent.ExperienceBuffer;
buf2 = S2.agent.ExperienceBuffer;

bufMerged = mergeReplayMemory(buf1, buf2, 50000);

agent = S2.agent;
trainingStats = S2.trainingStats;

% Put replay buffer into the agent
agent.ExperienceBuffer = bufMerged;

if canUseGPU
    % Move actor to GPU
    actor = getActor(agent);
    actor.UseDevice = "gpu";
    agent = setActor(agent, actor);

    % Move both critics to GPU
    critics = getCritic(agent);
    for k = 1:numel(critics)
        critics(k).UseDevice = "gpu";
        agent = setCritic(agent, critics(k));
    end
end

% Offline-training options
tfdOpts = rlTrainingFromDataOptions( ...
    MaxEpochs = 10, ...
    NumStepsPerEpoch = 800, ...
    Plots = "none", ...
    Verbose = true);

% Train from the replay buffer
tic;
tfdStats = trainFromData(agent, tfdOpts);
toc;

% % Save updated agent
% save("/agent_sac_variable_impedance.mat","agent","trainingStats","-v7.3");