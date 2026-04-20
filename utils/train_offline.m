clc;clear;

% Load agent
S1 = load("data/agent_sac_variable_impedance_pc1.mat","agent");
S2 = load("data/agent_sac_variable_impedance_pc2.mat","agent");

buf1 = S1.agent.ExperienceBuffer;
buf2 = S2.agent.ExperienceBuffer;

bufMerged = mergeReplayMemory(buf1, buf2, 10000);

agent = S1.agent;

% Put replay buffer into the agent
agent.ExperienceBuffer = bufMerged;

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

% Offline-training options
tfdOpts = rlTrainingFromDataOptions( ...
    MaxEpochs = 20, ...
    NumStepsPerEpoch = 1000, ...
    Plots = "none", ...
    Verbose = true);

% Train from the replay buffer
tfdStats = trainFromData(agent, tfdOpts);

% % Save updated agent
% save("/agent_sac_variable_impedance.mat","agent","tfdStats","-v7.3");