%% User settings
mdl = "ctrlSys_rl_env";
agentBlk = "RL_RefAdjSup/RL Agent";
TsRL = 5.0; % RL supervisory sample time [s]

%% RL initialization
% Observation specification
obsInfo = rlNumericSpec([15 1], ...
    LowerLimit = -inf(15,1), ...
    UpperLimit =  inf(15,1));
obsInfo.Name = "observations";
obsInfo.Description = "FSI_n, SampEn_n, Z_n, Zadj_n_prev, dZ_n_prev";

% Action specification
actInfo = rlNumericSpec([3 1], ...
    LowerLimit = -ones(3,1), ...
    UpperLimit =  ones(3,1));
actInfo.Name = "normalized_dZ_action";

% Create environment
env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);

% Optional: randomized reset function
env.ResetFcn = @(in)localResetFcn(in);

% Create default SAC agent
agent = rlSACAgent(obsInfo, actInfo);
% SAC options
agentOpts = rlSACAgentOptions;
agentOpts.SampleTime = TsRL;
agentOpts.DiscountFactor = 0.99;
agentOpts.MiniBatchSize = 128;
agentOpts.ExperienceBufferLength = 1e6;

% Slower, safer initial learning rates
agentOpts.ActorOptimizerOptions.LearnRate = 1e-4;
agentOpts.CriticOptimizerOptions(1).LearnRate = 1e-3;
agentOpts.CriticOptimizerOptions(2).LearnRate = 1e-3;

agent.AgentOptions = agentOpts;

%% Train
% Training options
% trainOpts = rlTrainingOptions( ...
%     MaxEpisodes = 300, ...
%     MaxStepsPerEpisode = 200, ...
%     ScoreAveragingWindowLength = 20, ...
%     Verbose = true, ...
%     Plots = "training-progress", ...
%     StopTrainingCriteria = "AverageReward", ...
%     StopTrainingValue = 0);
trainOpts = rlTrainingOptions( ...
    MaxEpisodes = 2, ...
    MaxStepsPerEpisode = 10, ...
    ScoreAveragingWindowLength = 20, ...
    Verbose = true, ...
    Plots = "training-progress", ...
    StopTrainingCriteria = "AverageReward", ...
    StopTrainingValue = 0);

trainingStats = train(agent, env, trainOpts);

% Save
save("data/agent_sac_variable_impedance.mat", "agent", "trainingStats");

%% Local helpers
function in = localResetFcn(in)
% Keep this minimal first. Add randomization later.

% Example placeholders:
% in = setVariable(in, "Zadj0", zeros(3,1));
% in = setVariable(in, "noise_level", 0.0);

end