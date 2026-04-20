function train_vi_one_episode(max_episode)
    for k = 1:max_episode
        rl_vi_one_episode_resume();
    end
end

%% rl_vi_one_episode_resume
function rl_vi_one_episode_resume()
    mdl = "ctrlSys_rl_env_test";
    agentBlk = "ctrlSys_rl_env_test/Subsystem/RL Agent";
    TsRL = 5.0;
    saveFile = "data/agent_sac_variable_impedance.mat";
    workers = 2;

    cleanupObj = onCleanup(@() localCleanup(mdl));

    %% Observation specification
    obsInfo = rlNumericSpec([15 1], ...
        LowerLimit = -inf(15,1), ...
        UpperLimit =  inf(15,1));
    obsInfo.Name = "observations";
    obsInfo.Description = "FSI_n, SampEn_n, Z_n, Zadj_n_prev, dZ_n_prev";

    %% Action specification
    actInfo = rlNumericSpec([3 1], ...
        LowerLimit = -ones(3,1), ...
        UpperLimit =  ones(3,1));
    actInfo.Name = "normalized_dZ_action";

    %% Create environment
    env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);
    env.ResetFcn = @(in)localResetFcn(in);

    %% Load pretrained agent and trainingStats, or create new ones
    if isfile(saveFile)
        S = load(saveFile, "agent", "trainingStats");
        agent = S.agent;

        if isfield(S, "trainingStats")
            trainingStatsPrev = S.trainingStats;
        else
            trainingStatsPrev = [];
        end

        disp("Loaded pretrained agent.");
    else
        agent = rlSACAgent(obsInfo, actInfo);

        agentOpts = rlSACAgentOptions;
        agentOpts.SampleTime = TsRL;
        agentOpts.DiscountFactor = 0.99;
        agentOpts.MiniBatchSize = 64;
        agentOpts.ExperienceBufferLength = 10000;

        agentOpts.ActorOptimizerOptions.LearnRate = 1e-4;
        agentOpts.CriticOptimizerOptions(1).LearnRate = 1e-3;
        agentOpts.CriticOptimizerOptions(2).LearnRate = 1e-3;

        agentOpts.InfoToSave.ExperienceBuffer = true;

        agent.AgentOptions = agentOpts;

        trainingStatsPrev = [];
        disp("Created new agent.");
    end

    %% Fresh training options for first run only
    trainOpts = rlTrainingOptions( ...
        MaxEpisodes = workers, ...
        MaxStepsPerEpisode = 120, ...
        ScoreAveragingWindowLength = 5, ...
        Verbose = true, ...
        Plots = "none", ...
        UseParallel = true, ...
        SimulationStorageType = "none");

    %% Train: always feed old trainingStats back if it exists

    if isempty(gcp('nocreate'))
        parpool(workers);
    end

    if ~isempty(trainingStatsPrev)
        trainingStatsPrev.TrainingOptions.MaxEpisodes = ...
            trainingStatsPrev.TrainingOptions.MaxEpisodes + workers;
        trainingStats = train(agent, env, trainingStatsPrev);
    else
        trainingStats = train(agent, env, trainOpts);
    end

    delete(gcp('nocreate'));

    %% Save updated agent and updated trainingStats
    save(saveFile, "agent", "trainingStats", "-v7.3");
end

function localCleanup(mdl)
    if bdIsLoaded(mdl)
        bdclose(mdl);
    end
end

function in = localResetFcn(in)
end