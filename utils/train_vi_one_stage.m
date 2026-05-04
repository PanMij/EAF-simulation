function train_vi_one_stage( ...
    max_episode, useParallel, workers, ...
    MaxStepsPerEpisode, stage, resetParams)

    if nargin < 6 || isempty(resetParams)
        error("train_vi_one_stage:MissingResetParams", ...
            "resetParams must be provided explicitly as the sixth input argument.");
    end

    for k = 1:max_episode
        rl_vi_one_stage_resume(max_episode, ...
            useParallel, workers, MaxStepsPerEpisode, stage, resetParams);
    end
end

%% rl_vi_one_stage_resume
function rl_vi_one_stage_resume(max_episode, ...
    useParallel, workers, MaxStepsPerEpisode, stage, resetParams)

    mdl = resetParams.mdl;
    agentBlk = mdl + "/RL_sup/RL Agent";

    TsRL = 5.0;
    saveFile = "data/agent_sac_variable_impedance.mat";

    if ~useParallel
        workers = 1;
    end

    cleanupObj = onCleanup(@() localCleanup(mdl));

    %% Observation specification
    obsInfo = rlNumericSpec([5 1], ...
        LowerLimit = -inf(5,1), ...
        UpperLimit =  inf(5,1));
    obsInfo.Name = "observations";
    obsInfo.Description = "FSI_n, SampEn_n, Z_n, Zadj_n_prev, dZ_n_prev";

    %% Action specification
    actInfo = rlNumericSpec([1 1], ...
        LowerLimit = -ones(1,1), ...
        UpperLimit =  ones(1,1));
    actInfo.Name = "normalized_dZ_action";

    %% Create environment
    env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);

    % Pass resetParams into localResetFcn
    env.ResetFcn = @(in)localResetFcn(in, resetParams);

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
        agentOpts.MiniBatchSize = 128;
        agentOpts.ExperienceBufferLength = 100000;
        agentOpts.NumEpoch = 1;
        agentOpts.MaxMiniBatchPerEpoch = 100;

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
        MaxStepsPerEpisode = MaxStepsPerEpisode, ...
        ScoreAveragingWindowLength = 5, ...
        Verbose = true, ...
        Plots = "none", ...
        UseParallel = useParallel, ...
        SimulationStorageType = "file", ...
        SaveSimulationDirectory = "data/training_log/stage"+stage);

    %% Train: always feed old trainingStats back if it exists
    if useParallel && isempty(gcp('nocreate'))
        % trainOpts.ParallelizationOptions.Mode = "async";
        parpool(workers);
    end

    if ~isempty(trainingStatsPrev)
        maxEp = trainingStatsPrev.TrainingOptions.MaxEpisodes;
        trainingStatsPrev.TrainingOptions = trainOpts;
        % trainingStatsPrev.TrainingOptions.MaxEpisodes = maxEp + max_episode;
        trainingStatsPrev.TrainingOptions.MaxEpisodes = maxEp + workers;
        if useParallel
            trainingStatsPrev.TrainingOptions.ParallelizationOptions.WorkerRandomSeeds = ...
                randi(2^31-1, 1, workers);
        end
        trainingStats = train(agent, env, trainingStatsPrev);
    else
        if useParallel
            trainOpts.ParallelizationOptions.WorkerRandomSeeds = ...
                randi(2^31-1, 1, workers);
        end
        trainingStats = train(agent, env, trainOpts);
    end

    % if useParallel
    %     delete(gcp('nocreate'));
    % end

    %% Save updated agent and updated trainingStats
    save(saveFile, "agent", "trainingStats", "-v7.3");
end

function localCleanup(mdl)
    if bdIsLoaded(mdl)
        bdclose(mdl);
    end
end

function in = localResetFcn(in, resetParams)
%LOCALRESETFCN Reset function for variable-impedance RL training.
%
% Root input ports:
%   Z_init : 3x1 constant signal
%   l_slag : 3x1 ramp signal
%
% Required resetParams fields used here:
%   mdl, phaseIdx, Z_init0, hy_out0, TstopDefault,
%   rhoMin, rhoMax, ZadjScaleMin, ZadjScaleMax,
%   dZ_prev_scale_min, dZ_prev_scale_max, reward
%
% Optional resetParams field:
%   l_slagSlope : scalar or 3x1 ramp slope of l_slag [m/s]
%                 Default: zeros(3,1), which gives the old constant signal.
%
% Ramp definition:
%   l_slag(t) = l_slag0 + l_slagSlope * t,     0 <= t <= Tstop
%
% If l_slagSlope = 0, then l_slag(t) = l_slag0, so the signal is constant.

    mdl = resetParams.mdl;

    %% ---------------- User settings from struct ----------------

    phaseIdx = resetParams.phaseIdx;
    Z_init0 = resetParams.Z_init0(:);
    hy_out0 = resetParams.hy_out0;

    if phaseIdx < 1 || phaseIdx > numel(Z_init0)
        error("resetParams.phaseIdx must be between 1 and %d.", numel(Z_init0));
    end

    % Simulation stop time
    Tstop = resetParams.TstopDefault;

    %% ------------------------------------------------------------

    % Slag coverage ratio:
    %   rho_slag = l_slag / hy_out
    %
    % This preserves your original initial-condition behavior:
    % one random rho value is shared by all three phases.
    rho_nominal = resetParams.rhoMin ...
        + (resetParams.rhoMax - resetParams.rhoMin) * ones(3, 1) * rand();

    rho_slag = rho_nominal;

    % Initial slag height
    l_slag0 = rho_slag .* hy_out0;

    % Ramp slope of slag height [m/s].
    % A scalar slope is applied to all three phases.
    % If this field is absent, the old constant l_slag behavior is used.
    if isfield(resetParams, "l_slagSlope") && ~isempty(resetParams.l_slagSlope)
        l_slagSlope = resetParams.l_slagSlope(:);
        if isscalar(l_slagSlope)
            l_slagSlope = repmat(l_slagSlope, size(l_slag0));
        end
    else
        l_slagSlope = zeros(size(l_slag0));
    end

    if numel(l_slagSlope) ~= numel(l_slag0)
        error("resetParams.l_slagSlope must be a scalar or a %dx1 vector.", numel(l_slag0));
    end

    % End value of the ramp signal.
    l_slag_end = l_slag0 + l_slagSlope * Tstop;

    % Basic physical check. Remove this if negative values are intentionally tested.
    if any(l_slag0 < 0) || any(l_slag_end < 0)
        error("l_slag ramp becomes negative. Check resetParams.l_slagSlope or the rho range.");
    end

    %% Initial internal variables

    Zadj0 = zeros(size(Z_init0));

    ZadjScale = resetParams.ZadjScaleMin ...
        + (resetParams.ZadjScaleMax - resetParams.ZadjScaleMin) * rand();

    Zadj0(phaseIdx) = ZadjScale * Z_init0(phaseIdx);

    dZ_prev0 = zeros(size(Z_init0));
    dZ_prev0_scale = resetParams.dZ_prev_scale_min ...
        + (resetParams.dZ_prev_scale_max - resetParams.dZ_prev_scale_min) * rand();
    dZ_prev0(phaseIdx) = dZ_prev0_scale * Z_init0(phaseIdx);

    in = setVariable(in, "Z_adj_init", Zadj0, "Workspace", mdl);
    in = setVariable(in, "dZ_prev_init", dZ_prev0, "Workspace", mdl);

    %% Set reward weights

    reward = resetParams.reward;

    in = setVariable(in, "alphaF", reward.alphaF, "Workspace", mdl);
    in = setVariable(in, "alphaS", reward.alphaS, "Workspace", mdl);
    in = setVariable(in, "beta_effort", reward.beta_effort, "Workspace", mdl);
    in = setVariable(in, "beta_freeze", reward.beta_freeze, "Workspace", mdl);
    in = setVariable(in, "beta_smooth", reward.beta_smooth, "Workspace", mdl);
    in = setVariable(in, "beta_return", reward.beta_return, "Workspace", mdl);

    %% Feed root-level input ports using external input dataset

    % Z_init remains constant.
    % l_slag is now a ramp. With zero slope, the two rows are identical,
    % so this reduces to the previous constant-signal case.
    t = [0; Tstop];

    Z_init_data = repmat(Z_init0.', numel(t), 1);
    l_slag_data = [l_slag0.'; l_slag_end.'];

    Z_init_ts = timeseries(Z_init_data, t);
    l_slag_ts = timeseries(l_slag_data, t);

    Z_init_ts.Name = "Z_init";
    l_slag_ts.Name = "l_slag";

    inputDataset = Simulink.SimulationData.Dataset;
    inputDataset = inputDataset.addElement(Z_init_ts, "Z_init");
    inputDataset = inputDataset.addElement(l_slag_ts, "l_slag");

    in = setExternalInput(in, inputDataset);
end