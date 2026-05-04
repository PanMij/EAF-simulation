function [agent, tfdStats] = train_offline(stage, opts)
%TRAIN_OFFLINE Train the current SAC agent from a saved stage replay buffer.

    if nargin < 1 || isempty(stage)
        error("train_offline:MissingStage", ...
            "stage must be provided as the first input argument.");
    end
    if nargin < 2 || isempty(opts)
        opts = struct();
    end

    opts = localNormalizeOfflineOpts(stage, opts);

    if ~isfile(opts.AgentFile)
        error("train_offline:MissingAgentFile", ...
            "Agent checkpoint %s does not exist.", opts.AgentFile);
    end
    if ~isfile(opts.BufferFile)
        error("train_offline:MissingBufferFile", ...
            "Stage replay buffer %s does not exist.", opts.BufferFile);
    end

    S = load(opts.AgentFile);
    if ~isfield(S, "agent")
        error("train_offline:MissingAgent", ...
            "Agent checkpoint %s does not contain an agent variable.", opts.AgentFile);
    end

    agent = S.agent;
    if isfield(S, "trainingStats")
        trainingStats = S.trainingStats;
    else
        trainingStats = [];
    end
    if isfield(S, "curriculumState")
        curriculumState = S.curriculumState;
    else
        curriculumState = struct();
    end

    stageBuffer = localLoadStageBuffer(opts.BufferFile);
    bufferLength = localBufferLength(stageBuffer);
    if bufferLength <= 0
        error("train_offline:EmptyBuffer", ...
            "Stage replay buffer %s has no experiences.", opts.BufferFile);
    elseif bufferLength < agent.AgentOptions.MiniBatchSize
        warning("train_offline:SmallBuffer", ...
            "Stage replay buffer %s has only %d experiences, which is less than the agent's mini-batch size of %d. Training may not be effective.", ...
            opts.BufferFile, bufferLength, agent.AgentOptions.MiniBatchSize);
        return;
    end

    agent.ExperienceBuffer = stageBuffer;
    agent = localMoveAgentToDevice(agent, opts.UseGPU);

    tfdOpts = rlTrainingFromDataOptions( ...
        MaxEpochs = opts.MaxEpochs, ...
        NumStepsPerEpoch = opts.NumStepsPerEpoch, ...
        Plots = opts.Plots, ...
        Verbose = opts.Verbose);

    tic;
    tfdStats = trainFromData(agent, tfdOpts);
    elapsedSeconds = toc;

    curriculumState.activeStage = stage;
    curriculumState.lastOfflineStage = stage;
    curriculumState.lastOfflineBufferFile = opts.BufferFile;
    curriculumState.lastOfflineCompletedAt = string(datetime("now"));

    localEnsureFolder(fileparts(opts.AgentFile));
    save(char(opts.AgentFile), "agent", "trainingStats", "curriculumState", "-v7.3");

    localEnsureFolder(opts.OfflineDir);
    metadata = struct( ...
        "stage", stage, ...
        "bufferLength", bufferLength, ...
        "createdAt", string(datetime("now")), ...
        "agentFile", opts.AgentFile, ...
        "bufferFile", opts.BufferFile, ...
        "elapsedSeconds", elapsedSeconds);

    offlineAgentFile = fullfile(opts.OfflineDir, ...
        "stage_"+string(stage)+"_offline_agent.mat");
    save(char(offlineAgentFile), ...
        "agent", "trainingStats", "tfdStats", "metadata", "-v7.3");
end

function opts = localNormalizeOfflineOpts(stage, opts)
    if ~isstruct(opts)
        error("train_offline:InvalidOptions", ...
            "opts must be a struct.");
    end

    replayBufferDir = localGetOption(opts, "ReplayBufferDir", ...
        fullfile("data", "replay_buffers"));
    replayBufferDir = localGetOption(opts, "BufferDir", replayBufferDir);

    opts.AgentFile = localGetOption(opts, "AgentFile", ...
        "data/agent_sac_variable_impedance.mat");
    opts.BufferFile = localGetOption(opts, "BufferFile", ...
        fullfile(replayBufferDir, "stage_"+string(stage)+"_buffer.mat"));
    opts.OfflineDir = localGetOption(opts, "OfflineDir", ...
        fullfile("data", "offline_training"));
    opts.MaxEpochs = localGetOption(opts, "MaxEpochs", 20);
    opts.NumStepsPerEpoch = localGetOption(opts, "NumStepsPerEpoch", 1000);
    opts.UseGPU = localGetOption(opts, "UseGPU", localCanUseGPU());
    opts.Plots = localGetOption(opts, "Plots", "none");
    opts.Verbose = localGetOption(opts, "Verbose", true);

    opts.AgentFile = string(opts.AgentFile);
    opts.BufferFile = string(opts.BufferFile);
    opts.OfflineDir = string(opts.OfflineDir);
    opts.UseGPU = logical(opts.UseGPU);
end

function value = localGetOption(opts, fieldName, defaultValue)
    fieldName = char(fieldName);
    if isfield(opts, fieldName) && ~isempty(opts.(fieldName))
        value = opts.(fieldName);
    else
        value = defaultValue;
    end
end

function tf = localCanUseGPU()
    try
        tf = canUseGPU;
    catch
        tf = false;
    end
end

function stageBuffer = localLoadStageBuffer(bufferFile)
    S = load(bufferFile);

    if isfield(S, "stageBuffer")
        stageBuffer = S.stageBuffer;
    elseif isfield(S, "replayBuffer")
        stageBuffer = S.replayBuffer;
    elseif isfield(S, "buffer")
        stageBuffer = S.buffer;
    else
        error("train_offline:MissingStageBuffer", ...
            "Replay buffer file %s does not contain a stageBuffer variable.", bufferFile);
    end
end

function bufferLength = localBufferLength(buffer)
    bufferLength = 0;
    if isempty(buffer)
        return;
    end
    if isprop(buffer, "Length")
        bufferLength = buffer.Length;
    end
end

function agent = localMoveAgentToDevice(agent, useGPU)
    if ~useGPU
        return;
    end

    try
        actor = getActor(agent);
        if isprop(actor, "UseDevice")
            actor.UseDevice = "gpu";
            agent = setActor(agent, actor);
        end

        critics = getCritic(agent);
        for k = 1:numel(critics)
            if isprop(critics(k), "UseDevice")
                critics(k).UseDevice = "gpu";
                agent = setCritic(agent, critics(k));
            end
        end
    catch ME
        warning("train_offline:GPUSetupFailed", ...
            "Could not move the agent to GPU. Continuing on the default device. %s", ...
            ME.message);
    end
end

function localEnsureFolder(folderName)
    if strlength(string(folderName)) == 0
        return;
    end
    if ~isfolder(folderName)
        mkdir(folderName);
    end
end
