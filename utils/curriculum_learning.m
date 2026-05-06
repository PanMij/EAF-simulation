clc;clear;
close all;

useParallel = false;
workers = 4;

Ts_sup = 5;

workflowOpts = struct();
workflowOpts.AgentFile = "data/agent_sac_variable_impedance.mat";
workflowOpts.ReplayBufferDir = "data/replay_buffers";
workflowOpts.TrainingLogDir = "data/training_log";

offlineOpts = struct();
offlineOpts.AgentFile = workflowOpts.AgentFile;
offlineOpts.ReplayBufferDir = workflowOpts.ReplayBufferDir;
offlineOpts.OfflineDir = "data/offline_training";

globalMaxEpochs = 10;
globalNumStepsPerEpoch = 100;

%% stage 1
disp("==================== Stage 1 ====================");

numEpisodes = 100;
numSteps = 10;

resetParams = struct();
resetParams.mdl = "ctrlSys_rl_env_sg";
resetParams.phaseIdx = 2;
% Nominal impedance of three phases
resetParams.Z_init0 = [0.055; 0.055; 0.055];
% Fixed arc length / hydraulic output
resetParams.hy_out0 = 0.2;
% Default stop time used if model StopTime is invalid
resetParams.TstopDefault = numSteps * Ts_sup;

% Slag coverage ratio range
resetParams.rhoMin = 0.95;
resetParams.rhoMax = 1.05;

% Initial impedance adjustment range
% Original code:
% Zadj0(phaseIdx) = -0.02 * Z_init0(phaseIdx) ...
%                   + 0.02 * Z_init0(phaseIdx) * rand();
%
% This is equivalent to a random scale in [-0.02, 0].
resetParams.ZadjScaleMin = -0.02;
resetParams.ZadjScaleMax = 0.0;

resetParams.dZ_prev_scale_max = 0.0;
resetParams.dZ_prev_scale_min = -0.0;

% Reward weights
resetParams.reward = struct( ...
    'alphaF', 1, ...
    'alphaS', 1, ...
    'beta_effort', 0.3, ...
    'beta_freeze', 0.0, ...
    'beta_smooth', 0.0, ...
    'beta_return', 0.0);

train_vi_one_stage(ceil(numEpisodes / workers), useParallel, workers, numSteps, 1, resetParams, workflowOpts);
offlineOpts = getOfflineOpts(5, offlineOpts, globalNumStepsPerEpoch, globalMaxEpochs);
train_offline(1, offlineOpts);


%% stage 2
disp("==================== Stage 2 ====================");

numEpisodes = 150;
numSteps = 20;

resetParams = struct();
resetParams.mdl = "ctrlSys_rl_env_sg";
resetParams.phaseIdx = 2;
resetParams.Z_init0 = [0.055; 0.055; 0.055];
resetParams.hy_out0 = 0.2;
resetParams.TstopDefault = numSteps * Ts_sup;

resetParams.rhoMin = 0.85;
resetParams.rhoMax = 0.95;

resetParams.ZadjScaleMin = -0.03;
resetParams.ZadjScaleMax = 0.0;

resetParams.dZ_prev_scale_max = 0.005;
resetParams.dZ_prev_scale_min = -0.005;
% Reward weights
resetParams.reward = struct( ...
    'alphaF', 1, ...
    'alphaS', 1, ...
    'beta_effort', 0.3, ...
    'beta_freeze', 0.0, ...
    'beta_smooth', 0.0, ...
    'beta_return', 0.0);

train_vi_one_stage(ceil(numEpisodes / workers), useParallel, workers, numSteps, 2, resetParams, workflowOpts);
offlineOpts = getOfflineOpts(5, offlineOpts, globalNumStepsPerEpoch, globalMaxEpochs);
train_offline(2, offlineOpts);


%% stage 3
disp("==================== Stage 3 ====================");

numEpisodes = 150;
numSteps = 20;

resetParams = struct();
resetParams.mdl = "ctrlSys_rl_env_sg";
resetParams.phaseIdx = 2;
resetParams.Z_init0 = [0.055; 0.055; 0.055];
resetParams.hy_out0 = 0.2;
resetParams.TstopDefault = numSteps * Ts_sup;

resetParams.rhoMin = 0.85;
resetParams.rhoMax = 1.05;

resetParams.ZadjScaleMin = -0.05;
resetParams.ZadjScaleMax = 0.0;

resetParams.dZ_prev_scale_max = 0.005;
resetParams.dZ_prev_scale_min = -0.01;
% Reward weights
resetParams.reward = struct( ...
    'alphaF', 1, ...
    'alphaS', 1, ...
    'beta_effort', 0.3, ...
    'beta_freeze', 0.3, ...
    'beta_smooth', 0.0, ...
    'beta_return', 0.0);

train_vi_one_stage(ceil(numEpisodes / workers), useParallel, workers, numSteps, 3, resetParams, workflowOpts);
offlineOpts = getOfflineOpts(5, offlineOpts, globalNumStepsPerEpoch, globalMaxEpochs);
train_offline(3, offlineOpts);


%% stage 4
disp("==================== Stage 4 ====================");

numEpisodes = 200;
numSteps = 20;

resetParams = struct();
resetParams.mdl = "ctrlSys_rl_env_sg";
resetParams.phaseIdx = 2;
resetParams.Z_init0 = [0.055; 0.055; 0.055];
resetParams.hy_out0 = 0.2;
resetParams.TstopDefault = numSteps * Ts_sup;

resetParams.rhoMin = 0.95;
resetParams.rhoMax = 1.05;

resetParams.ZadjScaleMin = -0.1;
resetParams.ZadjScaleMax = -0.05;

resetParams.dZ_prev_scale_max = 0.005;
resetParams.dZ_prev_scale_min = -0.005;

resetParams.reward = struct( ...
    'alphaF', 1, ...
    'alphaS', 1, ...
    'beta_effort', 0.3, ...
    'beta_freeze', 0.3, ...
    'beta_smooth', 0.0, ...
    'beta_return', 1.0);

train_vi_one_stage(ceil(numEpisodes / workers), useParallel, workers, numSteps, 4, resetParams, workflowOpts);
offlineOpts = getOfflineOpts(5, offlineOpts, globalNumStepsPerEpoch, globalMaxEpochs);
train_offline(4, offlineOpts);


%% stage 5
disp("==================== Stage 5 ====================");

numEpisodes = 250;
numSteps = 30;

resetParams = struct();
resetParams.mdl = "ctrlSys_rl_env_sg";
resetParams.phaseIdx = 2;
resetParams.Z_init0 = [0.055; 0.055; 0.055];
resetParams.hy_out0 = 0.2;
resetParams.TstopDefault = numSteps * Ts_sup;

resetParams.rhoMin = 0.88;
resetParams.rhoMax = 1.05;

resetParams.ZadjScaleMin = -0.1;
resetParams.ZadjScaleMax = -0.00;

resetParams.dZ_prev_scale_max = 0.01;
resetParams.dZ_prev_scale_min = -0.03;

resetParams.reward = struct( ...
    'alphaF', 1, ...
    'alphaS', 1, ...
    'beta_effort', 0.3, ...
    'beta_freeze', 0.3, ...
    'beta_smooth', 0.3, ...
    'beta_return', 1.0);

train_vi_one_stage(ceil(numEpisodes / workers), useParallel, workers, numSteps, 5, resetParams, workflowOpts);
offlineOpts = getOfflineOpts(5, offlineOpts, globalNumStepsPerEpoch, globalMaxEpochs);
train_offline(5, offlineOpts);


%% Helper functions

function offlineOpts = getOfflineOpts(reuseRatio, opts, globalNumStepsPerEpoch, globalMaxEpochs)
    S = load(opts.AgentFile);
    N = S.agent.ExperienceBuffer.Length;
    B = S.agent.AgentOptions.MiniBatchSize;

    totalUpdates = ceil(reuseRatio * N / B);

    NumStepsPerEpoch = min(globalNumStepsPerEpoch, totalUpdates);
    MaxEpochs = min(globalMaxEpochs, ceil(totalUpdates / NumStepsPerEpoch));

    opts.MaxEpochs = MaxEpochs;
    opts.NumStepsPerEpoch = NumStepsPerEpoch;
    offlineOpts = opts;
end