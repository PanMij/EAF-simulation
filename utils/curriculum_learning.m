clc;clear;
close all;

useParallel = true;
workers = 4;

Ts_sup = 5;

%% stage 1
disp("==================== Stage 1 ====================");

numEpisodes = 25;
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

train_vi_one_stage(numEpisodes, useParallel, workers, numSteps, 1, resetParams);


%% stage 2
disp("==================== Stage 2 ====================");

numEpisodes = 50;
numSteps = 20;

resetParams = struct();
resetParams.mdl = "ctrlSys_rl_env_sg";
resetParams.phaseIdx = 2;
resetParams.Z_init0 = [0.055; 0.055; 0.055];
resetParams.hy_out0 = 0.2;
resetParams.TstopDefault = numSteps * Ts_sup;

resetParams.rhoMin = 0.85;train_vi_one_episode
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

train_vi_one_stage(numEpisodes, useParallel, workers, numSteps, 2, resetParams);


%% stage 3
disp("==================== Stage 3 ====================");

numEpisodes = 50;
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

train_vi_one_stage(ceil(numEpisodes / workers), useParallel, workers, numSteps, 3, resetParams);


%% stage 4
disp("==================== Stage 4 ====================");

numEpisodes = 50;
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

train_vi_one_stage(ceil(numEpisodes / workers), useParallel, workers, numSteps, 4, resetParams);


%% stage 5
disp("==================== Stage 5 ====================");

numEpisodes = 100;
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

train_vi_one_stage(ceil(numEpisodes / workers), useParallel, workers, numSteps, 5, resetParams);