%% ctrlSys_init.m
% Initialization script for EAF electrode control simulations

% ---------- Fundamental parameters ----------
Ts_ctrl  = 5e-3;   % Controller / gate sample time [s]
Ts_meas  = 1e-3;   % Measurement / RMS sample time [s]
Ts_local = 1e-5;   % Simscape local solver sample time [s]
f        = 50;     % AC source frequency [Hz]

% Choose a fundamental fixed-step size for Simulink (if needed)
Ts_main  = Ts_meas;       % Main fixed-step size [s], e.g. 1e-3

disp('ctrlSys_init: simulation parameters initialized.');
