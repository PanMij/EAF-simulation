% startup.m - Automatically runs when MATLAB starts in this folder
% Adds necessary folders to MATLAB path

projectRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(projectRoot, 'data'));
addpath(fullfile(projectRoot, 'Identification'));
addpath(fullfile(projectRoot, 'models'));