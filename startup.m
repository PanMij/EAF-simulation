% startup.m - Automatically runs when MATLAB starts in this folder
% Adds necessary folders to MATLAB path

projectRoot = fileparts(mfilename('fullpath'));

folders = {
    'data'
    'Identification'
    'models'
    'ModelTests'
    'utils'
};

for i = 1:numel(folders)
    addpath(genpath(fullfile(projectRoot, folders{i})));
end