% startup.m
% Add project subfolders to path
addpath(genpath('sensors'));     % genpath adds the folder and all its subfolders

% Run variable initialization
simulink_vars; 

%Generate matrices
expanded_jacobian;
offline_kalman_expanded;

%Generate control variables
new_strategy;

fprintf('Drone Project Environment Loaded.\n');

