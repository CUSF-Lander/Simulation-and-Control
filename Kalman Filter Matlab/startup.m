% startup.m
% Add project subfolders to path
addpath(genpath('sensors'));     % genpath adds the folder and all its subfolders

% Run your variable initialization
simulink_vars; 

%Generate matrices
expanded_jacobian;
offline_kalman_expanded;

fprintf('Drone Project Environment Loaded.\n');