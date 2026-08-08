% startup.m

clc
clear

% Add project subfolders to path
addpath(genpath('sensors'));     % genpath adds the folder and all its subfolders

% Run variable initialization   
simulink_vars; 

%Generate matrices
generate_jacobians;
calculate_kalman_gain;

%Generate control variables
new_strategy_integrated;

fprintf('Drone Project Environment Loaded.\n');

