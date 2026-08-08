%% Parameters

% Drone Constants
Kt = 0.021;       % N / (1/s^2) 
Jx = 0.1;         % Kg * m^2
Jy = 0.1;         % Kg * m^2
Jz = 0.3;         % Kg * m^2
m = 1.0;          % Kg
g = 9.807;        % m/s^2
a = 0.1;          % m
b = 0;            % m 
Km = 1.05e-3;     % Moment arm constant

% Time step
dt = 0.01;

% Put parameters into a struct that can be input into the kalman_filter()
% function
params = struct();

params.m  = m;   
params.g  = g;   
params.Kt = Kt;     
params.Km = Km;     
params.Jx = Jx;    
params.Jy = Jy;    
params.Jz = Jz;    
params.a  = a;     
params.b  = b;     
params.dt = dt;


%% EQUILIBRIUM

% equilibrium point
x_eq = zeros(18,1);
x_eq(3) = 3;

% Calculate motor speed (wt) needed to hover: Ft = Kt*(wt1^2 + wt2^2) = m*g
weight = m*g;
stationary_wt = sqrt(weight/(Kt * 2));

%Control inputs at equilibrium
u_eq = [0; 0; stationary_wt; stationary_wt]; 

%Process noise (the 'physics' noise) and initial noise
angular_acc_noise = 10*ones(3,1);
TUNING = 1; %Can modify this as a way to tune the kalman filter
process_noise_variance = TUNING*[1;1;1;angular_acc_noise]; % [x,y,z,p,q,u] %this is only the noise in the acceleration
initial_noise = 0.000001; % Effectively zero

% Initial State
X0 = [0;0;3; 0;0;0; 0;0;0; 0;0;0]; % [position (m), euler angles (rad), velocity (m/s), angular velocity (rad/s)]
U_initial = [0;0;9.81;0]; %In terms of new control scheme


%% Sensors

% Variance of sensors
imu_variance = [0.0037,0.0037,0.0037,0.1225,0.1225,0.1225,0.0029,0.0029,0.0029]; %[euler angles (rad), linear acceleration(m/s^2), angular velocity(rad/s)]
barometer_variance = [0.0625]; % (m) - 0.25m error
gps_variance = 0.0025 * ones(1,3); % (m) - 0.05m error

% Sampling frequency (s)
imu_sample_time = 1/100; 
barometer_sample_time = 1/100;
gps_sample_time = 1/20;

ekf_sample_time = min([imu_sample_time, barometer_sample_time, gps_sample_time]);
