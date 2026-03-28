% Drone Constants
Kt = 0.021;      % N / (1/s^2) 
Jx = 0.1;          % Kg * m^2
Jy = 0.1;    % Kg * m^2
Jz = 0.3;    % Kg * m^2
m = 1.0;           % Kg
g = 9.807;          % m/s^2
a = 0.1;            % m
b = 0;            % m 
Km = 1.05e-3;          % Moment arm constant

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


weight = m*g;
stationary_wt = sqrt(weight/(Kt * 2));

%% EQUILIBRIUM

% equilibrium point
x_eq = zeros(18,1);
x_eq(3) = 3;

% Calculate motor speed (wt) needed to hover: Ft = Kt*(wt1^2 + wt2^2) = m*g
u_eq = [0; 0; stationary_wt; stationary_wt]; 
%u_eq = [0; 0; 0; 0]; 
process_noise_variance = 0.01; %this is only the noise in the acceleration - VARIANCE?
initial_noise = 0.000001;


% Initial State
X0 = [0;0;3; 0;0;0; 0;0;0; 0;0;0];
U_initial = [0;0;9.81;0]; %In terms of new control scheme

%% Sensors

% ERROR SQUARED
imu_variance = [0.0037,0.0037,0.0037,0.1225,0.1225,0.1225,0.0029,0.0029,0.0029]; %angular velocity, linear acceleration, angular acceleration
barometer_variance = [0.0625]; % 25 cm error
gps_variance = 0.0025 * ones(1,3); % 5 cm error

imu_sample_time = 1/100;
barometer_sample_time = 1/100;
gps_sample_time = 1/20;

ekf_sample_time = min([imu_sample_time, barometer_sample_time, gps_sample_time]);

imu_drift = [0.0,0.0,0.0,0.00,0.00,0.00,0.0,0.0,0.0]; %angular velocity, linear acceleration, angular acceleration

%imu_drift = zeros(9,1);


