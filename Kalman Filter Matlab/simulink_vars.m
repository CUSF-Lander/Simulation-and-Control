% Drone Constants
Kt = 0.2;      % N / (1/s^2) 
Jx = 0.01;          % Kg * m^2
Jy = 0.01;    % Kg * m^2
Jz = 0.03;    % Kg * m^2
m = 0.75;           % Kg
g = 9.807;          % m/s^2
a = 0.1;            % m
b = 0.1;            % m 
Km = 0.2;          % Moment arm constant

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

% control input: [a1 a2 wt1 wt 2]
% wt 1 and 2 are rotor rotation rates

process_noise_variance = 0.0001; %this is only the noise in the acceleration - VARIANCE?
initial_noise = 0.0001;

% ERROR SQUARED
imu_variance = [0.029,0.029,0.029,0.1225,0.1225,0.1225,0.0037,0.0037,0.0037]; %angular velocity, linear acceleration, angular acceleration
barometer_variance = [0.0625]; % 25 cm error
gps_variance = 0.0025 * ones(1,3); % 5 cm error

imu_sample_time = 1/100;
barometer_sample_time = 1/100;
gps_sample_time = 1/20;

ekf_sample_time = min([imu_sample_time, barometer_sample_time, gps_sample_time]);

imu_drift = [0.0,0.0,0.0,0.00,0.00,0.00,0.0,0.0,0.0]; %angular velocity, linear acceleration, angular acceleration

%imu_drift = zeros(9,1);