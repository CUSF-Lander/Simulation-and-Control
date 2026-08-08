%% Calculates kalman gain at equlibrium

% Sample time
%dt = 0.01; 

% System Matrix-A 
A = A_numeric;

% Input Matrix B 
B = B_numeric;

% Measurements Matrix-C 
C_imu = C_numeric; % IMU

C_barometer = [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]; % barometer

C_gps = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]; % gps
       
C_100 = [C_imu; C_barometer];
C_20 = [C_imu; C_barometer; C_gps];
C = C_imu;

% Process Noice Covariance-Q
process_noise_vector = zeros(18,1);
process_noise_vector(13:18,1) = process_noise_variance;
Q = diag(process_noise_vector); %process_noise_variance;

% Measurement Noise Covariance-R
R = diag(imu_variance);
R_100 = diag([imu_variance barometer_variance]);
R_20 = diag([imu_variance barometer_variance gps_variance]);

% Process Noise Distribution Matrix. - Noise enters only in acceleration
G = zeros(18, 18); 
G(13:18, 13:18) = eye(6); 

% Calculate discrete steady-state kalman gain
A_nudge = A * 0.9999; %dirty step to avoid error
kf = dlqe(A_nudge,G,C,Q,R);
kf = round( kf, 5 );

kf_100 = dlqe(A_nudge,G,C_100,Q,R_100);
kf_100 = round( kf_100, 5 );

kf_20 = dlqe(A_nudge,G,C_20,Q,R_20);
kf_20 = round( kf_20, 5 );
