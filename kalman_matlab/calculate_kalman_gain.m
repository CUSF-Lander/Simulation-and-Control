%% Calculates kalman gain at equlibrium

% System Matrix-A 
A = A_numeric;

% Input Matrix B 
B = B_numeric;

% Measurements Matrix-C 
H_imu = H_numeric; % IMU

H_barometer = [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]; % barometer

H_gps = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]; % gps
       
H_100 = [H_imu; H_barometer];       % 100Hz Measurement Matrix (excludes gps)
H_20 = [H_imu; H_barometer; H_gps]; % 20Hz Measurement Matrix (includes gps)
H = H_imu;

% Process Noice Covariance-Q
process_noise_vector = zeros(18,1);
process_noise_vector(13:18,1) = process_noise_variance;
Q = diag(process_noise_vector);

% Measurement Noise Covariance-R
R = diag(imu_variance);
R_100 = diag([imu_variance barometer_variance]);
R_20 = diag([imu_variance barometer_variance gps_variance]);

% Process Noise Distribution Matrix. - Noise enters only through acceleration
G = zeros(18, 18); 
G(13:18, 13:18) = eye(6); 


% Calculate discrete steady-state kalman gain
A_nudge = A * 0.9999; %dirty step to avoid error

kf_100 = dlqe(A_nudge, G, H_100, Q, R_100);
kf_100 = round(kf_100, 5);

kf_20 = dlqe(A_nudge, G, H_20, Q, R_20);
kf_20 = round(kf_20, 5);


%Write matrices to CSV file
writematrix(kf_100,'kf_100.csv');
writematrix(kf_20,'kf_20.csv');