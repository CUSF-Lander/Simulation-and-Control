%% Kalman filter design parameters

% State Vectir
% x = [x y z vx vy vz]

% Sample time
dt = 0.01; 

% System Matrix-A 
A = A_numeric;


% Input Matrix B 
%This translates the linear accelerations from the IMU into x_dot (linear
%components)
B_kinematic = [ 0.5*dt^2  0         0         ;  % dx
                0         0.5*dt^2  0         ;  % dy
                0         0         0.5*dt^2  ;  % dz
                0         0         0         ;
                0         0         0         ;
                0         0         0         ;
                dt        0         0         ;  % dvx
                0         dt        0         ;  % dvy
                0         0         dt        ;
                0         0         0         ;
                0         0         0         ;
                0         0         0         ]; % dvz

B_numeric_angular_only = zeros(12,4);%This takes the control input and finds the angular x_dot components
B_numeric_angular_only(10:12,1:4) = B_numeric(10:12,1:4);

%B = B_numeric
B = B_numeric_angular_only;

%B_numeric is the bog standard B matrix

% Measurements Matrix-C 
%This is the C matrix for the kinematic approach, where the accelerations
%are treated separately
%It extracts the angular positions and  velocity
C_kinematic = [0 0 0 1 0 0 0 0 0 0 0 0;
               0 0 0 0 1 0 0 0 0 0 0 0;
               0 0 0 0 0 1 0 0 0 0 0 0;
               0 0 0 0 0 0 0 0 0 1 0 0;
               0 0 0 0 0 0 0 0 0 0 1 0;
               0 0 0 0 0 0 0 0 0 0 0 1];

%C = C_numeric;
C = C_kinematic;

% Process Noice Covariance-Q
% NEEDS TO BE ADAPTED FOR KINEMATIC APPROACH
Q = eye(12)*0.001;

% Measurement Noise Covariance-R
imu_variance = [0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01];
R_kinematic = diag(imu_variance(1:6));

%R_kinematic = diag(imu_variance); Full measurement matrix
R = R_kinematic;
% Calculate discrete steady-state kalman gain
G = eye(12)*1; %Process Noise Distribution Matrix. If the noise directly affects every state, G might be an Identity matrix.

A_nudge = A * 0.9999; %dirty step to avoid error
kf = dlqe(A_nudge,G,C,Q,R);
kf = round( kf, 5 );
