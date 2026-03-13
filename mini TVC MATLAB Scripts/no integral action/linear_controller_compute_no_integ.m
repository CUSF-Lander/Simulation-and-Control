clc
clear

syms x y z vx vy vz p q u wx wy wz % States
syms a1 a2 wt1 wt2 % Inputs
syms Kt Km m Jx Jy Jz a L g

%% Definitions
% Transformation matrix from body angular velocity to Tait-Bryan Rates

W = [ 1, 0,      -sin(q)         ;
      0, cos(p),  cos(q)*sin(p)  ;
      0, -sin(p), cos(q)*cos(p) ];

Winv = simplify(inv(W));
disp(Winv)

% Rotation matrix from body to world frame, input: roll, pitch, yaw
R = [cos(q)*cos(u), sin(p)*sin(q)*cos(u)-cos(p)*sin(u), cos(p)*sin(q)*cos(u)+sin(p)*sin(u) ;
     cos(q)*sin(u), sin(p)*sin(q)*sin(u)+cos(p)*cos(u), cos(p)*sin(q)*sin(u)-sin(p)*cos(u) ;
     -sin(q),       sin(p)*cos(q),                      cos(p)*cos(q)                     ];

% Matrix of mass inertia
J = [Jx 0  0  ;
     0  Jy 0  ;
     0  0  Jz];

% Magnitude of force from rotor
Ft = Kt * (wt1^2 + wt2 ^2);

% Direction of gimbal
X_force = [sin(a2) * cos(a1);
           -sin(a1);
           cos(a2) * cos(a1)];

% Force from rotors
fb = Ft * X_force;

% Moment from thrust force
t1 = a * Ft * [-sin(a1); 
               -sin(a2) * cos(a1);
               0];


% reaction moment from motor
t2 = (Km * (wt1^2 - wt2^2)) * X_force;

% Resultant torque
tb = t1 + t2;

% State vectors used for derivation
nw = [p q u].';     % Attitude (world frame)
wb = [wx wy wz].';  % Angular velocity (body frame)
pw = [x y z].';     % Position (world frame)
vb = [vx vy vz].';  % Velocity (body frame)

% Total state vector
X = [nw; wb; pw; vb];
X_red = [nw; wb; pw(3); vb(3)]; % Reduced state vector (only attitude and altitude)
X_hor = [ pw(1); pw(2); vb(1); vb(2) ]; % Reduced state vector for horizontal movements

X_roll = [nw(1); wb(1); pw(1); vb(1)];

% Input vector 
U = [a1; a2; wt1; wt2];

% Input vector for horizontal model
U_hor = [p; q];

% Roll 
U_roll = [a1; a2];

%% Rotational dynamics

nw_dot = Winv * wb;
wb_dot = J \ (tb - cross(wb, J * wb)  );

%% Translational dynamics

pw_dot = R * vb;
vb_dot = 1/m * ( fb -  R.' * [0 0 m*g].');

% Translational dynamics in world
vw_dot = 1/m * R * fb - [0 0 m*g].';

%% Combined non-linear model

f = [ nw_dot  ;
      wb_dot  ;
      pw_dot  ;
      vb_dot ];

disp(f)

% Reduced non-linear model
f_red = [ nw_dot     ;
          wb_dot     ;
          pw_dot(3)  ;
          vb_dot(3) ];
      
% Horizontal non-linear model
f_hor = [ pw_dot(1) ;
          pw_dot(2) ;
          vb_dot(1) ;
          vb_dot(2)];
      
      
f_roll = [ nw_dot(1) ;
          wb_dot(1)  ;
          pw_dot(1)  ;
          vb_dot(1) ];

%% Linearization

% Using the Jacobian method, the set of nonlinear system equations are
% linearized around the hover point

A = jacobian(f, X);
B = jacobian(f, U);

disp(A)

% Reduced model (only z-axis in position)
A2 = jacobian(f_red, X_red);
B2 = jacobian(f_red, U);

disp(A2)

% Horizontal model (only x- and y-direction)
A3 = jacobian( f_hor, X_hor );
B3 = jacobian( f_hor, U_hor );

disp(A3)


% Single dimension model (roll/x axis)
A4 = jacobian( f_roll, X_roll );
B4 = jacobian( f_roll, U_roll );

disp(A)
% The A and B matrixes are now filled with partial derivatives, similar to
% an taylor expansion to approximate a nonlinear function/ODE
% We must insert the state- and input-values at the operating point

% All the states is zero at the hover point
x = 0; y = 0; z = 0; vx = 0; vy = 0; vz = 0; p = 0; q = 0; u = 0; wx = 0; wy = 0; wz = 0;

% All input is not zero!
a1 = 0; a2 = 0;
wt1 = 129.5; % kRPM = hoverpoint
wt2 = 129.5;

% Drone Constants
Kt = 0.001;      % N / (1/s^2)
Jx = 0.1;          % Kg * m^2
Jy = 0.1;    % Kg * m^2
Jz = 0.03;    % Kg * m^2
m = 0.75;           % Kg
g = 9.807;          % m/s^2
a = 0.1;            % m
b = 0.1;            % m 
Km = 0.00016;          % Moment arm constant

% substitution
B_numeric = subs(B);
disp(B_numeric)

A_numeric = subs(A);
disp(A_numeric)

J_mat = [Jx, 0, 0;
         0, Jy, 0;
         0, 0, Jz];

J_mat_inv = J_mat ^ -1;



% Now the A and B matrixes can be evaluted, yield the full linear model
% around the hover point.
A_sys = double(vpa(A_numeric, 4));
B_sys = double(vpa(B_numeric, 4));
C_sys = eye(12);
D_sys = zeros(12,4);


%Checking controllability
Co = ctrb(A_sys, B_sys);
rank_Co = rank(Co);
disp(['Rank of controllability matrix: ', num2str(rank_Co)]);
disp(['State dimension: ', num2str(size(A_sys,1))]);

% Reduced model (attitude only)
A_red_numeric = subs(A2);
B_red_numeric = subs(B2);
A_red = double(vpa(A_red_numeric, 4));
B_red = double(vpa(B_red_numeric, 4));
size(B_red)
C_red = eye(8);
D_red = zeros(8,4);

% Horizontal model (position)
A_hor_numeric= subs(A3);
B_hor_numeric = subs(B3);
A_hor = double(vpa(A_hor_numeric,4));
B_hor = double(vpa(B_hor_numeric,4));
size(B_hor)
C_hor = eye(4);
D_hor = zeros(4,2);


% Reduced model with integral action states
G_hov = [ 0 0 0 0 0 0 1 0 ]; % z
   
A_int = [A_red; G_hov];
A_int = [A_int zeros(9,1) ];
B_int = [B_red; zeros(1,4) ];
C_int = eye(9);
D_int = zeros(9,4);


% Horizontal model with integral action states
G_pos = [ 1 0 0 0; 
         0 1 0 0 ];

A_hint = [A_hor; G_pos];
A_hint = [A_hint zeros(6,2) ];
B_hint = [B_hor; zeros(2,2) ];
C_hint = eye(6);
D_hint = zeros(6,2);

%% Open Loop dynamics

sys = ss(A_sys,B_sys,C_sys,D_sys);
sys_red = ss(A_red,B_red,C_red,D_red); % Reduced, attitude and altitude only, no integral
sys_int = ss(A_int,B_int,C_int, D_int);
sys_hor = ss(A_hor, B_hor, C_hor, D_hor); % xy position only
sys_hint = ss(A_hint, B_hint, C_hint, D_hint);

%% Design controller

% Bryson's Rule. 
% Max angle of 0.3 radians. Maximum angular rate of 5 rad/second
Q = [ 1/0.1^2     0        0        0      0      0      0        0    ;  % Roll
      0        1/0.1^2     0        0      0      0      0        0    ;  % Pitch
      0        0        1/1^2    0      0      0      0        0       ;  % Yaw
      0        0        0        1/1^2  0      0      0        0       ;  % omega_x
      0        0        0        0      1/1^2  0      0        0       ;  % omega_y
      0        0        0        0      0      1/2^2  0        0       ;  % omega_z
      0        0        0        0      0      0      1/0.5^2    0       ;  % z
      0        0        0        0      0      0      0        1/1^2  ]; % v_z

Q_red = Q;  
  
% Integral action  
Q(9,9) = [ 1/0.15^2 ]; % z
      
% Max actuation angle of +-10 degress

R = [1/0.03    0       0       0       ;   % a1
     0         1/0.03  0       0       ;   % a2
     0         0       1       0       ;   % wt1
     0         0       0       1       ;]; % wt2

% Compute "optimal" controller
K_hov0 = lqr(sys_red, Q_red, R);
disp(K_hov0)


% Compute integral limit matching the steady-state motor velocity

int_lim = 5.0;

sys_d = c2d(sys_int, 0.008, 'zoh' );

K_lqrd = dlqr(sys_d.A, sys_d.B, Q, R);

% matrix_to_cpp( K_hov )

% Calcuate closed loop system
% figure(1)
%cl_sys = ss((A_red - B_red*K_red), B_red, C_red, D_red );
sys_cl_hov = feedback( sys_red*K_hov0, eye(8));

figure(1)
pzmap(sys_cl_hov);
[p,z] = pzmap(sys_cl_hov);
grid on

Q_pos = [ 1/0.5^2  0         0        0        ;
          0         1/0.5^2  0        0        ;
          0         0         1/2^2  0        ;
          0         0         0        1/2^2 ];

     
R_pos = [ 1/0.05^2  0;
          0          1/0.05^2];

K_pos = lqr(sys_hor, Q_pos, R_pos);
disp(K_pos)