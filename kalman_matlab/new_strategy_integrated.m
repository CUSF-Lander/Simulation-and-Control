%clc
%clear


syms x y z vx vy vz phi theta psi wx wy wz  % states
syms F1 F2 F3 Mz  % inputs
syms m Jx Jy Jz c r g  % parameters

%% Definitions 
% --- 1. Rotation Matrix R (Body to World: Z-Y-X Convention) ---
% R = Rz(psi) * Ry(theta) * Rx(phi)
Rx = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
Ry = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
Rz = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
R = Rz * Ry * Rx;

% --- 2. Euler Rate Propagation Matrix W ---
% Relates body rates [p; q; r] to Euler rates [phi_dot; theta_dot; psi_dot]
W = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
     0, cos(phi),           -sin(phi);
     0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

% --- 3. Dynamics Setup ---
J = diag([Jx, Jy, Jz]);

% state vectors used for derivation
p = [x y z].'; % Position (world frame)
v = [vx vy vz].'; % velocity (world frame)
n = [phi theta psi].'; % attitude (world frame)
wb = [wx wy wz].'; % angular velocity (body frame)


% Total state vector
X = [p; v; n; wb];
X_red = [p(3); v(3); n; wb]; % Reduced state vector (only attitude and altitude)
X_hor = [p(1); p(2); v(1); v(2)]; % reduced state for horizontal movements

U = [F1; F2; F3; Mz];

% Force from motor 1 
force1 = [F1; F2; F3];

moment = [0; 0; Mz];

% proportion scaling Force2 and force1
lambda = 1 - (Mz / (c * F3));

% total force
total_force_motor = (1 + lambda) * force1;

% CoM to pivot
r_og = [0; 0; -r];

% total torque
total_moment = cross(r_og, force1) + moment;


%% Equations of motion
% Translational dynamics
p_dot = v;
v_dot = (R * total_force_motor) / m - ([0 0 g].');

% Rotational Dynamics
n_dot = W * wb;
wb_dot = J \ (total_moment - cross(wb, J*wb));

%% Combined non-linear model 
f = [p_dot;
     v_dot;
     n_dot;
     wb_dot];

disp(f)

f_red = [p_dot(3);
         v_dot(3);
         n_dot;
         wb_dot];

f_hor = [p_dot(1);
         p_dot(2);
         v_dot(1);
         v_dot(2)];
U_hor = [phi; theta];

%% Linearisation

% Jacobian method
A = jacobian(f, X);
B = jacobian(f, U);

% Reduced model
A2 = jacobian(f_red, X_red);
B2 = jacobian(f_red, U);

% Horizontal model 
A3 = jacobian(f_hor, X_hor);
B3 = jacobian(f_hor, U_hor);

%% Substitution 
% Parameters
m = params.m;
Jx = params.Jx; 
Jy = params.Jy;
Jz = params.Jz; 
g = params.g; 
c = params.Km/params.Kt;%0.05; %constant Tm = c * F
r = params.a + params.b;%0.1;

params_control = [m; Jx; Jy; Jz; c; r; g];

x = 0; y= 0; z = 0.0; vx = 0; vy = 0; vz = 0.0; phi = 0; theta = 0; psi = 0; wx = 0; wy = 0; wz = 0; %note qw = 1 when hovering


% initial state
X0 = [0;0;3; 0;0;0; 0;0;0; 0;0;0];
F1 = 0; F2 = 0; F3 = 9.81; Mz = 0; 

A_numeric = subs(A);
disp(A_numeric)

B_numeric = subs(B);
disp(B_numeric)

A_sys = double(vpa(A_numeric, 4));
B_sys = double(vpa(B_numeric, 4));
C_sys = eye(12);
D_sys = zeros(12, 4);

sys_ss = ss(A_sys, B_sys, C_sys, D_sys);

Co = ctrb(A_sys, B_sys);

disp(rank(Co))


% reduced model 
A2_numeric = subs(A2);

B2_numeric = subs(B2);

A2_sys = double(vpa(A2_numeric, 4));
B2_sys = double(vpa(B2_numeric, 4));
C2_sys = eye(8);
D2_sys = zeros(8, 4);

% integral action
G_hov = [1 0 0 0 0 0 0 0];

A2_int = [A2_sys,   zeros(8,1); ...
          G_hov,   0          ];
B2_int = [B2_sys; zeros(1,4)];
C2_int = eye(9);
D2_int = zeros(9,4);


sys_red_ss = ss(A2_sys, B2_sys, C2_sys, D2_sys);

sys_red_int = ss(A2_int, B2_int, C2_int, D2_int);


Q = diag([30.0,10.0, 300,300, 10.0, 10.0, 10.0, 1.0, 30.0]);
R = diag([1.0, 1.0, 0.6, 2500]);

K_att = lqr(sys_red_int, Q, R);
disp(K_att)

A3_numeric = subs(A3);
B3_numeric = subs(B3);

A3_sys = double(vpa(A3_numeric, 4));
B3_sys = double(vpa(B3_numeric, 4));
C3_sys = eye(4);
D3_sys = zeros(4, 2);
disp(A3)

G_pos = [1 0 0 0;
         0 1 0 0];
A3_int = [ A3_sys zeros(4,2);
           G_pos zeros(2,2)];
B3_int = [B3_sys; zeros(2,2)];
C3_int = eye(6);
D3_int = zeros(6,2);



Q_hor = diag([1e7, 1e7, 1e4, 1e4, 30.0, 30.0]);
R_hor = diag([1e9, 1e9]);

sys_hor_ss = ss(A3_sys, B3_sys, C3_sys, D3_sys);
sys_hor_int = ss(A3_int, B3_int, C3_int, D3_int);

K_hor = lqr(sys_hor_int, Q_hor, R_hor);
disp(K_hor)


