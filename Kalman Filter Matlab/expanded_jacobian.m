%% Linearization

syms x x2 x3 p q u vx vy vz wx wy wz ax ay az ix iy iz
syms a1 a2 wt1 wt2 

state_sym = [x; x2; x3; p; q; u; vx; vy; vz; wx; wy; wz; ax; ay; az; ix; iy; iz];
input_sym = [a1; a2; wt1; wt2];

%% 2. Call your functions with Symbols
z_imu_sym = expanded_imu(state_sym, input_sym, params);
dx_sym = expanded_drone_dynamics(state_sym, input_sym, params); %expanded_drone_dynamics is a discretized function

%% 3. Derive the Jacobians
A_sym = jacobian(dx_sym, state_sym);
B_sym = jacobian(dx_sym, input_sym);
H_imu_sym = jacobian(z_imu_sym, state_sym);

matlabFunction(A_sym, B_sym, H_imu_sym, 'File', 'calc_Jacobians', 'Vars', {state_sym, input_sym});
% equilibrium point
x_eq = zeros(18,1);

% Calculate motor speed (wt) needed to hover: Ft = Kt*(wt1^2 + wt2^2) = m*g
u_eq = [0; 0; stationary_wt; stationary_wt]; 
%u_eq = [0; 0; 0; 0]; 

% Convert to Numerical Matrices (All discretized already)
A_numeric = double(subs(A_sym, [state_sym; input_sym], [x_eq; u_eq]));
B_numeric = double(subs(B_sym, [state_sym; input_sym], [x_eq; u_eq]));
C_numeric = double(subs(H_imu_sym, [state_sym; input_sym], [x_eq; u_eq]));
