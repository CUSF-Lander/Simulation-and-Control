%% Linearization

syms x x2 x3 p q u vx vy vz wx wy wz ax ay az ix iy iz
syms a1 a2 wt1 wt2 

state_sym = [x; x2; x3; p; q; u; vx; vy; vz; wx; wy; wz; ax; ay; az; ix; iy; iz];
input_sym = [a1; a2; wt1; wt2];

%% Call functions with symbols
z_imu_sym = expanded_imu(state_sym, input_sym, params);
dx_sym = expanded_drone_dynamics(state_sym, input_sym, params); %expanded_drone_dynamics is a discretized function

%% Derive the symbolic Jacobians
A_sym = jacobian(dx_sym, state_sym);        % State transition matrix
B_sym = jacobian(dx_sym, input_sym);        % Control input matrix
H_imu_sym = jacobian(z_imu_sym, state_sym); % Measurement matrix

% Save the symbolic matrices to a file
matlabFunction(A_sym, B_sym, H_imu_sym, 'File', 'get_Jacobians', 'Vars', {state_sym, input_sym});

%% Calculate the numerical Jacobians (for the Kalman gain calculations)

% Convert to numerical matrices at the equilbrium point (All discretized already)
A_numeric = double(subs(A_sym, [state_sym; input_sym], [x_eq; u_eq]));
B_numeric = double(subs(B_sym, [state_sym; input_sym], [x_eq; u_eq]));
H_numeric = double(subs(H_imu_sym, [state_sym; input_sym], [x_eq; u_eq]));
