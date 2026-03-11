%% Linearization

syms x x2 x3 p q u vx vy vz wx wy wz 
syms a1 a2 wt1 wt2 

state_sym = [x; x2; x3; p; q; u; vx; vy; vz; wx; wy; wz];
input_sym = [a1; a2; wt1; wt2];

%% 2. Call your functions with Symbols
y_sym = imu(state_sym, input_sym);
dx_sym = drone_step(state_sym, input_sym); %drone_step is the discretized function

%% 3. Derive the Jacobians
A_sym = jacobian(dx_sym, state_sym);
B_sym = jacobian(dx_sym, input_sym);
H_sym = jacobian(y_sym, state_sym);

% equilibrium point
x_eq = zeros(12,1);
x_eq(5) = 0.05;

% Calculate motor speed (wt) needed to hover: Ft = Kt*(wt1^2 + wt2^2) = m*g
wt_hover = sqrt((0.75 * 9.807) / (2 * 0.2)); 
u_eq = [0; 0; wt_hover; wt_hover]; 
%u_eq = [0; 0; 0; 0]; 

% Convert to Numerical Matrices (All discretized already)
A_numeric = double(subs(A_sym, [state_sym; input_sym], [x_eq; u_eq]));
B_numeric = double(subs(B_sym, [state_sym; input_sym], [x_eq; u_eq]));
C_numeric = double(subs(H_sym, [state_sym; input_sym], [x_eq; u_eq]));
