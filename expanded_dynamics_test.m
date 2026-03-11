%% expanded dynamics function test

state = zeros(18,1);

input = zeros(4,1);
%input(1) = 0.1;
%input(3:4) = [10;10];

next_state = expanded_drone_dynamics(state, input, params);
disp(next_state);


next_state = expanded_drone_dynamics(next_state, input, params);
disp(next_state);

next_state = expanded_drone_dynamics(next_state, input, params);
disp(next_state);