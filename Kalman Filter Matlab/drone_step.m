function state_next = drone_step(state, inputs)
    dt = 0.01; %This has to match the EKF's frequency

    dx_cts = drone_dynamics(state, inputs);

    state_next = state + dx_cts * dt;

end