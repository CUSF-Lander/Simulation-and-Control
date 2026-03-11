function y = expanded_imu(state, inputs, params)
    %Extract States 
    % Position
    x  = state(1);  y  = state(2);  z  = state(3);  
    % Euler Angles (Roll, Pitch, Yaw)
    p = state(4); q = state(5); u = state(6); 
    % Velocity
    vx = state(7);  vy = state(8);  vz = state(9);  
    % Angular Rates (p, q, r)
    wx = state(10); wy = state(11); wz = state(12);
    %Linear Accelerations
    ax = state(13); ay = state(14); az = state(15);
    %Angular Accelerations
    ix = state(16); iy = state(17); iz = state(18);
    
    % Extract control inputs
    a1 = inputs(1); a2 = inputs(2); wt1 = inputs(3); wt2 = inputs(4);
    
    %Drone constants
    m = params.m;
    g = params.g;

    % Rotation matrix from body to world frame, input: roll, pitch, yaw
    R = [cos(q)*cos(u), sin(p)*sin(q)*cos(u)-cos(p)*sin(u), cos(p)*sin(q)*cos(u)+sin(p)*sin(u) ;
         cos(q)*sin(u), sin(p)*sin(q)*sin(u)+cos(p)*cos(u), cos(p)*sin(q)*sin(u)-sin(p)*cos(u) ;
         -sin(q),       sin(p)*cos(q),                      cos(p)*cos(q)                     ];


    % State vectors used for derivation
    nw = [p q u].';     % Attitude (world frame)
    wb = [wx wy wz].';  % Angular velocity (body frame)
    vw_dot = [ax; ay; az];%+ [0; 0; g]; %IMUs don't read gravity
    vb_dot = R.' * vw_dot; 

    %% Acceleration Vectors
    y = zeros(9,1);
    y = [nw; % angular position
        vb_dot; % linear acceleration in body frame
        wb]; % angular velcity
end