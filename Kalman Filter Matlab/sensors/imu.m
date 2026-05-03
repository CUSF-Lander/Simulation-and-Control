function y = imu(state, inputs)
    %  Extract States 
    % Position
    x  = state(1);  x2  = state(2);  x3  = state(3);  
    % Euler Angles (Roll, Pitch, Yaw)
    p = state(4); q = state(5); u = state(6); 
    % Velocity
    vx = state(7);  vy = state(8);  vz = state(9);  
    % Angular Rates (p, q, r)
    wx = state(10); wy = state(11); wz = state(12);
    
    % Extract control inputs
    a1 = inputs(1); a2 = inputs(2); wt1 = inputs(3); wt2 = inputs(4);
    
    % Transformation matrix from body angular velocity to Tait-Bryan Rates

    W = [ 1, 0,      -sin(q)         ;
          0, cos(p),  cos(q)*sin(p)  ;
          0, -sin(p), cos(q)*cos(p) ];
    
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
            
    % State vectors used for derivation
    nw = [p q u].';     % Attitude (world frame)
    wb = [wx wy wz].';  % Angular velocity (body frame)
       
    %% Translational dynamics
    
    vb_dot = 1/m * ( fb );% -  R.' * [0 0 m*g].'); %IMUs don't measure gravity

    %% Acceleration Vectors
    y = zeros(9,1);
    y = [nw; % angular position
        vb_dot; % linear acceleration in body frame
        wb]; % angular velcity
end