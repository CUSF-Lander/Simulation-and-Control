function next_state = expanded_drone_dynamics(state, inputs, params)
    dt = 0.01;
    
    %% POSITION COMPONENT OF STATE VECTOR IS IN WORLD FRAME 
    %%

    % Extract States 
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
    a1 = inputs(1); a2 = inputs(2); wt1 = inputs(3); wt2 = inputs(4); % angles, rotation rates

    % Drone Constants
    m  = params.m;
    g  = params.g;
    Kt = params.Kt;
    Km = params.Km;
    Jx = params.Jx;
    Jy = params.Jy;
    Jz = params.Jz;
    a = params.a;
    b = params.b;  
    
    % Transformation matrix from body angular velocity to Tait-Bryan Rates

    W = [ 1, 0,      -sin(q)         ;
          0, cos(p),  cos(q)*sin(p)  ;
          0, -sin(p), cos(q)*cos(p) ];
    
    Winv = [ 1,  sin(p)*tan(q),  cos(p)*tan(q);
             0,  cos(p),         -sin(p);
             0,  sin(p)/cos(q),  cos(p)/cos(q) ];
    
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
    pw = [x y z].';     % Position (world frame)
    vw = [vx vy vz].';  % Velocity (body frame)   % NEED TO SWITCH TO WORLD FRAME
    nw = [p q u].';     % Attitude (world frame)
    wb = [wx wy wz].';  % Angular velocity (body frame) 

    
    %% Rotational dynamics
    
    wb_dot = J \ (tb - cross(wb, J * wb)  );
    
    %% Translational dynamics
    
    vb_dot = 1/m * ( fb -  R.' * [0 0 m*g].');
    vw_dot = R * vb_dot;

    %% Combined non-linear model

    A = [1 0 0 0 0 0 dt 0 0 0 0 0 0.5*(dt^2) 0 0 0 0 0;
         0 1 0 0 0 0 0 dt 0 0 0 0 0 0.5*(dt^2) 0 0 0 0;
         0 0 1 0 0 0 0 0 dt 0 0 0 0 0 0.5*(dt^2) 0 0 0;
         0 0 0 1 0 0 0 0 0 dt 0 0 0 0 0 0.5*(dt^2) 0 0;
         0 0 0 0 1 0 0 0 0 0 dt 0 0 0 0 0 0.5*(dt^2) 0;
         0 0 0 0 0 1 0 0 0 0 0 dt 0 0 0 0 0 0.5*(dt^2);

         0 0 0 0 0 0 1 0 0 0 0 0 dt 0 0 0 0 0;
         0 0 0 0 0 0 0 1 0 0 0 0 0 dt 0 0 0 0;
         0 0 0 0 0 0 0 0 1 0 0 0 0 0 dt 0 0 0;
         0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 dt 0 0;
         0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 dt 0;
         0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 dt;

         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        ];
    
    % 1. Use a simpler A matrix that only handles Pos and Vel integration
    A_kinematic = [eye(6), eye(6)*dt; 
                   zeros(6,6), eye(6)]; 
       
    % 3. Build the NEXT state
    % States 1-12 come from Kinematics (moving based on current Vel/Acc)
    % States 13-18 are DIRECTLY replaced by the new physics
    % next_state = [ A_kinematic * state(1:12); % Updates Pos and Vel
    %                vw_dot;                    % NEW Accel Linear
    %                wb_dot                     % NEW Accel Angular
    %              ];

    next_state = A*state + [0;0;0;0;0;0;0;0;0;0;0;0;vw_dot;wb_dot];
end
