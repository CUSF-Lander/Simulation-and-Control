function dx = drone_dynamics(state, inputs, params)

    %  Extract States 
    % Position
    x  = state(1);  y  = state(2);  z  = state(3);  
    % Euler Angles (Roll, Pitch, Yaw)
    p = state(4); q = state(5); u = state(6); 
    % Velocity
    vx = state(7);  vy = state(8);  vz = state(9);  
    % Angular Rates (p, q, r)
    wx = state(10); wy = state(11); wz = state(12);
    
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
    nw = [p q u].';     % Attitude (world frame)
    wb = [wx wy wz].';  % Angular velocity (body frame)
    pw = [x y z].';     % Position (world frame)
    vb = [vx vy vz].';  % Velocity (body frame)    

    
    %% Rotational dynamics
    
    nw_dot = Winv * wb;
    wb_dot = J \ (tb - cross(wb, J * wb)  );
    
    %% Translational dynamics
    
    pw_dot = R * vb;
    vb_dot = 1/m * ( fb -  R.' * [0 0 m*g].');
    
    vw_dot = R * vb_dot;
    %% Combined non-linear model
    
    dx = [ pw_dot;
            nw_dot;
            vw_dot;
            wb_dot ];

end
