function x_new = no_filter(x, GPS, IMU, barometer, t)
    
    dt = 0.01;
    x_new = x;
    % Calculates rotation matrix from body to world frame, input: roll, pitch, yaw
    p = x(4); q = x(5); u = x(6);  
    R = [cos(q)*cos(u), sin(p)*sin(q)*cos(u)-cos(p)*sin(u), cos(p)*sin(q)*cos(u)+sin(p)*sin(u) ;
        cos(q)*sin(u), sin(p)*sin(q)*sin(u)+cos(p)*cos(u), cos(p)*sin(q)*sin(u)-sin(p)*cos(u) ;
        -sin(q),       sin(p)*cos(q),                      cos(p)*cos(q)                     ];
    
    
    % Update angular state from measurements
    x_new(4:6) = IMU(1:3);
    x_new(10:12) = IMU(7:9);
    
    % Integrate acceleration measurements
    acc_measure = R * IMU(4:6);
    g = 9.807;
    acc_measure(3) = acc_measure(3) - g;
    delta_v = dt * acc_measure;
    delta_x = 0.5 * (dt^2) * acc_measure;
    
    
    % Update linear state using IMU measurements
    x_new(7:9) = x(7:9) + delta_v;
    x_new(1:3) = x(1:3) + x_new(7:9) * dt + delta_x;
    fprintf("\nMeasured x accs = %f", acc_measure(1));
    fprintf("\nMeasured y acc = %f", acc_measure(2));
    fprintf("\nMeasured z acc = %f", acc_measure(3));
    fprintf("\nestimated z = %f", x_new(3));

    % Take average for position measurements
    %x_internal(1:2) = GPS(1:2);
    %x_internal(3) = 0.5 * (GPS(3) + barometer);

end