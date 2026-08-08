function [x_estimate, last_gps_val] = kalman_filter(x_estimate, inputs, last_gps_val, z_imu, z_barometer, z_gps, kf_20, kf_100, params, H_barometer, H_gps)
    coder.extrinsic('fprintf');

    %Check if there is a new gps value
    gps_delta = last_gps_val - z_gps;
    if any(gps_delta)
        fprintf('');
        %fprintf('Inputs are %f',inputs(3));
        %new gps data
        gps_present = true;
        last_gps_val = z_gps;
    else
        % No new gps data
        gps_present = false;
    end
    
    %retrieves A, B, and H_imu matrices
    [A, B, H_imu] = get_Jacobians(x_estimate, inputs);

    % Calculates rotation matrix from body to world frame, input: roll, pitch, yaw
    p = x_estimate(4); q = x_estimate(5); u = x_estimate(6);     
    R = [cos(q)*cos(u), sin(p)*sin(q)*cos(u)-cos(p)*sin(u), cos(p)*sin(q)*cos(u)+sin(p)*sin(u) ;
         cos(q)*sin(u), sin(p)*sin(q)*sin(u)+cos(p)*cos(u), cos(p)*sin(q)*sin(u)-sin(p)*cos(u) ;
         -sin(q),       sin(p)*cos(q),                      cos(p)*cos(q)                     ];
    
    % Prediction update
    x_pred = A * x_estimate + 0.5 * B * inputs;
    fprintf('\n\nPredicted z acceleration  = %f', x_pred(15));

    % Convert imu measurements from body to world frame
    zw_acc = R * z_imu(4:6);
    z_imu = [z_imu(1:3);zw_acc;z_imu(7:9)];
        
    % Define measurement matrix and vector depending on if gps has updated
    if gps_present 
        H = [H_imu; H_barometer; H_gps];
        z = [z_imu; z_barometer; z_gps];
        kf = kf_20;
    else
        H = [H_imu; H_barometer];
        z = [z_imu; z_barometer];
        kf = kf_100;
    end

    % Innovation calculation
    innov = z - H * x_pred;

    % Measurement Update
    g_correction = zeros(18,1);
    g_correction(15) = -params.g; % to activate landed mode, set this to zerof

    x_estimate = x_pred + kf * (innov) + g_correction;

    fprintf('\nEstimated z acceleration  = %f', x_estimate(15));
end

 