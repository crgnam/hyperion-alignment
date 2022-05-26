function [roll, pitch, yaw] = quat_to_eulerangles(quats)
    % q = q0 + q1*i + q2*j + q3*k
    q0 = quats(4,:);
    q1 = quats(1,:);
    q2 = quats(2,:);
    q3 = quats(3,:);
    
    % roll (x-axis rotation)
    sinr_cosp = 2.0 * (q0 .* q1 + q2 .* q3);
    cosr_cosp = 1.0 - 2.0 * (q1 .* q1 + q2 .* q2);
    roll = atan2(sinr_cosp, cosr_cosp);
    
    % pitch (y-axis rotation)
    sinp = 2.0 * (q0 .* q2 - q3 .* q1);
    if (abs(sinp) >= 1)
      pitch = pi/2*sign(sinp); % use 90 degrees if out of range
    else
      pitch = asin(sinp);
    end
    
    % yaw (z-axis rotation)
    siny_cosp = 2.0 * (q0 .* q3 + q1 .* q2);
    cosy_cosp = 1.0 - 2.0 * (q2 .* q2 + q3 .* q3);  
    yaw = atan2(siny_cosp, cosy_cosp);
end