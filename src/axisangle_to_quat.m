function [quats] = axisangle_to_quat(axis,angle)
    qx = axis(1,:).*sin(angle/2);
    qy = axis(2,:).*sin(angle/2);
    qz = axis(3,:).*sin(angle/2);
    qw = cos(angle/2);
    
    quats = [qx; qy; qz; qw];
end