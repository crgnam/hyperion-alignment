function [q] = rotmat_to_quat(R)
    % This method allows converting a rotation matrix into a
    % quaternion of the shuster convention.

    R1 = R(1,1);
    R2 = R(2,2);
    R3 = R(3,3);
    q = zeros(4,1);

    q(4) = sqrt(.25*(1+trace(R)));
    q(1) = sqrt(.25*(1+R1-R2-R3));
    q(2) = sqrt(.25*(1-R1+R2-R3));
    q(3) = sqrt(.25*(1-R1-R2+R3));

    qmax = max(q(q==real(q)));

    if qmax == q(1)
        q(1) = q(1);
        q(2) = (R(1,2)+R(2,1))/(4*q(1));
        q(3) = (R(3,1)+R(1,3))/(4*q(1));
        q(4) = (R(2,3)-R(3,2))/(4*q(1));

    elseif qmax == q(2)
        q(1) = (R(1,2)+R(2,1))/(4*q(2));
        q(2) = q(2);
        q(3) = (R(2,3)+R(3,2))/(4*q(2));
        q(4) = (R(3,1)-R(1,3))/(4*q(2));

    elseif qmax == q(3)
        q(1) = (R(3,1)+R(1,3))/(4*q(3));
        q(2) = (R(2,3)+R(3,2))/(4*q(3));
        q(3) = q(3);
        q(4) = (R(1,2)-R(2,1))/(4*q(3));

    elseif qmax == q(4)
        q(1) = (R(2,3)-R(3,2))/(4*q(4));
        q(2) = (R(3,1)-R(1,3))/(4*q(4));
        q(3) = (R(1,2)-R(2,1))/(4*q(4));
        q(4) = q(4);
    end
end