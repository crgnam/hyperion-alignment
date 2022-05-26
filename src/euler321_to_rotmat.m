function [rotmat] = euler321_to_rotmat(rot1,rot2,rot3)
    T1 = [1       0            0;
          0   cos(rot1)   sin(rot1);
          0  -sin(rot1)   cos(rot1)];
    T2 = [cos(rot2)  0  -sin(rot2);
               0     1       0;
          sin(rot2)  0   cos(rot2)];
    T3 = [ cos(rot3)   sin(rot3)  0;
          -sin(rot3)   cos(rot3)  0;
               0           0      1];
    rotmat = T3*T2*T1;
end