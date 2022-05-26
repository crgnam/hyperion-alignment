function [rotmat] = axisangle_to_rotmat(u,t,use_deg)
    if nargin == 3
        assert(islogical(use_deg),'Input option for degrees must be a logical type')
        if use_deg
            t = deg2rad(t);
        end
    end
    x = u(1);
    y = u(2);
    z = u(3);
    c = cos(t);
    s = sin(t);
    rotmat = [ c+(x^2)*(1-c)   x*y*(1-c)-z*s   x*z*(1-c)+y*s;
              y*x*(1-c)+z*s    c+(y^2)*(1-c)   y*z*(1-c)-x*s;
              z*x*(1-c)-y*s    z*y*(1-c)+x*s   c+(z^2)*(1-c)];
end