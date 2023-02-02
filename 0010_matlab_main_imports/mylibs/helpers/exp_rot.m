function R = exp_rot(theta_deg, w)
    w_wedge = [0   -w(3) w(2); 
               w(3) 0   -w(1);
              -w(2) w(1) 0];
    theta_rad = deg2rad(theta_deg);
    R = eye(3,3) + w_wedge*sin(theta_rad) + w_wedge*w_wedge*(1-cos(theta_rad));
end