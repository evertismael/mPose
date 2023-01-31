function [xyz_el_gl, axs_el_gl] = draw_ellipsoid(abc, Gg_elps)
a = abc(1);
b = abc(2);
c = abc(3);

% make tht and lmd ranges:
tht_rng = 0:pi/10:2*pi;
lmd_rng = 0:pi/10:2*pi;
tht = [zeros(size(lmd_rng)),NaN,tht_rng,NaN,tht_rng];
lmd = [lmd_rng,NaN, zeros(size(tht_rng)),NaN, (pi/2)*ones(size(tht_rng))];

% in the ellipsoid coordinate system:
x_el = a*cos(tht).*cos(lmd);
y_el = b*cos(tht).*sin(lmd);
z_el = c*sin(tht);
xyz_el = cat(1,x_el,y_el,z_el, ones(size(z_el)));
xyz_el = cat(2,xyz_el,nan(4,1));


axs_el = [-a a NaN  0 0 NaN  0 0 NaN;
           0 0 NaN -b b NaN  0 0 NaN;
           0 0 NaN  0 0 NaN -c c NaN;
           1 1 NaN  1 1 NaN  1 1 NaN];


% transforming to global coords:
xyz_el_gl = zeros(size(xyz_el));
xyz_el_gl(1,:) = sum((Gg_elps(1,:).').*xyz_el,1);
xyz_el_gl(2,:) = sum((Gg_elps(2,:).').*xyz_el,1);
xyz_el_gl(3,:) = sum((Gg_elps(3,:).').*xyz_el,1);
xyz_el_gl(4,:) = sum((Gg_elps(4,:).').*xyz_el,1);


axs_el_gl = zeros(size(axs_el));
axs_el_gl(1,:) = sum((Gg_elps(1,:).').*axs_el,1);
axs_el_gl(2,:) = sum((Gg_elps(2,:).').*axs_el,1);
axs_el_gl(3,:) = sum((Gg_elps(3,:).').*axs_el,1);
axs_el_gl(4,:) = sum((Gg_elps(4,:).').*axs_el,1);





end

