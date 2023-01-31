function [elip_skel] = batch_ellipsoid_gen(abc, Ggcorr_elps)
    elip_skel = double.empty(4,0);
    
    for sc_i = 1:size(Ggcorr_elps,3)
        [xyz_el_gl, ~] = draw_ellipsoid(abc(:,sc_i), Ggcorr_elps(:,:,sc_i));
        elip_skel = cat(2,elip_skel,xyz_el_gl);
    end
end