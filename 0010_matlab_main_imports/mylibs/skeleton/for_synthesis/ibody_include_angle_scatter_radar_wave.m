function ibody = ibody_include_angle_scatter_radar_wave(ibody, rdr, rdr_wave_vect, adcOn_mask)
    % dim(xyz, Ntime, antenna, Scatter);
    Nscattr = size(rdr_wave_vect, 4);
    Nt_masked = size(rdr_wave_vect, 2);
    angle_scatter_wave_rad = nan(1,size(adcOn_mask,2),1,size(rdr_wave_vect,4));

    for a_idx = 1:Nscattr
        % axis of symetry is always y:
        sym_v_elip = [0 1 0 1].';
        sym_v_g = zeros(3, Nt_masked);

        % batch multiplication: G*v
        for i=1:3
            G = squeeze(ibody.Ggcorr_elps(:,:,a_idx,adcOn_mask));
            sym_v_g(i,:) =  sum(permute(G(i,:,:),[2,1,3]).* sym_v_elip,1);
        end
        
        % get angles:
        wave_a = rdr_wave_vect(:,:,:,a_idx);
        angle_scatter_wave_rad(1,adcOn_mask,1,a_idx) = atan2(vecnorm(cross(wave_a,sym_v_g,1),2,1), dot(wave_a,sym_v_g,1));
        '';
    end   
    
    % save:
    ibody.angle_scatter_wave_rad = angle_scatter_wave_rad;
end