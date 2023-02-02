function path = get_los_or_som_path_information(sctrs, rdr, mode, h)
    % ---------------------------------------------------------------------
    % Select Correction Transformation matrix:
    % It creates the image scatterers on the other side of the
    % floor/ceiling
    % ---------------------------------------------------------------------
    if strcmp(mode, 'los')
        G = eye(4,4); % no correction for LOS.
    elseif strcmp(mode, 'ceiling')
        G = get_G_ceiling_floor(h,'ceiling');
    elseif strcmp(mode, 'floor')
        G = get_G_ceiling_floor(h,'floor');
    else
        error('mode not accepted: los/ceiling/floor only');
    end
    
    % ---------------------------------------------------------------------
    % Apply Transformation matrix correction:
    % ---------------------------------------------------------------------
    % move position and velocity:
    tmp_sz = size(sctrs.pos);
    hom_sctrs_pos = cat(1,sctrs.pos,ones(1,tmp_sz(2),tmp_sz(3)));
    pos = batch_mul(G, hom_sctrs_pos);
    pos = pos(1:3,:,:);
    vel = batch_mul(G(1:3,1:3), sctrs.vel);
    
    % correct Gg_elp, Gg_skel:
    Gg_elp = batch_mul_gral(G, sctrs.Gg_elps);
    Gg_skel = batch_mul_gral(G, sctrs.Gg_skel);
    
    % ---------------------------------------------------------------------
    % Get measurements in RADAR coords, to the image scatterers:
    % range, azimuth, elevation, tht_path, a.
    % ---------------------------------------------------------------------

    % express pos into rdr coordinates:
    pos_uvw = rdr.glb2rdr_coords(pos, 'not-homogen');
    pos_uvw = permute(pos_uvw, [1,4,2,3]);
    tmp_tx_sctr = (pos_uvw - rdr.rf.tx_uvw(1:3,:));
    tmp_sctr_rx = (pos_uvw - rdr.rf.rx_uvw(1:3,:));
    
    % distances are coordinate invariante:
    path.tx_sctr = permute(sqrt(sum(tmp_tx_sctr.^2,1)),[2,3,4,1]);
    path.sctr_rx = permute(sqrt(sum(tmp_sctr_rx.^2,1)),[2,3,4,1]); 
    % by default range is computed using tx1, and rx1.
    path.rm = permute(0.5*(path.tx_sctr(1,:,:) + path.sctr_rx(1,:,:)),[2,3,1]); 
    
    % azimuth elevation:
    pos_uvw = permute(pos_uvw, [1,3,4,2]);
    path.azm = permute(atan2(pos_uvw(1,:,:),pos_uvw(2,:,:)),[2,3,1]); 
    path.elm = permute(asin(pos_uvw(3,:,:)./sqrt(sum((pos_uvw).^2,1)) ),[2,3,1]); 
    
    % radial velocity: default using tx1
    u_tx_sctr = tmp_tx_sctr(:,1,:,:)./sqrt(sum((tmp_tx_sctr(:,1,:,:)).^2,1));
    u_tx_sctr = permute(u_tx_sctr, [1,3,4,2]);
    % express vel in radar coords:
    vel_uvw = rdr.vel_glb2rdr_coords(vel);
    path.vm = permute(-sum(vel_uvw.*u_tx_sctr,1),[2,3,1]); 
    
    % ---------------------------------------------------------------------
    % Compute incidence angle: radar_wave and scatter.
    % ---------------------------------------------------------------------
    % wave_glb: dim(xyz, Ntx, Nsctr, Nfr)
    wave_glb = permute(pos,[1,4,2,3]) - rdr.rf.tx_glb(1:3,:); % vector along radial direction
    vy_glb = batch_mat_single_vect_product(Gg_elp,  [0 1 0 1].'); % get axis y
    vo_glb = batch_mat_single_vect_product(Gg_elp,  [0 0 0 1].'); % get origin
    v_glb = vy_glb - vo_glb; % vector along the bone.
    % v_g: dim(xyz, 1, Nsctr, Nfr)
    v_glb = permute(v_glb(1:3,:,:),[1,4,2,3]);
    v_glb = repmat(v_glb,1,3,1,1);
    tht_wave_sctr = atan2(vecnorm(cross(wave_glb,v_glb,1),2,1), dot(wave_glb,v_glb,1));
    tht_wave_sctr = permute(tht_wave_sctr,[2,3,4,1]); % dim(Ntx, Nsctr, Nfr);
    path.tht_wave_sctr = tht_wave_sctr;
    
    % TODO: compute reflextion coeficient:
    n1 = 1.0003; % air
    n2 = 2 + 1j; % bones:
    fresnel_coef = (n1*cos(tht_wave_sctr) - n2*cos(tht_wave_sctr))./(n1*cos(tht_wave_sctr) + n2*cos(tht_wave_sctr));
    tmp_num = (pi/4)*sctrs.abc(1,:,:).^4.*sctrs.abc(2,:,:).^2;
    tmp_den = sctrs.abc(1,:,:).^2.*sin(tht_wave_sctr).^2 + 0.25*sctrs.abc(2,:,:).^2.*cos(tht_wave_sctr).^2;
    sigma = sqrt(fresnel_coef).*tmp_num./tmp_den;
    path.sqrt_sigma = sqrt(sigma); % dim(Ntw, Nsctr, Nfr)

    % ---------------------------------------------------------------------
    % Compute gain_floor_ceiling
    % ---------------------------------------------------------------------
    wave_glb_xy = wave_glb;
    wave_glb_xy(3,:,:,:) = 0; % proyection on the xy-plane:
    tht_inc_comp = atan2(vecnorm(cross(wave_glb,wave_glb_xy,1),2,1), dot(wave_glb,wave_glb_xy,1));
    tht_inc_comp = permute(tht_inc_comp,[2,3,4,1]); % dim(Ntx, Nsctr, Nfr);
    tht_inc = pi/2 - tht_inc_comp;
    
    % TODO: compute reflextion coeficient:
    n1 = 1.0003; % air
    n2 = 2.55 - 1j*0.084; % concrete
    path.ref_coef = (n1*cos(tht_inc) - n2*cos(tht_inc))./(n1*cos(tht_inc) + n2*cos(tht_inc)); % dim(Ntw, Nsctr, Nfr)

    % ---------------------------------------------------------------------
    % last pieces of information:
    % ---------------------------------------------------------------------
    path.t_grid = sctrs.t_grid;
    path.pos = pos;
end