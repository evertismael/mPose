function [rm, vm, azm, elm, jntsm, bnm_cntr, t_grid] = simple_measurements(mbody, rdr, flag_noise, fr_init, fr_end)
    % SIMPLE_RDR_CUBE 
    % It computes (range, velocity, angles) for all bone-centers
    % of the mbody. IT  DOES NOT SIMULATE FMCW SIGNALS
    % Note: it does not take into account the number of antennas.
    
    if ~exist('flag_noise','var')
        flag_noise = false;
    end
    
    % ---------------------------------------------------------------------
    % obtain body movement between frames:
    % ---------------------------------------------------------------------
    [jntsm, ~] = mbody.get_joints_by_frame(fr_init);
    [pm, ~, ~, ~] = mbody.get_bones_by_frame(fr_init);
    for fr_i = (fr_init+1): fr_end
        [tmp_jnts_xyz, ~] = mbody.get_joints_by_frame(fr_i);
        [tmp_pm, ~, ~, ~] = mbody.get_bones_by_frame(fr_i);
        pm = cat(3,pm,tmp_pm);
        jntsm = cat(3,jntsm,tmp_jnts_xyz);
        '';
    end
    pm = pm(1:3,:,:);
    bnm_cntr = pm;
    
    Nfrm = fr_end - fr_init + 1;
    Nscttr = size(pm,2);

    % ---------------------------------------------------------------------
    % Get measurements in RADAR coords: range, azimuth, elevation
    % ---------------------------------------------------------------------
    uvw = rdr.glb2rdr_coords(pm, 'not-homogen');
    rm = sqrt(sum((uvw).^2,1));
    azm = atan2(uvw(1,:,:),uvw(2,:,:));
    elm = asin(uvw(3,:,:)./rm);

    % ---------------------------------------------------------------------
    % Get radial velocity: interpolate and derivate
    % ---------------------------------------------------------------------
    t_grid = mbody.t_grid(fr_init: fr_end);
    bones_vm = zeros(3,Nscttr,Nfrm);  
    for sctr_idx = 1:Nscttr
        pm_i = pm(:,sctr_idx,:);
        ppf=spline(t_grid, pm_i);
        ppdf = ppf;
        ppdf.order=ppdf.order-1;
        ppdf.coefs=ppdf.coefs(:,1:end-1).*(ppdf.order:-1:1);
        
        vm_i = ppval(ppdf,t_grid);
        bones_vm(:,sctr_idx,:) = vm_i;
    end
    
    % radial velocity:
    all_n = (pm - rdr.p0)./rm; % normal vector in glb coords:
    vm = -sum(bones_vm.*all_n,1);
    
    % ---------------------------------------------------------------------
    % add noise:
    % ---------------------------------------------------------------------
    sigma = 0.001;
    if flag_noise
        rm = rm + randn(size(rm));
        azm = azm + sigma*randn(size(azm));
        elm = elm + sigma*randn(size(elm));
        vm = vm + sigma*randn(size(rm));
    end

    % queeze()
    rm = squeeze(rm);
    vm = squeeze(vm);
    azm = squeeze(azm);
    elm = squeeze(elm);
    '';
end

