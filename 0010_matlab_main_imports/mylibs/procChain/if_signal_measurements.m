function [rdm_out, jntsm_out, bnm_cntr_out, ifs_out, gs, range_prof_out] = if_signal_measurements(mbody, rdr)
    % IF_SIGNAL_MEASUREMENTS 
    % It computes IF-signals in FMCW.
    % It uses the bone-centers as scatterers
    % Notice: The more Rx antennas, the more time it takes.

    glbp = glb_prms();
    burst = rdr.burst;

    % total simulation time:
    Nt_max = mbody.t_grid(end);
    t_sim_end = min([glbp.t_sim, Nt_max]);
    fprintf('chosen simulation time: %.2f  seconds\n', t_sim_end);
    t_sim_grid = 0:burst.dt:t_sim_end;

    % precompute parameters constant in FRAME:
    % FRAME processing time = 1 RDms.
    Nsamp_frame = burst.Nsamp_frame;
    N_frames = fix(size(t_sim_grid,2)/Nsamp_frame); 
    
    % initialize
    frame_idx = 0;
    ifsig_all = double.empty(1,0,rdr.Nrx);

    % init: output labels for training:
    Njnts = size(mbody.jnts_xyz,3);
    Nbns = size(mbody.bones_pm,3);
    jntsm_all = double.empty(3,Njnts,0);
    bnm_cntr_all = double.empty(3,Nbns,0);
    tic
    Nchar_line = fprintf('processing frame %d of %d', 0, N_frames);
    while frame_idx < N_frames
        fprintf(repmat('\b',1,Nchar_line))
        Nchar_line = fprintf('processing frame %d of %d', frame_idx, N_frames);
        % frame-time:
        t_frame = t_sim_grid( (1:Nsamp_frame) + (frame_idx)*Nsamp_frame );
        
        % Interpolate trajectory for all targets:
        Nscttr = size(mbody.bones_pm,3);
        all_traj_frame = zeros(3,(burst.Nsamp_frame),Nscttr);
        
        t_mask_frame = (mbody.t_grid >= t_frame(1) - 1/120) & (mbody.t_grid <= t_frame(end) + 1/120);
        tmp_t_grid = mbody.t_grid(t_mask_frame);
        tmp_bnm_cntr = mbody.bones_pm(:,t_mask_frame,:);
        tmp_jnts_pm = mbody.jnts_xyz(:,t_mask_frame,:);
        for sctr_idx = 1:Nscttr
            %i_traj_cpi = interp1(mbody.t_grid.', mbody.bones_pm(:,:,sctr_idx).', t_frame.','spline');
            i_traj_cpi = interp1(tmp_t_grid.', tmp_bnm_cntr(:,:,sctr_idx).', t_frame.','spline');
            all_traj_frame(:,:,sctr_idx) = i_traj_cpi.';
            '';
        end

        % generate IF-signals:
        ifsig = IF_signals(t_frame, all_traj_frame, rdr);
        ifsig_all = cat(2,ifsig_all,ifsig);
        
        % save joints and bones for frame:
        jntsm_all = cat(3,jntsm_all,permute(mean(tmp_jnts_pm,2),[1,3,2]));
        bnm_cntr_all = cat(3,bnm_cntr_all,permute(mean(tmp_bnm_cntr,2),[1,3,2]));

        % next frame:
        frame_idx = frame_idx + 1;
    end
    fprintf('\n')
    toc
    
    % ---------------------------------------------------------------------
    % radar processing:
    rdr_cube = ifs_to_cube(ifsig_all, rdr);
    [tmp_rdms, tmp_rdm_grids, tmp_range_prof] = make_rdm(rdr_cube, rdr);
    

    bnm_cntr_out = bnm_cntr_all;
    jntsm_out = jntsm_all;

    range_prof_out = tmp_range_prof;
    rdm_out = tmp_rdms;
    ifs_out = ifsig_all;
    
    gs.rdm_grids = tmp_rdm_grids;

    % time grid of radar:
    gs.t_chirp_grid = (0:(size(tmp_range_prof,4)*size(tmp_range_prof,2)-1))*(burst.dt_frame/burst.Nsamp_doppler);

end
