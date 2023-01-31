% IF_SIGNAL_MEASUREMENTS 
%   It computes IF-signals in FMCW.
%   It uses the bone-centers as scatterers
%   Notice: The more Rx antennas, the more time it takes.

function [jntsm, if_sig] = synthetic_if_signal(mbody, rdr, opts)
    glbp = glb_prms();
    
    % --------------------------------------------------------------------
    % Total simulation time and grid:
    % -------------------------------------------------------------------
    Nt_max = mbody.t_grid(end);
    t_sim_end = min([glbp.t_sim, Nt_max]);
    fprintf('chosen simulation time: %.2f  seconds \n (if not selected check: 0010/Config)\n', t_sim_end);
    t_sim_grid = 0:rdr.rf.dt:t_sim_end;
    
    % --------------------------------------------------------------------
    % Precompute params: 
    %       - Number of frames to simulate:
    %       - Init storage matrices: if_sig, jntsm.
    % --------------------------------------------------------------------
    N_frame_rate = rdr.frame.rate;
    N_frames = floor(t_sim_end/N_frame_rate); 
    
    % initialize
    if_sig = double.empty(1,0, rdr.rf.Nrx);
    jntsm = double.empty(3,mbody.Njnt,0);
    
    tic
    Nchar_line = fprintf('processing frame %d of %d', 0, N_frames);
    for fr_idx = 1:N_frames
        fprintf(repmat('\b',1,Nchar_line));
        Nchar_line = fprintf('processing frame %d of %d', fr_idx, N_frames);
        
        % -----------------------------------------------------------------
        % create multipath bodies
        % -----------------------------------------------------------------
        mbodies{1} = mbody;
        if opts.first_order_multipaths
            m1_mbody = create_first_order_multipath_mbody(mbody, fr_idxs, t_frame);
            mbodies{2} = m1_mbody;
        end
        if opts.second_order_multipaths
            m2_mbody = create_second_order_multipath_mbody(mbody, fr_idxs, t_frame);
            mbodies{3} = m2_mbody;
        end


        % -----------------------------------------------------------------
        % Interpolate data(channels, jnts, bones, etc) in the frame
        % -----------------------------------------------------------------
        t_frame = t_sim_grid( (1:rdr.frame.Nsamp_active) + (fr_idx-1)*rdr.frame.Nsamp_passive_and_active);
        fr_mask = (mbody.t_grid >= t_frame(1) - 1/120) & (mbody.t_grid <= t_frame(end) + 4/120);
        fr_idxs = find(fr_mask==1);
        
        ibodies = repmat({},1,size(mbodies,2));
        for b_idx = 1:size(mbodies,2)
            [ibody, ref_body] = create_interpolated_mbody(mbodies{b_idx}, fr_idxs, t_frame);
            ibodies{b_idx} = ibody;
% %             % test: to see we interpolated correctly.
% %             figure
% %             plot(mbody.t_grid(fr_mask).',squeeze(ref_body.Ggcorr_elps(:,1,1,:)).','o',...
% %                 t_frame.',squeeze(ibody.Ggcorr_elps(:,1,1,:)).',':.');
        end
        
        % -----------------------------------------------------------------
        % Generate the IF-signal of frame:
        % -----------------------------------------------------------------
        [if_sig_frame, jnts_frame]= IF_signals(t_frame, ibodies, rdr, opts);
        if_sig = cat(2,if_sig,if_sig_frame);
        
        % save joints and bones for frame:
        jntsm = 0; %cat(3,jntsm,squeeze(mean(tmp_jnts_pm,3)));
    end
    fprintf('\n')
    toc
    '';
end