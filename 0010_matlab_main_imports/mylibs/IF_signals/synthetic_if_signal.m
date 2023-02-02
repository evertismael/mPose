% IF_SIGNAL_MEASUREMENTS 
%   It computes IF-signals in FMCW.
%   It uses the bone-centers as scatterers
%   Notice: The more Rx antennas, the more time it takes.

function if_sig = synthetic_if_signal(los, fom, som, rdr, opts)
    % --------------------------------------------------------------------
    % Total simulation time and grid & initialization
    % --------------------------------------------------------------------
    t_sim_end = min([opts.t_sim, los.t_grid(end)]);
    fprintf('chosen simulation time: %.2f  seconds \n', t_sim_end);
    t_sim_grid = 0:rdr.rf.dt:t_sim_end;
    
    % generate frame by frame:
    N_frames = floor(t_sim_end/rdr.frame.rate); 
    Nchar_line = fprintf('processing frame %d of %d', 0, N_frames);

    [frame_adcOn_mask, frame_Tx_sel] = compute_frame_adc0n_mask_and_Tx_sel(rdr);
% %     Nsctr_total = size(los.tx_sctr,2);
% %     if ~isempty(fom) && ~isempty(som)
% %         Nsctr_total = Nsctr_total + size(fom.tx_sctr,2) + size(som.tx_sctr,2);
% %     end
    
    Nchirps_frame = rdr.frame.Nsamp_doppler*size(rdr.frame.tx_seq,2);
    if_sig = double.empty(rdr.chirp.Nsamp_range, Nchirps_frame, rdr.rf.Nrx, 0);
    for fr_idx = 1:N_frames
        fprintf(repmat('\b',1,Nchar_line));
        Nchar_line = fprintf('processing frame %d of %d', fr_idx, N_frames);
        
        % -----------------------------------------------------------------
        % Interpolate data in the frame
        % -----------------------------------------------------------------
        t_frame = t_sim_grid( (1:rdr.frame.Nsamp_active) + (fr_idx-1)*rdr.frame.Nsamp_passive_and_active);
        fr_mask = (los.t_grid >= t_frame(1) - 1/120) & (los.t_grid <= t_frame(end) + 1/120);
        fr_idxs = find(fr_mask==1);
        
        los_interp = interpolate_path_data(los, fr_idxs, t_frame(frame_adcOn_mask), rdr);
        los_interp = add_prop_distance_and_others(los_interp,frame_Tx_sel, rdr);

        if ~isempty(fom) && ~isempty(som)
            fom_interp = interpolate_path_data(fom, fr_idxs, t_frame(frame_adcOn_mask), rdr);
            fom_interp = add_prop_distance_and_others(fom_interp,frame_Tx_sel, rdr);

            som_interp = interpolate_path_data(som, fr_idxs, t_frame(frame_adcOn_mask), rdr);
            som_interp = add_prop_distance_and_others(som_interp,frame_Tx_sel, rdr);
        else
            fom_interp = {};
            som_interp = {};
        end
        '';
        % -----------------------------------------------------------------
        % test: to see we interpolated correctly
        % -----------------------------------------------------------------
% %         figure
% %         plot(los.t_grid(fr_idxs).',squeeze(los.tx_sctr(:,1,fr_idxs)).','o'); hold on;
% %         interp_data = los_interp.tx_sctr(:,1,:,:);
% %         sz = size(interp_data);
% %         interp_data = reshape(interp_data, sz(1), sz(2), prod(sz(3:end)));
% %         plot(t_frame(frame_adcOn_mask).',squeeze(interp_data).',':.');        

        % -----------------------------------------------------------------
        % Generate the IF-signal of frame:
        % -----------------------------------------------------------------
        if_sig_frame = IF_signals(los_interp, fom_interp, som_interp, rdr);
        if_sig = cat(4,if_sig,if_sig_frame);
    end
    fprintf('\n')
    
    '';
end