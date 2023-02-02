function [frame_adcOn_mask, frame_Tx_sel] = compute_frame_adc0n_mask_and_Tx_sel(rdr)
    % -----------------------------------------------------------------
    % Compute mask in frame:
    %       - adcOn mask: '1': to compute the signal. '0 otherwise'.
    %       - Tx_sel: index of tx to be used for chirp.
    % -----------------------------------------------------------------
    chirp_mask = false(1, rdr.chirp.Nsamp_all);
    chirp_adcOn_idx = (1:1:rdr.chirp.Nsamp_range) + floor(rdr.chirp.adc_start/rdr.rf.dt);
    chirp_mask(chirp_adcOn_idx) = true;
    frame_adcOn_mask = repmat(chirp_mask, 1, rdr.frame.Nsamp_doppler*size(rdr.frame.tx_seq,2));
    
    frame_Tx_sel = false(3,size(rdr.frame.tx_seq,2));
    for i = 1:size(rdr.frame.tx_seq,2)
        frame_Tx_sel(i, rdr.frame.tx_seq(i)) = true;
    end
    frame_Tx_sel = repmat(frame_Tx_sel, rdr.chirp.Nsamp_range, 1);
    frame_Tx_sel = frame_Tx_sel(:);
    frame_Tx_sel = repmat(frame_Tx_sel,rdr.frame.Nsamp_doppler,1);
    frame_Tx_sel = reshape(frame_Tx_sel,3,1,rdr.chirp.Nsamp_range, rdr.frame.Nsamp_doppler*size(rdr.frame.tx_seq,2));
    

end