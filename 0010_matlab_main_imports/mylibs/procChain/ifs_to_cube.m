% It extracts the chirps from the if-signal. It makes the cube
function if_cube = ifs_to_cube(ifsig, rdr)
    
    Nfr = fix(size(ifsig,2)/rdr.frame.Nsamp_active);
    k_init = floor(rdr.chirp.adc_start/rdr.rf.dt) + 1;
    k_end = k_init + rdr.chirp.Nsamp_range - 1;


    if_cube = zeros(rdr.chirp.Nsamp_range, ...
                    rdr.frame.Nsamp_doppler*size(rdr.frame.tx_seq,2),...
                    rdr.rf.Nrx, Nfr);
    for fr_idx = 1:Nfr
        ifsig_frame = ifsig(1, (1:rdr.frame.Nsamp_active) + (fr_idx-1)*rdr.frame.Nsamp_active, :);
        ifsig_frame = squeeze(ifsig_frame);
        
        % cube with idle samples and adc-off samples:
        tmp_cube = reshape(ifsig_frame, rdr.chirp.Nsamp_all, rdr.frame.Nsamp_doppler*size(rdr.frame.tx_seq,2),rdr.rf.Nrx);
        
        % remove the adc-off and idle samples:
        if_cube(:,:,:, fr_idx) = tmp_cube(k_init:k_end,:,:);
    end
    '';
end

