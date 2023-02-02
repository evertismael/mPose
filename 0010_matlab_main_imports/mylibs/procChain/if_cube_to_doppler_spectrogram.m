function [dsm, dsm_gs] = if_cube_to_doppler_spectrogram(rpm, rdr, winL, hop, Nfft_dtm)
    
    % Doppler-spectrogram map='dsm':
    % winL: number of range profiles to be used in the window.
    % hop: number of range profiles to hop for the next one e.g.(M/2)
    % Nfft_dtm: number of fft used for the doppler fft.
    
    Ndsm = fix(1 + (size(rpm,2)-winL)/hop);
    dsm = zeros(Nfft_dtm,Ndsm,size(rpm,3));
    sz_dtm_i = [size(rpm,1), Nfft_dtm, size(rpm,3)];

    wind = hann(winL).';
    for dtm_idx = 1:Ndsm
        subs_dtm = (1:winL) + (dtm_idx-1)*hop;
        s_win = rpm(:,subs_dtm,:);
        mean_s_win = mean(s_win,2);
        dtm_i = fftshift(fft((s_win-mean_s_win).*wind,Nfft_dtm,2),2);
        
        [~, max_rng_idx] = max(dtm_i,[],[1,2],'linear');
        [r, ~, ~] = ind2sub(sz_dtm_i,max_rng_idx);
        for i=1:size(rpm,3)
            r_win = -10:10 + r(i);
            r_win(r_win<1) = 1;
            r_win(r_win>size(rpm,1)) = size(rpm,1);
            tmp_i = sum(abs(dtm_i(r_win,:,i)),1);
            dsm(:,dtm_idx,i) = squeeze(tmp_i);
        end        
    end

    % ---------------------------------------------------------------------
    % create grid:
    % ---------------------------------------------------------------------
    dsm_dt = (rdr.chirp.duration*size(rdr.frame.tx_seq,2))*hop;
    dsm_gs.t_grid = (0:Ndsm-1)*dsm_dt;
    Ftot = (rdr.frame.Nsamp_doppler)*rdr.cube.doppler_res;
    dtm_df = Ftot/Nfft_dtm;
    dsm_gs.f_grid = ((0:Nfft_dtm-1) - Nfft_dtm/2)*dtm_df;
end