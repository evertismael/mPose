function path_interp = interpolate_path_data(path, fr_idxs, t_frame, rdr)
    % ---------------------------------------------------------------------
    % Collect all data to be interpolated: 
    %    tx_sctr, sctr_rx, sqrt_sigma, ref_coef
    % ---------------------------------------------------------------------
    tx_sctr = path.tx_sctr(:,:,fr_idxs);
    sctr_rx = path.sctr_rx(:,:,fr_idxs);
    sqrt_sigma = path.sqrt_sigma(:,:,fr_idxs);
    ref_coef = path.ref_coef(:,:,fr_idxs);
    
    % ---------------------------------------------------------------------
    % pack all data into a single vector to interpolate at once:
    % ---------------------------------------------------------------------
    
    % bring frame-dim to first position:
    tx_sctr = permute(tx_sctr, [3,1,2]); % fr, tx, sctr
    sctr_rx = permute(sctr_rx, [3,1,2]); % fr, rx, sctr
    sqrt_sigma = permute(sqrt_sigma, [3,1,2]); % fr, tx, sctr
    ref_coef = permute(ref_coef, [3,1,2]); % fr, tx, sctr
        
    % keep the sizes:
    sz_tx_sctr = size(tx_sctr);
    sz_sctr_rx = size(sctr_rx);
    sz_sqrt_sigma = size(sqrt_sigma);
    sz_ref_coef = size(ref_coef);
    
    Nfr = size(tx_sctr,1);
    v = double.empty(Nfr,0);
    v = cat(2,v, reshape(tx_sctr, [Nfr, prod(sz_tx_sctr(2:end))]));
    v = cat(2,v, reshape(sctr_rx, [Nfr, prod(sz_sctr_rx(2:end))]));
    v = cat(2,v, reshape(sqrt_sigma, [Nfr, prod(sz_sqrt_sigma(2:end))]));
    v = cat(2,v, reshape(ref_coef, [Nfr, prod(sz_ref_coef(2:end))]));
    
    % ---------------------------------------------------------------------
    % Interpolate
    % ---------------------------------------------------------------------
    t_v = path.t_grid(fr_idxs).';
    v_interp = interp1(t_v, v, t_frame.','spline');
    
    
    % ---------------------------------------------------------------------
    % prepare to unpack
    % ---------------------------------------------------------------------
    Nt_frame = size(v_interp,1);
    sz_tx_sctr(1) = Nt_frame;
    sz_sctr_rx(1) = Nt_frame;
    sz_sqrt_sigma(1) = Nt_frame;
    sz_ref_coef(1) = Nt_frame;
    
    % ---------------------------------------------------------------------
    % unpack
    % ---------------------------------------------------------------------
    path_interp = {};
    v_init = 0;
    v_sel = v_init + (1:prod(sz_tx_sctr(2:end)));
    path_interp.tx_sctr = reshape(v_interp(:, v_sel ), sz_tx_sctr);

    v_init = v_sel(end);
    v_sel = v_init + (1:prod(sz_sctr_rx(2:end)));
    path_interp.sctr_rx = reshape(v_interp(:, v_sel), sz_sctr_rx);

    v_init = v_sel(end);
    v_sel = v_init + (1:prod(sz_sqrt_sigma(2:end)));
    path_interp.sqrt_sigma = reshape(v_interp(:, v_sel), sz_sqrt_sigma);

    v_init = v_sel(end);
    v_sel = v_init + (1:prod(sz_ref_coef(2:end)));
    path_interp.ref_coef = reshape(v_interp(:, v_sel), sz_ref_coef);

    
    % ---------------------------------------------------------------------
    % put frame dim to the last position:
    % ---------------------------------------------------------------------
    path_interp.tx_sctr = permute(path_interp.tx_sctr, [2,3,1]); % tx, sctr, fr
    path_interp.sctr_rx = permute(path_interp.sctr_rx, [2,3,1]); % rx, sctr, fr
    path_interp.sqrt_sigma = permute(path_interp.sqrt_sigma, [2,3,1]); % tx, sctr, fr
    path_interp.ref_coef = permute(path_interp.ref_coef, [2,3,1]); % tx, sctr, fr

    % ---------------------------------------------------------------------
    % reshape, fr contains all adcOn samples per chirp.
    % ---------------------------------------------------------------------
    % final dimention: tx, sctr, chirp_samp, chirp
    Nchirps = rdr.frame.Nsamp_doppler*size(rdr.frame.tx_seq,2);
    sz = size(path_interp.tx_sctr);
    path_interp.tx_sctr = reshape(path_interp.tx_sctr, sz(1), sz(2),rdr.chirp.Nsamp_range, Nchirps);
    
    % final dimention: rx, sctr, chirp_samp, chirp
    sz = size(path_interp.sctr_rx);
    path_interp.sctr_rx = reshape(path_interp.sctr_rx, sz(1), sz(2),rdr.chirp.Nsamp_range, Nchirps);

    % final dimention: tx, sctr, chirp_samp, chirp
    sz = size(path_interp.sqrt_sigma);
    path_interp.sqrt_sigma = reshape(path_interp.sqrt_sigma, sz(1), sz(2),rdr.chirp.Nsamp_range, Nchirps);

    % final dimention: tx, sctr, chirp_samp, chirp
    sz = size(path_interp.ref_coef);
    path_interp.ref_coef = reshape(path_interp.ref_coef, sz(1), sz(2),rdr.chirp.Nsamp_range, Nchirps);
    '';
end