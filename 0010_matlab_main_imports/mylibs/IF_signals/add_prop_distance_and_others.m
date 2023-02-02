function path_interp = add_prop_distance_and_others(path_interp,frame_Tx_sel, rdr)

    % propagation distance:
    path_Tx_sel = repmat(frame_Tx_sel, 1,size(path_interp.tx_sctr,2),1,1);
    sz = size(path_interp.tx_sctr);
    tmp_tx_sctr = reshape(path_interp.tx_sctr(path_Tx_sel),1,sz(2),sz(3),sz(4));
    path_interp.prop_dist_m = tmp_tx_sctr + path_interp.sctr_rx;

    % compute path gain:
    sqrt_sigma = path_interp.sqrt_sigma(path_Tx_sel);
    sqrt_sigma = reshape(sqrt_sigma,1,sz(2),sz(3),sz(4));
    gamma_t = 10;% + rand(size(sqrt_sigma));
    path_interp.a_b = gamma_t.*sqrt_sigma./((0.5*path_interp.prop_dist_m).^2);

    % compute path wm, phim:
    path_interp.wm = 2*pi*rdr.chirp.S*path_interp.prop_dist_m*(1/rdr.rf.c);
    path_interp.phim = 2*pi*rdr.rf.fc*path_interp.prop_dist_m*(1/rdr.rf.c); % - 0.5*burst.S*(tau_m.^2));

    % select reflection coefitient:
    path_Tx_sel = repmat(frame_Tx_sel, 1,size(path_interp.tx_sctr,2),1,1);
    sz = size(path_interp.ref_coef);
    path_interp.reflex_coef = reshape(path_interp.ref_coef(path_Tx_sel),1,sz(2),sz(3),sz(4));
    '';
end