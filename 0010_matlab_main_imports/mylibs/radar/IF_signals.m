function [ifsig, jnt_in_frame] = IF_signals(t_frame, ibodies, rdr, opts)

% inputs:
%  t_frame: time of samples in frame (1 x Ntime)
%  ibodies: interpolated (to rf_ADC rate) data from mocap data.
%           first body is LOS, second and tird are multipaths
%  rdr: radar for which communicate the if-signals.
%  opts: additional options. e.g. multipath generation.

glbp = glb_prms();


% -----------------------------------------------------------------
% Compute:
%       - adcOn mask: '1': to compute the signal. '0 otherwise'.
%       - Tx_sel: index of tx to be used for chirp.
% -----------------------------------------------------------------
chirp_mask = false(1, rdr.chirp.Nsamp_all);
chirp_adcOn_idx = (1:1:rdr.chirp.Nsamp_range) + floor(rdr.chirp.adc_start/rdr.rf.dt);
chirp_mask(chirp_adcOn_idx) = true;
adcOn_mask = repmat(chirp_mask, 1, rdr.frame.Nsamp_doppler*size(rdr.frame.tx_seq,2));
Tx_chirp = repmat(rdr.frame.tx_seq, rdr.chirp.Nsamp_all, 1);
Tx_chirp = Tx_chirp(:);
Tx_chirp = repmat(Tx_chirp.',1,rdr.frame.Nsamp_doppler);

% -----------------------------------------------------------------
% Compute values that can be computed in batch (per ibody)
% -----------------------------------------------------------------
Tx_pos = rdr.rf.tx_glb(1:3,:); % dim: xyz, tx_idx
Rx_pos = rdr.rf.rx_glb(1:3,:); % dim: xyz, rx_idx
Tx_pos = Tx_pos(:,Tx_chirp(adcOn_mask)); % dim: xyz, time, tx_idx, scatter
Rx_pos = permute(Rx_pos,[1 3 2 4]); % dim: xyz, time, rx_idx, scatter


wm_frame = double.empty(1,size(adcOn_mask,2),rdr.rf.Nrx,0);
phim_frame = double.empty(1,size(adcOn_mask,2),rdr.rf.Nrx,0);
a_b_frame =  double.empty(1,size(adcOn_mask,2),rdr.rf.Nrx,0);
'';
for b_idx = 1:size(ibodies,2)
    % -----------------------------------------------------------------
    % Compute distances: scatter->Tx, scatter->Rx
    % -----------------------------------------------------------------
    sctr_pos = permute(ibodies{b_idx}.pm(1:3,:,:,:),[1 3 4 2]);               % dim: xyz, time, antenna, scatter
    ibodies{b_idx}.d_scttr_tx = nan(1, size(sctr_pos,2),1,size(sctr_pos,4));  % dim: xyz, time, tx/rx, scatter
    ibodies{b_idx}.d_scttr_rx = nan(1, size(sctr_pos,2),size(Rx_pos,3),size(sctr_pos,4));
    
    % compute distances only when adc is on:
    rdr_wave_vect = sctr_pos(:,adcOn_mask,:,:) - Tx_pos;
    ibodies{b_idx}.d_scttr_tx(:,adcOn_mask,:,:) = sqrt(sum((rdr_wave_vect).^2,1));
    ibodies{b_idx}.d_scttr_rx(:,adcOn_mask,:,:) = sqrt(sum((sctr_pos(:,adcOn_mask,:,:) - Rx_pos).^2,1));
    

    % -------------------------------------------------------------
    % Propagation distance:
    % -------------------------------------------------------------
    ibodies{b_idx}.prop_dist_m = nan(1, size(sctr_pos,2),size(Rx_pos,3),size(sctr_pos,4));  % dim: 1, time, 1, scatter
    ibodies{b_idx}.prop_dist_m(:,adcOn_mask,:,:) = (ibodies{b_idx}.d_scttr_tx(1,adcOn_mask,1,:) + ...
                                                    ibodies{b_idx}.d_scttr_rx(1,adcOn_mask,:,:));
    
    if opts.reflection_gain
        % ---------------------------------------------------------------------
        % Angle between radar-wave and bone mayor-axis:
        % ---------------------------------------------------------------------
        ibodies{b_idx} = ibody_include_angle_scatter_radar_wave(ibodies{b_idx}, rdr, rdr_wave_vect, adcOn_mask);
        
        % -------------------------------------------------------------
        % Propagation gains:
        %   - Fresnel reflection coefficient
        %   - sigma: rcs
        %   - gamma: progagation effects (ant attenuation, proc_gains, etc)
        % -------------------------------------------------------------
        % TODO: compute Fresnel coef
        F = 0.6; 
        
        % rcs of primitives:
        skel_a = permute(ibodies{b_idx}.abc(1,:,adcOn_mask),[1 3, 4, 2]);
        skel_b = permute(ibodies{b_idx}.abc(3,:,adcOn_mask),[1 3, 4, 2]);
        skel_c = permute(ibodies{b_idx}.abc(2,:,adcOn_mask),[1 3, 4, 2]); % symetry axes:c in skel is 2
        theta_i = ibodies{b_idx}.angle_scatter_wave_rad(1,adcOn_mask,1,:);
        sigma_den = ((skel_a.^2).*(sin(theta_i).^2) +  (skel_c.^2).*(cos(theta_i).^2)).^2;
        sigma_num = (pi*skel_b.^4.*skel_c.^2);
        sigma_b = sqrt(F).* sigma_num./sigma_den;
    
        % gamma
        gamma = 10;% + randn(size(sigma_b));
        
        % finally; compute reflectivities for each primitive:
        ibodies{b_idx}.a_b = nan(size(ibodies{b_idx}.prop_dist_m));
        ibodies{b_idx}.a_b(1,adcOn_mask,:,:) = gamma.*sqrt(sigma_b)./(ibodies{b_idx}.prop_dist_m(:,adcOn_mask,:,:).^2);
        '';
    else
        ibodies{b_idx}.a_b = nan(size(ibodies{b_idx}.prop_dist_m));
        ibodies{b_idx}.a_b(1,adcOn_mask,:,:) = ones(size(ibodies{b_idx}.a_b(1,adcOn_mask,:,:)));
    end


    % ---------------------------------------------------------------------
    % Concatenate: wm_frame, phim_frame, a_b_frame for easy access in loop:
    % ---------------------------------------------------------------------
    wm_frame = cat(4,wm_frame,2*pi*rdr.chirp.S*ibodies{b_idx}.prop_dist_m*(1/glbp.c));
    phim_frame = cat(4,phim_frame,2*pi*rdr.rf.fc*ibodies{b_idx}.prop_dist_m*(1/glbp.c)); % - 0.5*burst.S*(tau_m.^2));
    a_b_frame = cat(4,a_b_frame, ibodies{b_idx}.a_b);
end

% -----------------------------------------------------------------
% Start generating the IF signal: Sample by sample:
% -----------------------------------------------------------------
frame_arg_t = nan(1,rdr.frame.Nsamp_active,rdr.rf.Nrx, size(wm_frame,4));
for ch_i = 1:(rdr.frame.Nsamp_doppler*size(rdr.frame.tx_seq,2))
    smp_ch_k = (1:rdr.chirp.Nsamp_all) + (ch_i-1)*rdr.chirp.Nsamp_all;
    k_init = floor(rdr.chirp.adc_start/rdr.rf.dt) + 1;
    k_end = k_init + rdr.chirp.Nsamp_range - 1;

    for k = k_init:k_end
        if k == k_init
            frame_arg_t(1,smp_ch_k(k),:,:) = 0;
        else
            frame_arg_t(1,smp_ch_k(k),:,:) = frame_arg_t(1,smp_ch_k(k-1),:,:) + wm_frame(1,smp_ch_k(k-1),:,:)*rdr.rf.dt;
        end
    end
end

% ---------------------------------------------------------------------
tmp_IF = a_b_frame.*exp(-1j*(phim_frame + frame_arg_t));
%tmp_IF = exp(-1j*(phim_frame + frame_arg_t));

% add among the scatterers:
ifsig = sum(tmp_IF,4);
jnt_in_frame = 0;
'';
end


