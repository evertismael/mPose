function [capon_spec, tht_gs_deg] = capon_batch(rdm, rdr, tht_deg_max, tht_deg_res)
    Nrng = size(rdm,1);
    % Nvel = size(rdm,2);
    Nrx = size(rdm,3);
    Nfr = size(rdm,4);
    % 0010: estimate Rxx PER TARGET (RANGE):
    Rxx_inv_all = zeros(Nrx, Nrx, Nrng, Nfr); 

    for fr_i = 1:Nfr
        for rng_i = 1:Nrng
            tmp_x = rdm(rng_i,:,:,fr_i); % (1, vel, ant, 1)
            tmp_x = permute(tmp_x,[1,3,2,4]); % (1, ant, vel, 1)
            tmp_x_H = permute(conj(tmp_x),[2,1,3,4]); % (ant, 1, vel, 1)
            Rxx = squeeze(mean(tmp_x.*tmp_x_H,3)); % (ant, ant, 1)
            Rxx_inv = inv(Rxx);
            Rxx_inv_all(:,:,rng_i,fr_i) = Rxx_inv;
            '';
        end
        '';
    end
    
    d = rdr.rf.lambda/2;
    tmp_rx = 0:Nrx-1;

    tht_gs_rad = deg2rad(-tht_deg_max:tht_deg_res:tht_deg_max);
    phi_n_gs = (2*pi*d/rdr.rf.lambda)*((tmp_rx).').*sin(tht_gs_rad);
    a_gs = exp(-1j*phi_n_gs); % (and, angle)
    
    % get spectrum:
    tmp_Rxx_inv_times_a_all = zeros(Nrx, size(a_gs,2), Nrng, Nfr);
    for i=1:Nrx
        tmp_row_Rxx = permute(Rxx_inv_all(i,:,:,:),[2,1,3,4]);
        tmp_Rxx_inv_times_a_all(i,:,:,:) = sum(tmp_row_Rxx.*a_gs, 1);
    end
    
    a_gs_H = conj(a_gs);
    capon_den = sum(a_gs_H.*tmp_Rxx_inv_times_a_all,1);
    capon_spec = 1./capon_den;
    capon_spec = squeeze(capon_spec);
    capon_spec = permute(capon_spec, [2,1,3]);
    % capon dim: (rng, angle, fr)


    % ourput grid:
    tht_gs_deg = rad2deg(tht_gs_rad);

    
end




