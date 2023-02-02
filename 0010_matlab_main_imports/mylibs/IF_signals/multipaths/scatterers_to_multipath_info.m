% compute the LOS/(FOM/SOMultipah) measurements
%       range, vel, azim, elev, jnts, bone_center, time.
function [los, fom, som] = scatterers_to_multipath_info(sctrs, rdr, opts)
    
    % ---------------------------------------------------------------------
    % LOS:
    % ---------------------------------------------------------------------
    los = get_los_or_som_path_information(sctrs, rdr, 'los', 0);

    % ---------------------------------------------------------------------
    % SOM: Second Order Multipath:
    % ---------------------------------------------------------------------
    strc_som = {};
    if opts.floor.mpath_enable
        dept_vect = 0:opts.floor.d_step:opts.floor.depth;
        for d_i = 1:size(dept_vect,2)
            strc_som{d_i} = get_los_or_som_path_information(sctrs, rdr, 'floor', dept_vect(d_i));
        end
    end

    if opts.ceiling.mpath_enable
        dept_vect = 0:opts.ceiling.d_step:opts.ceiling.depth;
        N_som = size(strc_som,2);
        for d_i = 1:size(dept_vect,2)
            strc_som{N_som + d_i} = get_los_or_som_path_information(sctrs,...
                rdr, 'ceiling', opts.ceiling.height + dept_vect(d_i));
        end
        '';
    end
    % convert str_som into som (good format)
    som = format_strc_som_fom(strc_som);
    
    % ---------------------------------------------------------------------
    % FOM: First Order Multipath: (2 diff paths FOM per SOM):
    %    path-1: rdr-floor-body-rdr
    %    path-2: rdr-body-floor-rdr
    % ---------------------------------------------------------------------
    strc_fom = {};
    % path-1: rdr-floor-body-rdr
    for som_idx = 1:size(strc_som, 2)
        tmp_som = strc_som{som_idx};
        
        strc_fom{som_idx}.tx_sctr = tmp_som.tx_sctr;
        strc_fom{som_idx}.sctr_rx = los.sctr_rx;
        strc_fom{som_idx}.rm = 0.5*(tmp_som.rm + los.rm);
        strc_fom{som_idx}.azm = los.azm;
        strc_fom{som_idx}.elm = los.elm;
        strc_fom{som_idx}.vm =  tmp_som.vm;
        strc_fom{som_idx}.tht_wave_sctr =  tmp_som.tht_wave_sctr;
        strc_fom{som_idx}.sqrt_sigma =  tmp_som.sqrt_sigma;
        strc_fom{som_idx}.ref_coef =  tmp_som.ref_coef;

        strc_fom{som_idx}.pos = tmp_som.pos;
        strc_fom{som_idx}.t_grid = los.t_grid;
        '';
    end
    
    % path-2: rdr-body-floor-rdr
    Nfom = size(strc_fom,2);
    for som_idx = 1:size(strc_som, 2)
        tmp_som = strc_som{som_idx};
        strc_fom{Nfom + som_idx}.tx_sctr = los.tx_sctr;
        strc_fom{Nfom + som_idx}.sctr_rx = tmp_som.sctr_rx;
        strc_fom{Nfom + som_idx}.rm = 0.5*(tmp_som.rm + los.rm);
        strc_fom{Nfom + som_idx}.azm = tmp_som.azm;
        strc_fom{Nfom + som_idx}.elm = tmp_som.elm;
        strc_fom{Nfom + som_idx}.vm =  los.vm;
        strc_fom{Nfom + som_idx}.tht_wave_sctr =  los.tht_wave_sctr;
        strc_fom{Nfom + som_idx}.sqrt_sigma =  los.sqrt_sigma;
        strc_fom{Nfom + som_idx}.ref_coef =  tmp_som.ref_coef;
        strc_fom{Nfom + som_idx}.pos = tmp_som.pos;
        strc_fom{Nfom + som_idx}.t_grid = los.t_grid;
        '';
    end
    '';
    % convert str_som into som (good format)
    fom = format_strc_som_fom(strc_fom);
    '';
end