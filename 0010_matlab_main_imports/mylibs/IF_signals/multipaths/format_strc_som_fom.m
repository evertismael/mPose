function fsom = format_strc_som_fom(strc_fsom)
    fsom = {};
    if size(strc_fsom, 2) > 0
        fsom = strc_fsom{1};
        for idx = 2:size(strc_fsom, 2)
            fsom.tx_sctr = cat(2, fsom.tx_sctr, strc_fsom{idx}.tx_sctr);
            fsom.sctr_rx = cat(2, fsom.sctr_rx, strc_fsom{idx}.sctr_rx);
            fsom.rm = cat(1, fsom.rm, strc_fsom{idx}.rm);
            fsom.azm = cat(1, fsom.azm, strc_fsom{idx}.azm);
            fsom.elm = cat(1, fsom.elm, strc_fsom{idx}.elm);
            fsom.vm = cat(1, fsom.vm, strc_fsom{idx}.vm);
            fsom.tht_wave_sctr = cat(2, fsom.tht_wave_sctr, strc_fsom{idx}.tht_wave_sctr);            
            fsom.sqrt_sigma = cat(2, fsom.sqrt_sigma, strc_fsom{idx}.sqrt_sigma);            
            fsom.ref_coef = cat(2, fsom.ref_coef, strc_fsom{idx}.ref_coef);            
            
            fsom.pos = cat(2, fsom.pos, strc_fsom{idx}.pos);            
        end
    end
end