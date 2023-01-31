function [ram, ram_gs] = if_rdm_to_ram(rdm, rdr, rdm_gs)
    % rdm: (rng, vel, antena, frame):
    tht_deg_max = 90;
    tht_deg_res = 1;
    [capon_spec, tht_gs_deg]  = capon_batch(rdm, rdr,tht_deg_max, tht_deg_res);
    
    ram = capon_spec; % (rng, angle, fr)

    % grids:
    ram_gs.range_grid = rdm_gs.range_grid;
    ram_gs.angle_grid = tht_gs_deg;

    '';
end