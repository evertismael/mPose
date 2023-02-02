function [rdm_cube, rdm_gs] = if_cube_to_rdm(if_cube, rdr)
    
    % range-profile:
    mean_rdr_cube = mean(if_cube,1);
    w_range = hann(size(if_cube,1));
    tmp1 = (if_cube - mean_rdr_cube).*w_range;
    range_prof = fft(tmp1,rdr.cube.Nfft_range,1);
    
    % doppler:
    w_doppler = hann(size(if_cube,2));
    w_doppler = w_doppler.';
    mean_rng_prof = mean(range_prof,2);
    rdm_cube = fftshift(fft((range_prof - mean_rng_prof).*w_doppler,rdr.cube.Nfft_doppler,2),2);
            
    % rdm grids:
    range_grid = ((0:1:(rdr.cube.Nfft_range-1)))*rdr.cube.range_res;
    range_grid = fliplr(range_grid);
    doppler_grid = ((0:rdr.cube.Nfft_doppler-1) - rdr.cube.Nfft_doppler/2 + 1)*rdr.cube.doppler_res;
    
    rdm_gs.range_grid = range_grid;
    rdm_gs.doppler_grid = doppler_grid;
end