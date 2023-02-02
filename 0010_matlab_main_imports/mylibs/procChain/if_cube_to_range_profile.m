function [rpm, rpm_gs] = if_cube_to_range_profile(if_cube, rdr)
    
    % range-profile:
    mean_rdr_cube = mean(if_cube,1);
    w_range = hann(size(if_cube,1));
    tmp1 = (if_cube - mean_rdr_cube).*w_range;
    rpm = fft(tmp1,rdr.cube.Nfft_range,1);
    rpm = permute(rpm,[1,2,4,3]);
    rpm = reshape(rpm,size(rpm,1), size(rpm,2)*size(rpm,3), size(rpm,4));

    % rdm grids:
    range_grid = ((0:1:(rdr.cube.Nfft_range-1)))*rdr.cube.range_res;
    range_grid = fliplr(range_grid);
    
    rpm_gs.range_grid = range_grid;
    
    chirp_rate = rdr.chirp.duration*size(rdr.frame.tx_seq,2);
    rpm_gs.t_chirp_grid = (1:size(rpm,2))*chirp_rate;
end