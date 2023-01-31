function if_cube = if_cube_add_noise(if_cube, SNR_rx_lin)
    
    Prx =  mean(sum(abs(if_cube).^2, [1,2]), "all");
    Pn = Prx/SNR_rx_lin;
    sigma =sqrt(Pn);
    
    noise = 0.5*sigma*randn(size(if_cube)) + 1j*0.5*sigma*randn(size(if_cube));
    if_cube = if_cube + noise;
end