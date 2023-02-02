function rdrp = burst_prms_from_ti_cfg(rdim, rfp, tmp_cfg)
    % --------------------------------------------------------------------
    % RF:
    % --------------------------------------------------------------------
    rf.c = 2.99e8;
    rf.fc = rfp.startFreq*1e9;
    rf.ADC = 5e6;
    rf.dt = 1/rf.ADC;
    rf.lambda = rf.c/rf.fc;

    % receivers:
    rf.Nrx = 4; % always 4 antenas
    rf.rx_uvw = zeros(4,4);
    rf.rx_uvw(1,:) = (rf.lambda/2)*((1:4) - 2.5);
    rf.rx_uvw(4,:) = ones(1,4);

    % transmitters:
    rf.Ntx_enable = rdim.numTxAnt;       % number of transmitters [1 0 0]-> only tx1 is on
    rf.tx_uvw = (rf.lambda)*[0  -1  -2;
                             0   0   0;
                             0   0.5 0;
                             1   1   1;];
    rf.tx_uvw(1:3,:) = rf.tx_uvw(1:3,:) - (rf.lambda)*([2.25; 0; 0]); % offset See picture of TI-Radar 18XX.
    rf.tx_uvw(4,:) = ones(1,3);
    % uncomment to see antenna disposition:
% % %     figure;
% % %     plot3(rf.rx_uvw(1,:),rf.rx_uvw(2,:),rf.rx_uvw(3,:), '.'); hold on;
% % %     plot3(rf.tx_uvw(1,:),rf.tx_uvw(2,:),rf.tx_uvw(3,:), 'r.'); grid on;
% % %     axis equal
% % %     xlabel('x'); ylabel('y'); 
    
    % --------------------------------------------------------------------
    % CHIRP:
    % --------------------------------------------------------------------
    chirp.idle = tmp_cfg.chirp_o.idle_us*(1e-6); % irrelevant
    chirp.adc_start = tmp_cfg.chirp_o.adc_start_us*(1e-6);
    chirp.ramp_end = tmp_cfg.chirp_o.ramp_end_us*(1e-6);
    chirp.S = rfp.freqSlope*(1e6/1e-6); % MHz/us
    chirp.Nsamp_range = rfp.numRangeBins;
    
    
    chirp.duration = chirp.idle + chirp.ramp_end;
    chirp.Nsamp_all = NaN;
    chirp.B_effective = NaN;
    chirp.B_ramp = NaN;
    
    % --------------------------------------------------------------------
    % Frame:
    % --------------------------------------------------------------------
    frame.Nsamp_doppler = rfp.numDopplerBins;
    frame.tx_seq = 1:rdim.numTxAnt;        % sequence of chirps
    frame.Nsamp_active = NaN;
    frame.rate = rfp.framePeriodicity*1e-3;
    frame.duration_active = NaN;

    frame.Nsamp_passive_and_active = NaN;
    

    % --------------------------------------------------------------------
    % Radar processing:
    % --------------------------------------------------------------------
    cube.Nfft_range = rfp.numRangeBins;
    cube.Nfft_doppler = rfp.numDopplerBins;    

    cube.range_res = rfp.rangeResolutionsInMeters;
    cube.range_max = NaN;
    cube.Tcri = NaN;
    cube.doppler_res = rfp.dopplerResolutionMps;
    cube.doppler_max = NaN;

    % --------------------------------------------------------------------
    % Output the config:
    % --------------------------------------------------------------------
    rdrp.rf = rf;
    rdrp.chirp = chirp;
    rdrp.frame = frame;
    rdrp.cube = cube;


end