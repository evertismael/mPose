function burst = burst_prms(fc, tx_enable, tx_seq)
    % --------------------------------------------------------------------
    % RF:
    % --------------------------------------------------------------------
    rf.c = 2.99e8;
    rf.fc = fc;
    rf.ADC = 5e6;
    rf.dt = 1/rf.ADC;
    rf.lambda = rf.c/rf.fc;
    

    % receivers:
    rf.Nrx = 4; % always 4 antenas
    rf.rx_uvw = zeros(4,4);
    rf.rx_uvw(1,:) = (rf.lambda/2)*((1:4) - 2.5);
    rf.rx_uvw(4,:) = ones(1,4);

    % transmitters:
    rf.tx_enable = tx_enable;       % number of transmitters [1 0 0]-> only tx1 is on
    rf.Ntx = 3;
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
    chirp.idle = 60e-6;
    chirp.adc_start = 3e-6;
    chirp.ramp_end = 32e-6;
    chirp.S = 61e6/1e-6; % MHz/us
    chirp.Nsamp_range = 128;
    
    
    chirp.duration = chirp.idle + chirp.ramp_end;
    chirp.Nsamp_all = round(chirp.duration/rf.dt);
    chirp.B_effective = chirp.S*chirp.Nsamp_range*rf.dt;
    chirp.B_ramp = chirp.S*chirp.ramp_end;
    
    assert(chirp.ramp_end >= (chirp.adc_start + chirp.Nsamp_range*rf.dt), 'ramp_end too small');

    % --------------------------------------------------------------------
    % Frame:
    % --------------------------------------------------------------------
    frame.Nsamp_doppler = 128;
    frame.tx_seq = tx_seq;        % sequence of chirps
    frame.Nsamp_active = (chirp.Nsamp_all*size(frame.tx_seq,2))*frame.Nsamp_doppler;
    frame.rate = frame.Nsamp_active*rf.dt ;% 25e-3;
    %frame.rate = 25e-3;
    frame.duration_active = frame.Nsamp_active*rf.dt;

    frame.Nsamp_passive_and_active = round(frame.rate/rf.dt);
    

    assert(frame.rate >= (frame.duration_active), 'ramp_end too small');
    assert(all(rf.tx_enable(tx_seq)==1),'all tx in seq should be enabled');
    % --------------------------------------------------------------------
    % Radar processing:
    % --------------------------------------------------------------------
    cube.Nfft_range = 128;
    cube.Nfft_doppler = 128;    

    cube.range_res = rf.c/(2*chirp.B_effective);
    cube.range_max = (0.9*rf.ADC*rf.c)/(2*chirp.S);
    cube.Tcri = size(frame.tx_seq,2)*chirp.duration;
    cube.doppler_res = rf.lambda/(2*frame.Nsamp_doppler*cube.Tcri);
    cube.doppler_max = rf.lambda/(4*cube.Tcri);

    % --------------------------------------------------------------------
    % Output the config:
    % --------------------------------------------------------------------
    burst.rf = rf;
    burst.chirp = chirp;
    burst.frame = frame;
    burst.cube = cube;
end

