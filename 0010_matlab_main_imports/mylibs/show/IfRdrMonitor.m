classdef IfRdrMonitor
    % Notice: It only works with simple data generated from the mocap
    % ranges and velocities. NO IF-SIGNALS.
    % It displays the 1.- range doppler map, 2.- Spectrogram.

    properties
        fig
        handlers

        rng_lim
        vel_lim
        azim_lim

    end
    
    methods
        function obj = IfRdrMonitor(rng_lim, vel_lim, azim_lim)
            obj.rng_lim = fliplr(-rng_lim);
            obj.vel_lim = vel_lim;
            obj.azim_lim = azim_lim;
            
            obj.fig = figure('Position', [1 1121 1920 963], 'Name','IF_signal');
            %colormap jet
        end
        
        function obj = draw_rdm(obj, rdm, rdm_grids, Nrx_sel)
            figure(obj.fig);
            subplot(2,4,[1]);
            t_idx = 1;
            toplot = abs(rdm(:,:,Nrx_sel,t_idx));
            toplot = 10*log10(toplot);
            obj.handlers.rdm = imagesc(rdm_grids.doppler_grid, rdm_grids.range_grid, rot90(toplot,2));
            xlim(obj.vel_lim);
            ylim(obj.rng_lim);
            title('RDM'); xlabel('vel m/s'); ylabel('range m'); 
            hold on;
            '';
        end
        
        function obj = draw_range_profile(obj, rpm, rpm_grids, Nrx_sel)
            figure(obj.fig);
            subplot(2,4,[2])
            tmp = rpm(:,:,Nrx_sel);
            toplot = squeeze(abs(tmp));
            toplot = 10*log10(toplot);
            imagesc(rpm_grids.t_chirp_grid,rpm_grids.range_grid,rot90(toplot,2));
            ylim(obj.rng_lim);
            title('range profile'); xlabel('time s'); ylabel('range m'); 
            hold on;

        end

        function obj = draw_doppler_spect(obj, dsm, dsm_grids, Nrx_sel)
            figure(obj.fig);
            subplot(2,4,[3]);
            tmp = dsm(:,:,Nrx_sel);

            toplot = squeeze(abs(tmp));
            toplot = 10*log10(toplot);
            imagesc(dsm_grids.t_grid,dsm_grids.f_grid, flipud(toplot));
            ylim(obj.vel_lim);
            title('velocity profile'); xlabel('time s'); ylabel('velocity m/s'); 
            hold on;
            '';

        end
        
        function obj = draw_range_azim(obj, ram, ram_gs)
            figure(obj.fig);
            subplot(2,4,[4]);
            t_idx = 1;
            toplot = abs(ram(:,:,t_idx));
            toplot = 10*log10(toplot);
            obj.handlers.ram = imagesc(ram_gs.angle_grid, ram_gs.range_grid, rot90(toplot,2));
            xlim(obj.azim_lim); ylim(obj.rng_lim);
            title('range azimuth'); xlabel('azimuth deg'); ylabel('range m'); 
            hold on; grid on;
            '';
        end
        % -----------------------------------------------------------------
        % animation:
        function obj = update_rdm(obj, rdm, rdm_grids, Nrx_sel,t_idx)
            figure(obj.fig);
            subplot(2,4,[1]);
            toplot = (abs(rdm(:,:,Nrx_sel,t_idx)));
            toplot = 10*log10(toplot);
            set(obj.handlers.rdm, 'CData', rot90(toplot,2)); 
            title(t_idx);
            '';
        end
        function obj = update_range_azim(obj, ram, ram_gs, t_idx)
            figure(obj.fig);
            subplot(2,4,[4]);
            toplot = (abs(ram(:,:,t_idx)));
            toplot = 10*log10(toplot);
            set(obj.handlers.ram, 'CData', rot90(toplot,2));   
            '';
        end
    
    end
end

