classdef SimpleRdrMonitor
    % Notice: It only works with simple data generated from the mocap
    % ranges and velocities. NO IF-SIGNALS.
    % It displays the 1.- range doppler map, 2.- Spectrogram.

    properties
        fig
        handlers
        marker = {'o', 'o', 'o', 'o'};
        color = {'b',  'g','r', 'm'};
        alpha = {1, 0.1, 0.1, 0.3};

        rng_lim
        vel_lim
        az_lim
    end
    
    methods
        function obj = SimpleRdrMonitor(rng_lim, vel_lim, az_lim)
            obj.rng_lim = rng_lim;
            obj.vel_lim = vel_lim;
            obj.az_lim = az_lim;

            obj.fig = figure('Position', [50 141 1196 770], 'Name','Cloud of points');
            '';
        end
        
        function obj = draw_rdm(obj, rm, vm, id)
            figure(obj.fig);
            subplot(2,4,[1]);
            obj.handlers.rdm{id} = scatter(vm(:,1),rm(:,1), 5,obj.color{id} ,obj.marker{id},'filled');
            obj.handlers.rdm{id}.MarkerFaceAlpha = obj.alpha{id};                        
            obj.handlers.rdm{id}.MarkerEdgeAlpha = obj.alpha{id};    
            ylim(obj.rng_lim); xlim(obj.vel_lim);
            title('RDM'); xlabel('vel m/s'); ylabel('range m'); 
            hold on; grid on;
        end

        function obj = draw_range_profile(obj, rm, t_grid)
            figure(obj.fig);
            subplot(2,4,[2]);
            plot(t_grid,squeeze(rm),'-'); hold on;
            title('range profile'); xlabel('time s'); ylabel('range m'); 
            hold on; grid on;
        end

        function obj = draw_doppler_spect(obj, vm, t_grid, id)
            figure(obj.fig);
            subplot(2,4,[3]);
            plot(t_grid,squeeze(vm),'Color',obj.color{id}, 'LineWidth',obj.alpha{id}); hold on;
            title('doppler spectrogram'); xlabel('time s'); ylabel('velocity m/s'); 
            ylim(obj.vel_lim);
            hold on; grid on;
        end

        function obj = draw_range_azim(obj, rm, azm_rad, id)
            figure(obj.fig);
            subplot(2,4,[4]);
            obj.handlers.ram{id} = scatter(rad2deg(azm_rad(:,1)),rm(:,1), 5,obj.color{id} ,obj.marker{id},'filled');
            obj.handlers.ram{id}.MarkerFaceAlpha = obj.alpha{id};                        
            obj.handlers.ram{id}.MarkerEdgeAlpha = obj.alpha{id}; 
            xlim(obj.az_lim); ylim(obj.rng_lim);
            title('range azimuth'); xlabel('azimuth deg'); ylabel('range m');
            hold on; grid on;
        end

        function obj = draw_range_elev(obj, rm, elm_rad, id)
            figure(obj.fig);
            subplot(2,4,[5]);
            obj.handlers.rem{id} = scatter(rad2deg(elm_rad(:,1)),rm(:,1), 5, obj.color{id} ,obj.marker{id},'filled');
            obj.handlers.rem{id}.MarkerFaceAlpha = obj.alpha{id};                        
            obj.handlers.rem{id}.MarkerEdgeAlpha = obj.alpha{id};
            xlim(obj.az_lim); ylim(obj.rng_lim);
            title('range elevation'); xlabel('elevation deg'); ylabel('range m'); 
            hold on; grid on;
        end
        
        function obj = draw_range_elev_yz(obj, rm, azm_rad, elm_rad, rdr,id)
            figure(obj.fig);
            subplot(2,4,[6]);

            % convert radial (rng, azm) to (uvw)
            % u = zeros
            v = rm(:,1).*cos(elm_rad(:,1)).*cos(azm_rad(:,1));
            w = rm(:,1).*sin(elm_rad(:,1));
            uvw = cat(1,zeros(size(v.')), v.',w.');
            xyz = rdr.rdr2glb_coords(uvw,'not-homogen');

            obj.handlers.rem_xy{id} = scatter( xyz(2,:), xyz(3,:), 5, obj.color{id} ,obj.marker{id},'filled');
            obj.handlers.rem_xy{id}.MarkerFaceAlpha = obj.alpha{id};                        
            obj.handlers.rem_xy{id}.MarkerEdgeAlpha = obj.alpha{id}; 
            title('range elevation (in yz)'); xlabel('y m'); ylabel('z m'); 
            xlim([-3 3]); ylim([0 2]);
            hold on; grid on;

            '';
        end
        
        function obj = draw_range_azim_xy(obj, rm, azm_rad, rdr, id)
            figure(obj.fig);
            subplot(2,4,[7]);

            % convert radial (rng, azm) to (uvw)
            u = rm(:,1).*sin(azm_rad(:,1));
            v = rm(:,1).*cos(azm_rad(:,1));
            uvw = cat(1,u.',v.',zeros(size(u.')));
            xyz = rdr.rdr2glb_coords(uvw,'not-homogen');

            obj.handlers.ram_xy{id} = scatter( xyz(1,:), xyz(2,:), 5, obj.color{id} ,obj.marker{id},'filled');
            obj.handlers.ram_xy{id}.MarkerFaceAlpha = obj.alpha{id};                        
            obj.handlers.ram_xy{id}.MarkerEdgeAlpha = obj.alpha{id}; 
            xlim([-2 2]); ylim([-3 3]);
            title('range azimuth (in xy)'); xlabel('x m'); ylabel('y m'); 
            hold on; grid on; axis equal;            
            '';
        end
        
        % -----------------------------------------------------------------
        % Animation:
        % -----------------------------------------------------------------
        function obj = update_rdm(obj, rm, vm, t_idx, id)
            set(obj.handlers.rdm{id}, 'Xdata', vm(:,t_idx), 'Ydata', rm(:,t_idx));
        end

        function obj = update_range_azim(obj, rm, azm_rad, t_idx, id)
            set(obj.handlers.ram{id}, 'Xdata', rad2deg(azm_rad(:,t_idx)), 'Ydata', rm(:,t_idx));
        end
   
        function obj = update_range_elev(obj, rm, elv_rad, t_idx, id)
            set(obj.handlers.rem{id},'Xdata', rad2deg(elv_rad(:,t_idx)), 'Ydata', rm(:,t_idx));
        end
    
        function obj = update_range_azim_xy(obj, rm, azm_rad, fr_idx, rdr, id)
            % convert radial (rng, azm) to (uvw)
            u = rm(:,fr_idx).*sin(azm_rad(:,fr_idx));
            v = rm(:,fr_idx).*cos(azm_rad(:,fr_idx));
            uvw = cat(1,u.',v.',zeros(size(u.')));
            xyz = rdr.rdr2glb_coords(uvw,'not-homogen');
            set(obj.handlers.ram_xy{id},'Xdata', xyz(1,:),'Ydata', xyz(2,:));            
            '';
        end
        
        function obj = update_range_elev_yz(obj, rm, azm_rad, elm_rad, t_idx, rdr, id)
            % convert radial (rng, azm) to (uvw)
            u = rm(:,t_idx).*cos(elm_rad(:,t_idx)).*sin(azm_rad(:,t_idx));
            v = rm(:,t_idx).*cos(elm_rad(:,t_idx)).*cos(azm_rad(:,t_idx));
            w = rm(:,t_idx).*sin(elm_rad(:,t_idx));
            
            uvw = cat(1,u.', v.',w.');
            xyz = rdr.rdr2glb_coords(uvw,'not-homogen');
            
            set(obj.handlers.rem_xy{id},'Xdata', xyz(2,:), 'Ydata', xyz(3,:));            
            '';
        end
    end
end

