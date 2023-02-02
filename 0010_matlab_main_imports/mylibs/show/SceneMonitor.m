classdef SceneMonitor
    properties
        fig_scene
        hdlrs

        marker = {'o', 'o', 'o', 'o', 'o'};
        color = {'b', 'r', 'g', 'm', 'k'};
        alpha = {1, 0.3, 0.3, 0.3, 0.3};
    end
    
    methods
        function obj = SceneMonitor(idx_fig,x_lim, y_lim, z_lim)
            obj.fig_scene = figure('Name',num2str(idx_fig));
            axis equal;
            xlim(x_lim); ylim(y_lim); zlim(z_lim);
            xlabel('x'); ylabel('y'); 
            view(45,30);
            grid on; hold on;
            '';
        end
        
        function obj = draw_floor_ceiling(obj, ceiling_height)
            figure(obj.fig_scene);
            patch(10*[1 -1 -1 1], 10*[1 1 -1 -1], [0 0 0 0], 'r');
            alpha(0.05);
            patch(10*[1 -1 -1 1], 10*[1 1 -1 -1], ceiling_height*[1 1 1 1], 'r');
            alpha(0.05);
            '';
        end

        function obj = draw_radar(obj, rdr)
            figure(obj.fig_scene);
            
            % antennas:
            plot3(rdr.rf.tx_glb(1,:),rdr.rf.tx_glb(2,:),rdr.rf.tx_glb(3,:),'.');
            plot3(rdr.rf.rx_glb(1,:),rdr.rf.rx_glb(2,:),rdr.rf.rx_glb(3,:),'.');

            % radar axes:
            scale = 0.4;
            axes_rdr = [zeros(3,1), scale*eye(3,3); ones(1,4)];
            axes_glb = rdr.rdr2glb_coords(axes_rdr, 'not-homogen');
            ax_x = axes_glb(:,[1,2]);
            ax_y = axes_glb(:,[1,3]);
            ax_z = axes_glb(:,[1,4]);
            
            
            plot3(ax_x(1,:), ax_x(2,:), ax_x(3,:), 'r');
            plot3(ax_y(1,:), ax_y(2,:), ax_y(3,:), 'g');
            plot3(ax_z(1,:), ax_z(2,:), ax_z(3,:), 'b');
            '';
        end

        function obj = draw_jnts(obj, jnts)
            figure(obj.fig_scene);
            fr_idx = 1;
            jnts_xyz = jnts.pos(:,:,fr_idx);
            obj.hdlrs.joints{jnts.id} = scatter3(jnts_xyz(1,:),jnts_xyz(2,:),jnts_xyz(3,:),3);
            obj.hdlrs.joints{jnts.id}.MarkerFaceColor = obj.color{jnts.id};
            obj.hdlrs.joints{jnts.id}.MarkerFaceAlpha = obj.alpha{jnts.id};
            obj.hdlrs.joints{jnts.id}.Marker = obj.marker{jnts.id};
            '';
        end
        
        function obj = draw_sctrs(obj, sctrs)
            figure(obj.fig_scene);
            fr_idx = 1; id = sctrs.id;
            pos = sctrs.pos(:,:,fr_idx);
            Gg_elps = sctrs.Gg_elps(:,:,:,fr_idx);
            abc = sctrs.abc(:,:,fr_idx);

            % bones ellipsoid axes:
            [a2p_x, a2p_y, a2p_z] = Gmat2axes(Gg_elps, 1);
            obj.hdlrs.elip_center{id} = plot3(pos(1,:),pos(2,:),pos(3,:),'r*','MarkerSize',3);
            obj.hdlrs.elip_axes_x{id} = plot3(a2p_x(1,:),a2p_x(2,:),a2p_x(3,:),'r','LineWidth',0.5);
            obj.hdlrs.elip_axes_y{id} = plot3(a2p_y(1,:),a2p_y(2,:),a2p_y(3,:),'g','LineWidth',0.5);
            obj.hdlrs.elip_axes_z{id} = plot3(a2p_z(1,:),a2p_z(2,:),a2p_z(3,:),'b','LineWidth',0.5);
            

            % bones ellipsoids
            [elip_skel] = batch_ellipsoid_gen(abc, Gg_elps);
            obj.hdlrs.elip_skel{id} = plot3(elip_skel(1,:),elip_skel(2,:),elip_skel(3,:),...
                'Color', obj.color{id},'LineWidth',0.1);
            '';
        end

        % -----------------------------------------------------------------
        % Animate:
        % -----------------------------------------------------------------

        function obj = update_jnts(obj, jnts, fr_idx)
            id = jnts.id;
            jnts_xyz = jnts.pos(:,:,fr_idx);

            % update plot:
            set(obj.hdlrs.joints{id}, 'Xdata', jnts_xyz(1,:), 'Ydata', jnts_xyz(2,:), 'Zdata', jnts_xyz(3,:));
        end
        
        function obj = update_sctrs(obj, sctrs, fr_idx)
            id = sctrs.id;
            pos = sctrs.pos(:,:,fr_idx);
            Gg_elps = sctrs.Gg_elps(:,:,:,fr_idx);
            abc = sctrs.abc(:,:,fr_idx);
            
            % bones ellipsoid axes:
            [a2p_x, a2p_y, a2p_z] = Gmat2axes(Gg_elps, 1);
            set(obj.hdlrs.elip_center{id}, 'Xdata', pos(1,:),'Ydata', pos(2,:), 'Zdata', pos(3,:));
            set(obj.hdlrs.elip_axes_x{id}, 'Xdata', a2p_x(1,:), 'Ydata', a2p_x(2,:), 'Zdata', a2p_x(3,:));
            set(obj.hdlrs.elip_axes_y{id}, 'Xdata', a2p_y(1,:), 'Ydata', a2p_y(2,:), 'Zdata', a2p_y(3,:));
            set(obj.hdlrs.elip_axes_z{id}, 'Xdata', a2p_z(1,:), 'Ydata', a2p_z(2,:), 'Zdata', a2p_z(3,:));

            % bones ellipsoids
            [elip_skel] = batch_ellipsoid_gen(abc, Gg_elps);
            set(obj.hdlrs.elip_skel{id}, 'Xdata', elip_skel(1,:), 'Ydata', elip_skel(2,:), 'Zdata', elip_skel(3,:));
        end

    end
end

