classdef SceneMonitor
    properties
        fig_scene
        handlers

        marker = {'o', 'o', 'o', 'o', 'o'};
        color = {'b', 'r', 'g', 'm', 'k'};
        alpha = {1, 0.3, 0.3, 0.3, 0.3};
    end
    
    methods
        function obj = SceneMonitor(idx_fig,x_lim, y_lim, z_lim)
            obj.fig_scene = figure('Name',num2str(idx_fig));

            hold on; axis equal;
            xlim(x_lim); ylim(y_lim); zlim(z_lim);
            xlabel('x'); ylabel('y'); 
            view(90,45);
            grid on;
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

        function obj = draw_body(obj, mbody)
            figure(obj.fig_scene);
            
            % joints:
            fr_idx = 1; id = mbody.id;
            [jnts_xyz, ~] = mbody.get_joints_by_frame(fr_idx);
            obj.handlers.joints{id} = scatter3(jnts_xyz(1,:),jnts_xyz(2,:),jnts_xyz(3,:),3, ...
                obj.color{id} ,obj.marker{id},'filled');
            obj.handlers.joints{id}.MarkerFaceAlpha = obj.alpha{id};
        end
        
        function obj = draw_bone_ellipsoids(obj, mbody)
            figure(obj.fig_scene);
            fr_idx = 1; id = mbody.id;
            [pm, abc, Ggcorr_elps, ~] = mbody.get_bones_by_frame(fr_idx);
            
            % bones ellipsoid axes:
            [a2p_x, a2p_y, a2p_z] = Gmat2axes(Ggcorr_elps, 1);
            obj.handlers.elip_center{id} = plot3(pm(1,:),pm(2,:),pm(3,:),'r*','MarkerSize',3);
            obj.handlers.elip_axes_x{id} = plot3(a2p_x(1,:),a2p_x(2,:),a2p_x(3,:),'r','LineWidth',0.5);
            obj.handlers.elip_axes_y{id} = plot3(a2p_y(1,:),a2p_y(2,:),a2p_y(3,:),'g','LineWidth',0.5);
            obj.handlers.elip_axes_z{id} = plot3(a2p_z(1,:),a2p_z(2,:),a2p_z(3,:),'b','LineWidth',0.5);
            

            % bones ellipsoids
            [elip_skel] = batch_ellipsoid_gen(abc, Ggcorr_elps);
            obj.handlers.elip_skel{id} = plot3(elip_skel(1,:),elip_skel(2,:),elip_skel(3,:),...
                'Color', obj.color{id},'LineWidth',0.1);
            '';
        end

        % -----------------------------------------------------------------
        % Animate:
        % -----------------------------------------------------------------

        function obj = update_body(obj, mbody, fr_idx)
            figure(obj.fig_scene);
            id = mbody.id;
            [jnts_xyz, ~] = mbody.get_joints_by_frame(fr_idx);

            % update plot:
            set(obj.handlers.joints{id},...
                'Xdata', jnts_xyz(1,:), ...
                'Ydata', jnts_xyz(2,:), ...
                'Zdata', jnts_xyz(3,:));
        end
        
        function obj = update_bone_ellipsoids(obj, mbody, fr_idx)
            id = mbody.id;
            [pm, abc, Ggcorr_elps, ~] = mbody.get_bones_by_frame(fr_idx);
            
            % bones ellipsoid axes:
            [a2p_x, a2p_y, a2p_z] = Gmat2axes(Ggcorr_elps, 1);
            set(obj.handlers.elip_center{id}, 'Xdata', pm(1,:),'Ydata', pm(2,:), 'Zdata', pm(3,:));
            set(obj.handlers.elip_axes_x{id}, 'Xdata', a2p_x(1,:), 'Ydata', a2p_x(2,:), 'Zdata', a2p_x(3,:));
            set(obj.handlers.elip_axes_y{id}, 'Xdata', a2p_y(1,:), 'Ydata', a2p_y(2,:), 'Zdata', a2p_y(3,:));
            set(obj.handlers.elip_axes_z{id}, 'Xdata', a2p_z(1,:), 'Ydata', a2p_z(2,:), 'Zdata', a2p_z(3,:));

            % bones ellipsoids
            [elip_skel] = batch_ellipsoid_gen(abc, Ggcorr_elps);
            set(obj.handlers.elip_skel{id}, 'Xdata', elip_skel(1,:), 'Ydata', elip_skel(2,:), 'Zdata', elip_skel(3,:));
            
        end

       
        function obj = draw_trajectory(obj, mbody)
            figure(obj.fig_scene);
            
            % joints:
            torso_xyz = squeeze(mbody.jnts_xyz(:,1,:));
            t_axis = 1:5:size(torso_xyz,2);
            
            h_joints = scatter(torso_xyz(1,t_axis),torso_xyz(2,t_axis),0.1,'k','.');
            h_joints.MarkerFaceAlpha = .2;
            h_joints.MarkerEdgeAlpha = .2;
            obj.handlers.joints = h_joints;
        end
    end
end

