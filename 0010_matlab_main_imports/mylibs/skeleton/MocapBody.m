classdef MocapBody
    % It reads the data from a mocap, asf file and manipulates the skeleton
    % to obtain the joints and bones:
    % joints: 3D points over time.
    % bones: (pm,Am,abc, rot) -> center, gain, mayor/minor axes, rotation.

    properties
        id
        w_size = 20;
        XYZ_FACTOR = (1/0.45)*2.54/100.0; % conversion from inches and other factors;

        Nfr
        t_grid
        
        % skeleton:
        scatter_res = 80;  %in metters
        skel
        channels
        
        % informative:
        Njnt
        Nscttr
    end
    
    methods
        function obj = MocapBody(dataset_path,subject, trial)
            % -------------------------------------------------------------
            % Load data:
            % -------------------------------------------------------------
            fname_amc = [dataset_path,'all_asfamc/subjects/',subject,'/',subject,'_',num2str(trial),'.amc'];
            fname_asf = [dataset_path,'all_asfamc/subjects/',subject,'/',subject,'.asf'];
            skel_asf = acclaimReadSkel(fname_asf);
            
            
            % -------------------------------------------------------------
            % Skeleton + rot-angles (movemenent->channels)
            %       - channels: 2D matrix: (time vs rot-angles)
            %       - skel: T pose + with rot-constraints
            % -------------------------------------------------------------
            [channels, skel] = acclaimLoadChannels(fname_amc, skel_asf);
            skel = compute_G_all(skel, 1); % resting-homogen transf. mats G

            % skel axes output order is XZY with scale factor. 
            % To compensate for that such that output is XYZ with scale 1:
            skel.G_corr = [obj.XYZ_FACTOR 0              0           0;
                           0              0           obj.XYZ_FACTOR 0;
                           0        obj.XYZ_FACTOR       0           0;
                           0              0              0           1];
            skel.G_uncorr = [inv(skel.G_corr(1:3, 1:3)) zeros(3,1); zeros(1,3), 1];
            
            % add boneSkel: hard-defined:
            scale = 1/obj.XYZ_FACTOR;
            skel = include_map_bones_2_width(skel,subject,scale);
            skel.scatter_res = obj.scatter_res;
            
            % -------------------------------------------------------------
            % smooth the channels (reduce mocap noise)
            % -------------------------------------------------------------
            window_filt = hann(obj.w_size);
            window_filt = window_filt.'/sum(window_filt);
            if obj.w_size > 2
                init = zeros(obj.w_size-1,1);   
                channels_smooth = filter(window_filt, 1, channels,init,1);
                channels = channels_smooth(obj.w_size:end-obj.w_size/2,:); % avoid transient
                '';
            end
            
            % output:
            obj.Nfr = size(channels,1);
            obj.t_grid = (0:(obj.Nfr-1))*(1/120); 
            obj.skel = skel;
            obj.channels = channels.';   
            
            % informative output:
            [jnts_xyz, ~] = get_joints_by_frame(obj, 1);
            [pm, ~, ~, ~] = get_bones_by_frame(obj, 1);
            obj.Njnt = size(jnts_xyz,2);
            obj.Nscttr = size(pm, 2);
            '';
        end

        function [jnts_xyz, jnts_names] = get_joints_by_frame(obj, fr_idx)
            
            % update skeleton with channels:
            tmp_skel = update_DoF(obj.skel, 1, obj.channels(:,fr_idx).'); 

            % obtain joints:
            [jnts_xyz, jnts_names] = skel2jnts(tmp_skel);
        end

        function [pm, abc, Ggcorr_elps, Ggcorr_skel_jnt] = get_bones_by_frame(obj, fr_idx)

            % update skeleton with channels:
            tmp_skel = update_DoF(obj.skel, 1, obj.channels(:,fr_idx).'); 

            % get bone params:
            [pm, Ggcorr_skel_jnt, abc, Ggcorr_elps] = skel2bones(tmp_skel);
        end
        
        function obj = add_static_point(obj, p0)
            
            p0_mat = repmat(p0, 1, 1, size(obj.t_grid,2));
            
            Gg0_skel = [eye(3,3), p0([1,3,2])./obj.XYZ_FACTOR; ones(1,4)];
            Gg0_skel_mat = repmat(Gg0_skel, 1,1,1,size(obj.t_grid,2));
            abc0_mat = repmat(0.5*ones(3,1), 1, 1, size(obj.t_grid,2))./obj.XYZ_FACTOR;

            % add one scatterer at point p0.
            obj.jnts_xyz = cat(2,obj.jnts_xyz,p0_mat);
            obj.Gg_skel = cat(3,obj.Gg_skel,Gg0_skel_mat);
            obj.pm = cat(2,obj.pm,p0_mat);
            obj.abc = cat(2,obj.abc,abc0_mat);
            obj.Gg_elps = cat(3,obj.Gg_elps,Gg0_skel_mat);
            '';
        end
        
        function [sctrs, jnts] = get_scatterers(obj)
            % -------------------------------------------------------------
            % Obtain scatter information: pos, abc, Gg_elip
            % -------------------------------------------------------------
            jnts.pos = zeros(3,obj.Njnt, obj.Nfr);
            sctrs.pos = zeros(3,obj.Nscttr, obj.Nfr);
            sctrs.abc = zeros(3,obj.Nscttr, obj.Nfr);
            sctrs.Gg_elps = zeros(4,4,obj.Nscttr, obj.Nfr);
            sctrs.Gg_skel = zeros(4,4,obj.Njnt, obj.Nfr);
            for fr_idx = 1:obj.Nfr
                [jnts.pos(:,:,fr_idx), ~] = obj.get_joints_by_frame(fr_idx);
                [pm, abc, Ggcorr_elps, Ggcorr_skel_jnt] = obj.get_bones_by_frame(fr_idx);
                sctrs.pos(:,:,fr_idx) = pm(1:3,:,:);
                sctrs.abc(:,:,fr_idx) = abc;
                sctrs.Gg_elps(:,:,:,fr_idx) = Ggcorr_elps;
                sctrs.Gg_skel(:,:,:,fr_idx) = Ggcorr_skel_jnt;
            end
            
            % -------------------------------------------------------------
            % Obtain velocity of scatterers: interpolate and derivate
            % -------------------------------------------------------------
            sctrs.vel = zeros(3,obj.Nscttr, obj.Nfr);
            for sctr_idx = 1:obj.Nscttr
                pm_i = sctrs.pos(:,sctr_idx,:);
                ppf=spline(obj.t_grid, pm_i);
                ppdf = ppf;
                ppdf.order=ppdf.order-1;    %  derivate the exponent
                ppdf.coefs=ppdf.coefs(:,1:end-1).*(ppdf.order:-1:1); % (x^2)' = 2*x
                vm_i = ppval(ppdf,obj.t_grid);
                sctrs.vel(:,sctr_idx,:) = vm_i;
            end
            % addint time and id:
            jnts.id = obj.id;
            jnts.t_grid = obj.t_grid;
            
            sctrs.id = obj.id;
            sctrs.t_grid = obj.t_grid;
            '';
        end
    end
end

