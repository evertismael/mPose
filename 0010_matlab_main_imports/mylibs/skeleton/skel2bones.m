function [pm_t, Ggcorr_skel_jnt_t, abc_t, Ggcorr_elps_t] = skel2bones(skel)

    Njnts = size(skel.tree,2);
    pm_t = double.empty(4,0);
    Ggcorr_skel_jnt_t = zeros(4,4,Njnts);
    
    abc_t = double.empty(3,0); % abc of ellipses
    Ggcorr_elps_t = double.empty(4,4,0);

    XYZ_FACTOR = (1/0.45)*2.54/100.0;
    for a_idx = 2:size(skel.tree,2)
        a = skel.tree(a_idx);

        % determine number of scatterers:
        norm_pm = sqrt(sum((a.offset.').^2,1))*XYZ_FACTOR;
        Nscatter = floor(norm_pm/skel.scatter_res);
        if Nscatter ==0
            Nscatter = 1;
        end

        % get center scatterers:
        scatter_idx = (0.5:1:Nscatter)/Nscatter;
        pm_rn = [scatter_idx.*a.offset.';ones(1,Nscatter)];

        % Determine scatteres in global coords:
        Rrn_n = a.Grn_n(1:3,1:3);
        Trn = a.Grn_n(1:3,4);
        Gn_rn = [Rrn_n.', Rrn_n.'*Trn; zeros(1,3), 1];
        Gg_rn = a.Gg_cn*Gn_rn;
        Ggcorr_rn = skel.G_corr*Gg_rn;

        tmp_pm_t = batch_mul(Ggcorr_rn,pm_rn);
        pm_t = cat(2, pm_t, tmp_pm_t);

        % get rot: skel->global:
        Ggcorr_skel_jnt_t(:,:,a_idx) = skel.G_corr*a.Gg_cn;

        % Determine Rot from elipsoid to Global; (all scatters in one bone share the same axis orientation)
        for sctr_i = 1:Nscatter
            y_n = [0,1,0].';
            v_n = Gn_rn*pm_rn(:,sctr_i);
            v_n = v_n([1,2,3]);
            w_n = cross(y_n,v_n);
            if ~all(w_n==0)
                w_n = w_n./(sqrt(sum(w_n.^2)));
                theta_rad_elps_n = atan2(norm(cross(y_n,v_n)), dot(y_n,v_n));
                theta_deg_elps_n = rad2deg(theta_rad_elps_n);
                R_elps_n = exp_rot(-theta_deg_elps_n, w_n);
                '';
            else
                R_elps_n = eye(3,3);
            end
            
            '';
            % get rot: elip -> global:
            G_n_elps = [R_elps_n.' v_n(1:3); zeros(1,3), 1];
            tmp_Ggcorr_elps = skel.G_corr*a.Gg_cn*G_n_elps;
            Ggcorr_elps_t = cat(3,Ggcorr_elps_t,tmp_Ggcorr_elps);

            % Determine the lenghts of mayor/minor axes in ellipsoid:
            width = skel.mapBones2Width(a.name);
            
            v_elps = R_elps_n*v_n; % mayor axis vector / Nscatter:
            assert(abs(v_elps(2))==sqrt(sum(v_elps.^2,'all')));
            
            abc = width*ones(1,3);
            abc(v_elps>1e-3) = sqrt(sum((a.offset.').^2,1))/(Nscatter*2); %v_elps(v_elps>1e-3);
            abc_t = cat(2,abc_t, abc.');
            '';
        end
    end
    '';
end