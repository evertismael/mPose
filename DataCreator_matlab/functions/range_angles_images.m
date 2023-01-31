function [rm_azm, rm_elm, roi_grids, torso_xyz_all] = range_angles_images(rm, azm, elm, jnts_xyz, rdr, show_fig)
    
    % convert to degrees:
    azm = rad2deg(azm);
    elm = rad2deg(elm);

    % define grid params:
    [r_min,r_max, r_dres] = deal(0,8,0.03);
    r_rng = r_min:r_dres:r_max;
    Nr = size(r_rng,2);

    [az_min, az_max, az_dres] = deal(-90,90,0.5);
    az_rng = az_min: az_dres: az_max;
    Naz = size(az_rng,2);

    [el_min, el_max, el_dres] = deal(-80,80,0.5);
    el_rng = el_min: el_dres: el_max;
    Nel = size(el_rng,2);
    
    % all r_ze_points:
    r_rng = r_rng.';
    [R,AZ] = meshgrid(r_rng,az_rng);
    r_az = [R(:) AZ(:)];

    [R,EL] = meshgrid(r_rng,el_rng);
    r_el = [R(:) EL(:)];


    
    Sigma = diag([0.01,0.5]);
    
    % construct images:
    if show_fig
        f = figure;
    end


    Nt = size(rm,2);
    
    % body arm/height:
    arm = 1.5;
    height = 2.2;
    scale = [80,80];
    rm_azm = zeros(80,80,Nt);
    rm_elm = zeros(80,80,Nt);
    torso_xyz_all = zeros(3,Nt);
    roi_grids = zeros(80,80,3,Nt);

    for t_idx = 1:Nt
        tmp_az_r = zeros(Naz, Nr);
        tmp_el_r = zeros(Nel, Nr);
        
        % -------------------------------------------------------
        % Construct the full images: rm (scatter, time)
        % -------------------------------------------------------
        for sctr_idx = 1:size(rm,1)
            % azimuth
            mu = [rm(sctr_idx,t_idx), azm(sctr_idx,t_idx)];
            y_i = mvnpdf(r_az,mu,Sigma);
            y_i = reshape(y_i,Naz,Nr);
            tmp_az_r = tmp_az_r + y_i;
            
            % elevation:
            mu = [rm(sctr_idx,t_idx), elm(sctr_idx,t_idx)];
            y_i = mvnpdf(r_el,mu,Sigma);
            y_i = reshape(y_i,Nel,Nr);
            tmp_el_r = tmp_el_r + y_i;
            '';
        end
        
        % -----------------------------------------------------------
        % extract ROI:
        % -----------------------------------------------------------
        % get torso range, azimuth, elevation: Center of ROI
        x0 = jnts_xyz(:,1,t_idx);
        torso_xyz_all(:,t_idx) = x0;

        % get boudaries of the boxes:
        uvw0 = rdr.glb2rdr_coords(x0, 'not-homogen');
        
        r0 = sqrt(sum((uvw0).^2,1));
        azm0 = rad2deg(atan2(uvw0(1),uvw0(2)));
        elm0 = rad2deg(asin(uvw0(3)/r0));
        dazm0 = rad2deg(atan(arm/(2*r0)));
        delm0 = rad2deg(asin((height-rdr.p0(3))/(r0)));

        % extract the image and rescale:
        r_idxs = abs(r_rng-r0)<(arm/2);
        az_idxs = abs(az_rng-azm0)<(dazm0);
        el_idxs = abs(el_rng-elm0)<delm0;
        
        roi_azm = tmp_az_r(az_idxs,r_idxs);
        roi_azm_out = imresize(roi_azm,scale);

        roi_elm = tmp_el_r(el_idxs,r_idxs);
        roi_elm_out = imresize(roi_elm,scale);
        
        % ----------------------------------------------------------
        %  ranges of r, az, el:
        % ----------------------------------------------------------
        roi_rgn_min = min(r_rng(r_idxs));
        roi_rgn_max = max(r_rng(r_idxs));
        roi_rng = linspace(roi_rgn_min,roi_rgn_max,80);

        roi_az_min = min(az_rng(az_idxs));
        roi_az_max = max(az_rng(az_idxs));
        roi_az = linspace(roi_az_min,roi_az_max,80);

        roi_el_min = min(el_rng(el_idxs));
        roi_el_max = max(el_rng(el_idxs));
        roi_el = linspace(roi_el_min,roi_el_max,80);
        
        roi_grids(:,:,1, t_idx) = repmat(roi_rng, [80, 1]);
        roi_grids(:,:,2, t_idx) = repmat(roi_az.', [1, 80]);
        roi_grids(:,:,3, t_idx) = repmat(roi_el.', [1, 80]);
        
        rm_azm(:,:,t_idx) = roi_azm_out;
        rm_elm(:,:,t_idx) = roi_elm_out;

        if show_fig
            figure(f)
            subplot(2,2,1);imagesc(r_rng,az_rng,tmp_az_r); title('range-azimuth');
            subplot(2,2,2);imagesc(r_rng,el_rng,tmp_el_r); title('range-elevation');
            subplot(2,2,3);imagesc(roi_azm_out); title('ram-ROI');
            subplot(2,2,4);imagesc(roi_elm_out); title('rem-ROI');
            pause(0.00005);
        end
        '';
    end
    '';
end


