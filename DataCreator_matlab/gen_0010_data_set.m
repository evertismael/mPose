clear all; clc; close all;
% it plots the scene: radar and skeleton (first and last frames)
% -------------------------------------------------------------------------
clear all; close all; clc;
addpath(genpath('../0010_matlab_main_imports'));
addpath(genpath('./functions'));

glbp = glb_prms();

% folder with mocap data:
dataset_path = '../0005_cmu_mocap_dataset/';

% read dataset_file:
f_name = 'datasetwalk';
%f_name = 'datasetwave_signals';
%f_name = 'datasetwalk_slow';

f_path = '../../0020_Dataset_selector/';
M = readtable([f_path, f_name, '.csv']);
M(1:3,:)

% -------------------------------------------------------------------------
% Radar:
% -------------------------------------------------------------------------
[Ntx_enable] = deal([1 0 0]);
tx_seq = [1];
fc = 77e9;
rdrp = burst_prms(fc, Ntx_enable, tx_seq);
[rdr_p0, rdr_rot_ZYX_deg] = deal([0, -3,  0.8].', [0, 0, 0]); 
rdr = Radar(rdr_p0, rdr_rot_ZYX_deg, rdrp);

% ------------------------------------
for batch_idx = 1:10
    batch_size = 4;
    [m_start, m_end] = deal(1+(batch_idx-1)*batch_size, batch_idx*batch_size);
    
    % --------------------------------------
    % output matrices: initialization:
    n_tpls = 0;
    roi_grids_all = double.empty(80,80,3,0);
    rm_azm_all = double.empty(80,80,0);
    rm_elevm_all = double.empty(80,80,0);

    jnts_xyz_all = double.empty(3,31,0);
    torso_xyz_all = double.empty(3,0);
    
    for idx = m_start:m_end
        [subject,trial] = deal(M{idx,['subject']}, M{idx,['trial']});
        description = M{idx,['description']}{1};
        
        subject = pad(num2str(subject),2,'left','0');
        trial = pad(num2str(trial),2,'left','0');
        fprintf('%d:  %s , %s, %s \n', idx, subject, trial, description)
        
        % ---------------------------------------------------------------------
        % 0010: Visualize the chosen data:
        % ---------------------------------------------------------------------
        mbody = MocapBody(dataset_path, subject, trial);
        mbody.id = 1;

        mid = str2double(subject);
        [x_lim, y_lim, z_lim] = deal([-2, 2], [-3, 3], [0, 2]);
        scnMon = SceneMonitor(mid, x_lim, y_lim, z_lim);
        scnMon = scnMon.draw_radar(rdr);
        scnMon = scnMon.draw_body(mbody);
        scnMon = scnMon.draw_bone_ellipsoids(mbody);

        % animate:
        for fr_idx = 1:10:size(mbody.t_grid,2)
            scnMon = scnMon.update_body(mbody, fr_idx);
            scnMon = scnMon.update_bone_ellipsoids(mbody, fr_idx);
            pause(0.01);
        end
        
        % ---------------------------------------------------------------------
        % 0020: Generate mPose images:
        % ---------------------------------------------------------------------
        [fr_init,fr_end] = deal(1,300);
        flag_noise = false;
        [rm, vm, azm, elm, jntsm, bnm_cntr, tm_grid] = simple_measurements(mbody, rdr, flag_noise, fr_init, fr_end);

        % decimate to reduce the data generated:
        fr_sel_idx = 1:10:size(tm_grid,2);
        [rm, azm, elm, jntsm, bnm_cntr] = deal(rm(:,fr_sel_idx), ...
            azm(:,fr_sel_idx), elm(:,fr_sel_idx), jntsm(:,:,fr_sel_idx),bnm_cntr(:,:,fr_sel_idx));
        
        '';
        figure;
        subplot(2,2,1); plot(rm.');
        subplot(2,2,2); plot(vm.');
        subplot(2,2,3); plot(azm.');
        subplot(2,2,4); plot(elm.');
        '';
        
        % -----------------------------------------------------------------
        % create Range angle image:
        % -----------------------------------------------------------------
        show_images = true;
        [rm_azm, rm_elevm, roi_grids, torso_xyz] = range_angles_images(...
                                        rm, azm, elm, jntsm, rdr, show_images);
     
        n_tpls = n_tpls + size(fr_sel_idx,2);  
        % -----------------------------------------------------------------
        % concatenate output data:
        % -----------------------------------------------------------------
        rm_azm_all = cat(3,rm_azm_all,rm_azm);
        rm_elevm_all = cat(3,rm_elevm_all,rm_elevm);
        roi_grids_all = cat(4,roi_grids_all,roi_grids);
        jnts_xyz_all = cat(3,jnts_xyz_all,jntsm);
        torso_xyz_all = cat(2,torso_xyz_all,torso_xyz);
        '';
    end

    disp(n_tpls)
    fout_name = ['./dataset/',f_name,'_batch_',num2str(m_start),'_',num2str(m_end),'.mat'];
    save(fout_name,'rm_azm_all','rm_elevm_all','roi_grids_all','jnts_xyz_all','torso_xyz_all');
end
