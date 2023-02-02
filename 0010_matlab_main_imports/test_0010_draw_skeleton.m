clear all; close all; clc;
addpath(genpath('../0010_matlab_main_imports/'));
dataset_path = '../0005_cmu_mocap_dataset/';

% -------------------------------------------------------------------------
% mbody: contains main information.
% sctrs: length, orientation, center of scatterers.
% -------------------------------------------------------------------------
[subject, trial ] = deal('05','01');
mbody = MocapBody(dataset_path, subject, trial);
mbody.id = 1;
[sctrs, jnts] = mbody.get_scatterers();
fprintf('Total simple Frames %d , total simple duration: %.2f sec \n', mbody.Nfr, mbody.t_grid(end));

% -------------------------------------------------------------------------
% Radar:
% -------------------------------------------------------------------------
[Ntx_enable, tx_seq] = deal([1 1 0], [1]);
rdrp = burst_prms(77e9, Ntx_enable, tx_seq);
[rdr_p0, rdr_rot_ZYX_deg] = deal([0, -3,  0.5].', [0, 0, 0]); 
rdr = Radar(rdr_p0, rdr_rot_ZYX_deg, rdrp);

%%
% -------------------------------------------------------------------------
% Plot Scene:
% -------------------------------------------------------------------------
idx = str2double(subject);
[x_lim, y_lim, z_lim] = deal([-2, 2], [-3, 3], [-3, 6]);
scnMon = SceneMonitor(idx, x_lim, y_lim, z_lim);
ceiling_height = 3;
scnMon = scnMon.draw_floor_ceiling(ceiling_height);
scnMon = scnMon.draw_radar(rdr);
scnMon = scnMon.draw_jnts(jnts);
scnMon = scnMon.draw_sctrs(sctrs);
% animate:
for fr_idx = 1:10:size(jnts.t_grid,2)
    scnMon = scnMon.update_jnts(jnts, fr_idx);
    scnMon = scnMon.update_sctrs(sctrs, fr_idx);
    pause(0.5);% pause(1/120);
end