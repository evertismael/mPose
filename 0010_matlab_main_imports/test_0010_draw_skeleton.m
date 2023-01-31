clear all; close all; clc;
addpath(genpath('../0010_matlab_main_imports_beta/'));
dataset_path = '../0005_cmu_mocap_dataset/';

% -------------------------------------------------------------------------
% Radar:
% -------------------------------------------------------------------------
[Nrx_enable, Ntx_enable] = deal([1 1 1 1], [1 0 0]);
Ntx_seq = [1];
fc = 77e9;
rdrp = burst_prms(fc, Nrx_enable, Ntx_enable, Ntx_seq);

[rdr_p0, rdr_rot_ZYX_deg] = deal([0, 3,  1].', [0, 0, 0]); 
rdr = Radar(rdr_p0, rdr_rot_ZYX_deg, rdrp);

% -------------------------------------------------------------------------
% mbody:
% -------------------------------------------------------------------------
[subject, trial ] = deal('05','01');
mbody = MocapBody(dataset_path, subject, trial);
mbody.id = 1;
[rm, vm, azm, elm, jntsm, bnm_cntr] = simple_measurements(mbody, rdr);

figure;
subplot(2,2,1); plot(rm.'); xlabel('time'); ylabel('range');
subplot(2,2,2); plot(vm.'); xlabel('time'); ylabel('vel');
subplot(2,2,3); plot(azm.'); xlabel('time'); ylabel('azim');
subplot(2,2,4); plot(elm.'); xlabel('time'); ylabel('elev');


% -------------------------------------------------------------------------
% Animate Scene:
% -------------------------------------------------------------------------
idx = str2double(subject);
[x_lim, y_lim, z_lim] = deal([-2, 2], [-3, 3], [0, 2]);

scnMon = SceneMonitor(idx, x_lim, y_lim, z_lim);
scnMon = scnMon.draw_radar(rdr);
scnMon = scnMon.draw_body(mbody);
% animate:
for t_idx = 1:10:size(mbody.t_grid,2)
    scnMon = scnMon.update_body(mbody, t_idx);
    pause(0.01);
end
scnMon = scnMon.draw_trajectory(mbody);
%%
% draw names of joints:
figure;
a = squeeze(mbody.jnts_xyz(:,:,1));
b = mbody.pm(:,:,1);
plot3(a(1,:),a(2,:),a(3,:),'.'); 
text(a(1,:),a(2,:),a(3,:),mbody.names,'HorizontalAlignment','left','FontSize',6);

axis 'equal'; hold on;
plot3(b(1,:),b(2,:),b(3,:),'+');
