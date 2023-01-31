clear all; clc; close all;
addpath(genpath('./matlab'))
addpath(genpath('./ndlutil-master'))

fname_amc = './16_18.amc';
fname_asf = './16.asf';
skel_asf = acclaimReadSkel(fname_asf);
[channels, skel] = acclaimLoadChannels(fname_amc, skel_asf);


% -------------------------------------------------------------------------
% get trajectories of joints:
% -------------------------------------------------------------------------
jnts_xyz = zeros(3,31,size(channels,1));
for fr_idx = 1:size(channels, 1)
    tmp_xyz = skel2xyz(skel, channels(fr_idx,:)); 
    tmp_xyz = tmp_xyz*(1/0.45)*2.54/100.0; % conversion from inches and other factors.
    tmp_xyz = tmp_xyz(:,[1,3,2]).';
    jnts_xyz(:,:,fr_idx) = tmp_xyz;
end


% -------------------------------------------------------------------------
% animate:
% -------------------------------------------------------------------------
figure;
for fr_idx = 1:size(channels, 1)
    if fr_idx==1
        h = plot3(jnts_xyz(1,:,fr_idx),jnts_xyz(2,:,fr_idx),jnts_xyz(3,:,fr_idx),'.');
        axs_max = max(jnts_xyz,[],[2,3]);
        axs_min = min(jnts_xyz,[],[2,3]);
        axis equal;
        xlim([axs_min(1), axs_max(1)]);
        ylim([axs_min(2), axs_max(2)]);
        zlim([axs_min(3), axs_max(3)]);
        hold on;
    else
        set(h,'Xdata', jnts_xyz(1,:,fr_idx), ...
              'Ydata', jnts_xyz(2,:,fr_idx), ...
              'Zdata', jnts_xyz(3,:,fr_idx));
    end
    pause(1/120);
end
'';