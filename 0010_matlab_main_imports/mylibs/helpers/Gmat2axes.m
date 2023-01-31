function [a2p_x, a2p_y, a2p_z] = Gmat2axes(G_glb_body, scale)
    axes_body = [zeros(3,1), scale*eye(3,3); ones(1,4)];

    Nsys_coords = size(G_glb_body,3);
    axes_glb = nan(4,5,Nsys_coords);
    for sc_i=1:Nsys_coords
        G_sys_1 = G_glb_body(:,:,sc_i);
        axes_glb(:,1:4, sc_i) = batch_mul(G_sys_1, axes_body);
    end
    
    % format for plotting:
    a2p = axes_glb(:,[1 2 5, 1 3 5, 1 4 5],:);
    a2p_x = squeeze(a2p(:,[1,2,3],:));
    a2p_y = squeeze(a2p(:,[4,5,6],:));
    a2p_z = squeeze(a2p(:,[7,8,9],:));
end