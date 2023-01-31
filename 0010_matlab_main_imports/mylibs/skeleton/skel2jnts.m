function [jnts, jnts_name] = skel2jnts(skel)
    % jnts are the offset vector (Other extreme of each bone:)
    jnts = zeros(4,31);
    jnts_name = {};
    for a_idx = 1:size(skel.tree,2)
        a = skel.tree(a_idx);

        % get joints:
        jnt_rn = [a.offset.';1];
        Rrn_n = a.Grn_n(1:3,1:3);
        Trn = a.Grn_n(1:3,4);
        Gn_rn = [Rrn_n.', Rrn_n.'*Trn; zeros(1,3), 1];
        % jnts(:,a_idx) = a.Gg_cn*Gn_rn*jnt_rn;
        jnts(:,a_idx) = skel.G_corr*a.Gg_cn*Gn_rn*jnt_rn;
        jnts_name{a_idx} = a.name;
        
    end
    jnts = jnts(1:3,:);
end