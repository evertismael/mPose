function axes = get_axes(skel)
    axes_n = [zeros(3,1), eye(3,3); ones(1,4)];
    axes = zeros(4,4,size(skel.tree,2));
    for a_idx = 1:size(skel.tree,2)
        a = skel.tree(a_idx);
        axes_g = zeros(size(axes_n));
        for p_idx = 1:4
            axes_g(:,p_idx) = a.Gg_cn*axes_n(:,p_idx);
        end
        axes(:,:,a_idx) = axes_g;
    end
end