function skel = compute_G_all(skel, idx)
    a = skel.tree(idx);
    if isempty(a.parent) || a.parent==0 % root node
        % Gg_c1 = Gg_r1 * D^(T1) *G_r1_1 * D^(c1);
        Tg = a.offset.';
        eul_deg = a.orientation; % r1 -> 1
        
        A1_r1 = rotationMatrix(deg2rad(eul_deg(1)), ...
                               deg2rad(eul_deg(2)), ...
                               deg2rad(eul_deg(3)), ...
                               a.axisOrder);
        a.An_rn = A1_r1;
        a.Gnm1_rn = [eye(3,3), Tg; zeros(1,3), 1];
        a.Grn_n = [a.An_rn.', zeros(3,1);  zeros(1,3), 1];
        
        % init DOF rotation and translation:
        a.Dcn = [eye(3,4); zeros(1,3), 1];
        a.Dtn = [eye(3,4); zeros(1,3), 1];
        
        a.Gg_cn = a.Gnm1_rn*a.Grn_n;
    else % chil:
        % gather info from parent:
        parent = skel.tree(a.parent);
        Trnm1 = parent.offset.';
        Anm1_rnm1 = parent.An_rn;

        % info for axis:
        eul_deg = a.axis; % rn -> n
        An_rn = rotationMatrix(deg2rad(eul_deg(1)), ...
                               deg2rad(eul_deg(2)), ...
                               deg2rad(eul_deg(3)), ...
                               a.axisOrder);
        
        a.An_rn = An_rn;
        a.Gnm1_rn = [Anm1_rnm1, Anm1_rnm1*Trnm1; zeros(1,3), 1];
        a.Grn_n = [a.An_rn.', zeros(3,1);  zeros(1,3), 1];
        
        % init DOF rotation:
        a.Dcn = [eye(3,4); zeros(1,3), 1];
        a.Dtn = [eye(3,4); zeros(1,3), 1];

        a.Gg_cn = parent.Gg_cn*a.Gnm1_rn*a.Grn_n;
        '';
    end

    % save:
    skel.tree(idx).An_rn = a.An_rn;
    skel.tree(idx).Gnm1_rn = a.Gnm1_rn;
    skel.tree(idx).Grn_n = a.Grn_n;
    skel.tree(idx).Dcn = a.Dcn;
    skel.tree(idx).Dtn = a.Dtn;
    skel.tree(idx).Gg_cn = a.Gg_cn;
    
    % call for children:
    for ch_idx = 1:size(a.children,2)
        skel = compute_G_all(skel, a.children(ch_idx));
    end

end