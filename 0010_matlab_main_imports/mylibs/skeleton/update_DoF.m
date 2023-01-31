function skel = update_DoF(skel, idx, channels)
    a = skel.tree(idx);
    if isempty(a.parent) || a.parent==0 % root node
        % get DOF: rotation
        eul_degres = zeros(1, 3);
        for j = 1:length(a.rotInd)
          rind = a.rotInd(j);
          if rind
            eul_degres(j) = channels(rind);
          end
        end

        Rcn = rotationMatrix(deg2rad(-eul_degres(1)), ...
                              deg2rad(-eul_degres(2)), ...
                              deg2rad(-eul_degres(3)), ...
                              'xyz');
                              %a.order);
        a.Dcn = [Rcn, zeros(3,1); zeros(1,3), 1];
        % get DOF: translation:
        tn = zeros(3,1);
        for i = 1:length(a.posInd)
          pind = a.posInd(i);
          if pind
            tn(i) = channels(pind);
          end
        end

        a.Dtn = [eye(3,3), tn; zeros(1,3), 1];
        
        % with Dcn and Dtn compute the global transformation matrix:
        a.Gg_cn = a.Gnm1_rn*a.Dtn*a.Grn_n*a.Dcn;
        '';
    else
        % get DOF: rotation
        eul_degres = zeros(1, 3);
        for j = 1:length(a.rotInd)
          rind = a.rotInd(j);
          if rind
            eul_degres(j) = channels(rind);
          end
        end
        Rcn = rotationMatrix(deg2rad(-eul_degres(1)), ...
                      deg2rad(-eul_degres(2)), ...
                      deg2rad(-eul_degres(3)), ...
                      'xyz');
                      %a.order);

        a.Dcn = [Rcn, zeros(3,1); zeros(1,3), 1];
        
        % get DOF: translation: TODO: include dilation of joints:
        a.Dtn = [eye(3,4); zeros(1,3), 1];

        % with Dcn and Dtn compute the global transformation matrix:
        parent = skel.tree(a.parent);
        a.Gg_cn = parent.Gg_cn*a.Gnm1_rn*a.Dtn*a.Grn_n*a.Dcn;
        '';
    end
    
    skel.tree(idx).Dcn = a.Dcn;
    skel.tree(idx).Dtn = a.Dtn;
    skel.tree(idx).Gg_cn = a.Gg_cn;
    

    % call for children:
    for ch_idx = 1:size(a.children,2)
        [skel] = update_DoF(skel, a.children(ch_idx), channels);
    end
end