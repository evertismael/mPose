function [rm1o, vm1o, azm1o, elm1o, tm1o_grid, idm1o] = ...
    simple_first_order_multipath(rm, vm, azm, elm, tm_grid, idm, bodies)
    
    % First order multipaths:
    [rm1o, vm1o, azm1o, elm1o, tm1o_grid, idm1o] = deal({});
    for m_i = 2:size(rm,2)
        % ---------------------------------------------------------------------
        % type A: rdr-body-floor-rdr
        % ---------------------------------------------------------------------
        idx_A = (m_i-2)*2 +1;
        rm1o{idx_A} = rm{1}/2 + rm{m_i}/2;
        vm1o{idx_A} = vm{1};
        azm1o{idx_A} = azm{m_i};
        elm1o{idx_A} = elm{m_i};
        tm1o_grid{idx_A} = tm_grid{1};
        idm1o{idx_A} = size(bodies,2) + idx_A;

        % ---------------------------------------------------------------------
        % type B: rdr-floor-body-rdr
        % ---------------------------------------------------------------------
        idx_B = (m_i-2)*2 + 2;
        rm1o{idx_B} = rm{1}/2 + rm{m_i}/2;
        vm1o{idx_B} = vm{m_i};
        azm1o{idx_B} = azm{1};
        elm1o{idx_B} = elm{1};
        tm1o_grid{idx_B} = tm_grid{1};
        idm1o{idx_B} = size(bodies,2) + idx_B;
    end
    '';
end