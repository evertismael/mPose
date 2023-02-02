function [G_c_f] = get_G_ceiling_floor(h, mode)
    if strcmp(mode, 'ceiling')
        G_c_f = [1 0 0   0;
                 0 1 0   0;
                 0 0 -1 2*h;
                 0 0 0   1];
    elseif strcmp(mode, 'floor')
        G_c_f = [1 0 0  0;
                0 1 0  0;
                0 0 -1 -2*h;
                0 0 0  1];
    else 
        error('mode not acccepted')
    end
end

