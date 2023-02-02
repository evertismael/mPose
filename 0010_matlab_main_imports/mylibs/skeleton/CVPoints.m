classdef CVPoints
    % Constant Velocity Points
    % bones: (pm,Am,abc, rot) -> center, gain, mayor/minor axes, rotation.

    properties
        Nfrm
        t_grid
        
        jnts_xyz

        bones_pm
        bones_Am
        bones_abc
        bones_rot
        bones_names
    end
    
    methods
        function obj = CVPoints(Npoints, t_sim)
            Npoints =  min([Npoints,2]);
            
            xyz_p0 = [ 0  0;
                       1  0;
                       0  0];
            xyz_dot_p0 = [5 -4; 
                          0   0;
                          0   0]; 
            
            
            p0 = xyz_p0(:,1:Npoints);
            v0 = xyz_dot_p0(:,1:Npoints);
            % generate trajectory:
            dt = 0.01;
            t_grid = 0:dt:(t_sim-dt);
            t_grid = permute(t_grid,[1,3,2]);
            pm = p0 + v0.*t_grid;
            
            % -------------------------------------------------------------
            % extract bones:
            obj.Nfrm = size(pm,3);
            
            obj.bones_pm = permute(pm, [1,3,2]);
            obj.bones_abc = NaN;
            obj.bones_rot = NaN;
            obj.bones_Am = NaN; % TODO.
            obj.bones_names = NaN;
            % -------------------------------------------------------------
            % time:
            obj.t_grid = (0:(obj.Nfrm-1))*(dt);  
            
            %
            obj.jnts_xyz = obj.bones_pm;
        end
        
    end
end

