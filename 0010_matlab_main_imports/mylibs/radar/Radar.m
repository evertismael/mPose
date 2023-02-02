classdef Radar
    properties
        % --------------------------------------------------------
        % Position and Rotation:
        % ---------------------------------------------------------
        p0                  % position in GLOBAL coords (xyz).
        rdr_rot_ZYX_deg     % from global -> radar coordinates.
        G_rdr_glb       % transf. matrix: global -> radar coords.
        G_glb_rdr       % transf. matrix: radar -> global coords.
        
        % --------------------------------------------------------
        % Radar Configuration:
        % ---------------------------------------------------------
        rf      
        chirp   
        frame
        cube
        ifgen
    end
    
    methods
        function obj = Radar(rdr_p0, rdr_rot_ZYX_deg, rdrp)
            
            % --------------------------------------------------------
            % Position and Rotation:
            % ---------------------------------------------------------
            obj.p0 = rdr_p0;
            
            obj.rdr_rot_ZYX_deg = rdr_rot_ZYX_deg; % rot order: Z, Y, X
            tmp_R_rdr_glb = eul2rotm(deg2rad(rdr_rot_ZYX_deg),'ZYX');
            
            R = [tmp_R_rdr_glb, zeros(3,1); zeros(1,3), 1];
            T = [eye(3,3), -obj.p0; zeros(1,3), 1];
            obj.G_rdr_glb = R*T;
            
            R = obj.G_rdr_glb(1:3,1:3);
            T = obj.G_rdr_glb(1:3,4);
            obj.G_glb_rdr = [R.', -R.'*T; zeros(1,3), 1];
            
            
            % --------------------------------------------------------
            % Radar Configuration:
            % ---------------------------------------------------------
            obj.rf = rdrp.rf;
            obj.chirp = rdrp.chirp;
            obj.frame = rdrp.frame;
            obj.cube = rdrp.cube;
            
            
            % --------------------------------------------------------
            % compute antennas-position in global coords:
            % ---------------------------------------------------------
            obj.rf.rx_glb = batch_mul(obj.G_glb_rdr, obj.rf.rx_uvw);
            obj.rf.tx_glb = batch_mul(obj.G_glb_rdr, obj.rf.tx_uvw);
            '';
        end
        

        function p_rdr = glb2rdr_coords(obj, p_glb, hom)
            % convert from global coordinates to radar coordinates:
            if size(p_glb,1)==3
                sz = size(p_glb);
                sz(1) = 1;
                p_glb = cat(1,p_glb,ones(sz));
            end
            p_rdr = batch_mul(obj.G_rdr_glb, p_glb);
            

            if strcmp(hom, 'not-homogen')
                colons = repmat({':'},1,ndims(p_rdr)-1);
                p_rdr = p_rdr(1:3, colons{:});
            elseif strcmp(hom, 'homogen')
                p_rdr = p_rdr;
            else
                error('spec homogeneous or not');
            end
        end
        
        function p_glb = rdr2glb_coords(obj, p_rdr, hom)
            % convert from global coordinates to radar coordinates:
            if size(p_rdr,1)==3
                p_rdr = cat(1,p_rdr,ones(1,size(p_rdr,2)));
            end
            p_glb = batch_mul(obj.G_glb_rdr, p_rdr);

            if strcmp(hom, 'not-homogen')
                colons = repmat({':'},1,ndims(p_glb)-1);
                p_glb = p_glb(1:3, colons{:});
            elseif strcmp(hom, 'homogen')
                p_glb = p_glb;
            else
                error('spec homogeneous or not');
            end
        end

        function vel_glb = vel_rdr2glb_coords(obj, vel_rdr)
            % rotate velocity in radar coords to global:
            vel_glb = batch_mul(obj.G_glb_rdr(1:3,1:3), vel_rdr);
        end

        function vel_rdr = vel_glb2rdr_coords(obj, vel_glb)
            % rotate velocity in global coords to radar:
            vel_rdr = batch_mul(obj.G_rdr_glb(1:3,1:3), vel_glb);
        end
    end
end

