% it first computes data: (jnts, pm, abc bones, etc.) for all frames in fr_idxs
% Then it interpolates all values to t_frame.
% the interpolated data into ibody.
function [ibody, ref_body] = create_interpolated_mbody(mbody, fr_idxs, t_frame)
    
    % ---------------------------------------------------------------------
    % Collect all required data for frames fr_idxs
    % ---------------------------------------------------------------------
    [jnts, ~] = mbody.get_joints_by_frame(fr_idxs(1));
    [pm, abc, Ggcorr_elps, Ggcorr_skel_jnt] = mbody.get_bones_by_frame(fr_idxs(1));
    Nfr = size(fr_idxs,2);
    for i = 2:Nfr
        [tmp_jnts, ~] = mbody.get_joints_by_frame(fr_idxs(i));
        [tmp_pm, tmp_abc, tmp_Ggcorr_elps, tmp_Ggcorr_skel_jnt] = mbody.get_bones_by_frame(fr_idxs(i));

        jnts = cat(3,jnts,tmp_jnts);
        pm = cat(3,pm,tmp_pm);
        abc = cat(3,abc,tmp_abc);
        Ggcorr_elps = cat(4,Ggcorr_elps,tmp_Ggcorr_elps);
        Ggcorr_skel_jnt = cat(4,Ggcorr_skel_jnt,tmp_Ggcorr_skel_jnt);
        '';
    end
    channels = mbody.channels(:,fr_idxs);

    % ---------------------------------------------------------------------
    % pack all data into a single vector to interpolate at once:
    % ---------------------------------------------------------------------
    
    % bring frame-dim to first position:
    jnts = permute(jnts, [3,1,2]); % fr, xyz, jnts
    pm = permute(pm, [3,1,2]); % fr, xyz, bone
    abc = permute(abc, [3,1,2]); % fr, xyz, bone
    Ggcorr_elps = permute(Ggcorr_elps, [4,1,2,3]); % fr, rotx, roty, bone
    Ggcorr_skel_jnt = permute(Ggcorr_skel_jnt, [4,1,2,3]); % fr, rotx, roty, bone
    channels = permute(channels, [2,1]); % fr, chan
    
    % keep the sizes:
    sz_jnts = size(jnts);
    sz_pm = size(pm);
    sz_abc = size(abc);
    sz_Ggcorr_elps = size(Ggcorr_elps);
    sz_Ggcorr_skel_jnt = size(Ggcorr_skel_jnt);
    sz_channels = size(channels);

    v = double.empty(Nfr,0);
    v = cat(2,v, reshape(jnts, [Nfr, prod(sz_jnts(2:end))]));
    v = cat(2,v, reshape(pm, [Nfr, prod(sz_pm(2:end))]));
    v = cat(2,v, reshape(abc, [Nfr, prod(sz_abc(2:end))]));
    v = cat(2,v, reshape(Ggcorr_elps, [Nfr, prod(sz_Ggcorr_elps(2:end))]));
    v = cat(2,v, reshape(Ggcorr_skel_jnt, [Nfr, prod(sz_Ggcorr_skel_jnt(2:end))]));
    v = cat(2,v, reshape(channels, [Nfr, prod(sz_channels(2:end))]));
    
    % ---------------------------------------------------------------------
    % Interpolate
    % ---------------------------------------------------------------------
    t_v = mbody.t_grid(fr_idxs).';
    v_interp = interp1(t_v, v, t_frame.','spline');
    
    
    % ---------------------------------------------------------------------
    % prepare to unpack
    % ---------------------------------------------------------------------
    Nt_frame = size(v_interp,1);
    sz_jnts(1) = Nt_frame;
    sz_pm(1) = Nt_frame;
    sz_abc(1) = Nt_frame;
    sz_Ggcorr_elps(1) = Nt_frame;
    sz_Ggcorr_skel_jnt(1) = Nt_frame;
    sz_channels(1) = Nt_frame;
    
    % ---------------------------------------------------------------------
    % unpack
    % ---------------------------------------------------------------------
    ibody = {};
    ibody.id = mbody.id;
    v_init = 0;
    v_sel = v_init + (1:prod(sz_jnts(2:end)));
    ibody.jnts = reshape(v_interp(:, v_sel ), sz_jnts);

    v_init = v_sel(end);
    v_sel = v_init + (1:prod(sz_pm(2:end)));
    ibody.pm = reshape(v_interp(:, v_sel), sz_pm);

    v_init = v_sel(end);
    v_sel = v_init + (1:prod(sz_abc(2:end)));
    ibody.abc = reshape(v_interp(:, v_sel), sz_abc);

    v_init = v_sel(end);
    v_sel = v_init + (1:prod(sz_Ggcorr_elps(2:end)));
    ibody.Ggcorr_elps = reshape(v_interp(:, v_sel), sz_Ggcorr_elps);

    v_init = v_sel(end);
    v_sel = v_init + (1:prod(sz_Ggcorr_skel_jnt(2:end)));
    ibody.Ggcorr_skel_jnt = reshape(v_interp(:, v_sel), sz_Ggcorr_skel_jnt);

    v_init = v_sel(end);
    v_sel = v_init + (1:prod(sz_channels(2:end)));
    ibody.channels = reshape(v_interp(:, v_sel), sz_channels);


    % ---------------------------------------------------------------------
    % put frame dim to the last position:
    % ---------------------------------------------------------------------
    ibody.jnts = permute(ibody.jnts, [2,3,1]); % xyz, jnts, fr
    ibody.pm = permute(ibody.pm, [2,3,1]); % xyz, bone, fr
    ibody.abc = permute(ibody.abc, [2,3,1]); % xyz, bone, fr
    ibody.Ggcorr_elps = permute(ibody.Ggcorr_elps, [2,3,4,1]); % rotx, roty, bone, fr
    ibody.Ggcorr_skel_jnt = permute(ibody.Ggcorr_skel_jnt, [2,3,4,1]); % rotx, roty, bone, fr
    ibody.channels = permute(ibody.channels, [2,1]); % chan, fr

    % just for reference:
    ref_body.jnts = permute(jnts, [2,3,1]); % xyz, jnts, fr
    ref_body.pm = permute(pm, [2,3,1]); % xyz, bone, fr
    ref_body.abc = permute(abc, [2,3,1]); % xyz, bone, fr
    ref_body.Ggcorr_elps = permute(Ggcorr_elps, [2,3,4,1]); % rotx, roty, bone, fr
    ref_body.Ggcorr_skel_jnt = permute(Ggcorr_skel_jnt, [2,3,4,1]); % rotx, roty, bone, fr
    ref_body.channels = permute(channels, [2,1]); % chan, fr
    '';
end