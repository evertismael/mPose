function ifsig_frame = IF_signals(los_interp, fom_interp, som_interp, rdr)
% inputs:
%  path_interp: interpolated data at rdr_adc_rate.
%  rdr: radar for which communicate the if-signals.
%  opts: additional options. e.g. multipath generation.


% -----------------------------------------------------------------
% Concatenate wm, phim, a_b for: los, fom and som
% -----------------------------------------------------------------
wm = los_interp.wm;
phim = los_interp.phim;
a_b = los_interp.a_b;
if ~isempty(fom_interp) && ~isempty(som_interp)
    wm = cat(2,wm,fom_interp.wm);
    phim = cat(2,phim,fom_interp.phim);
    a_b = cat(2,a_b,fom_interp.a_b .* fom_interp.reflex_coef.*0.5);

    wm = cat(2,wm,som_interp.wm);
    phim = cat(2,phim,som_interp.phim);
    a_b = cat(2,a_b,som_interp.a_b .* som_interp.reflex_coef.^2 .*0.8);
end

% -----------------------------------------------------------------
% Start generating the IF signal: Sample by sample:
% -----------------------------------------------------------------
Nchirps = (rdr.frame.Nsamp_doppler*size(rdr.frame.tx_seq,2));
arg_t = nan(rdr.rf.Nrx, size(wm,2), rdr.chirp.Nsamp_range, Nchirps);
for ch_i = 1:Nchirps
    for k = 1:rdr.chirp.Nsamp_range
        if k == 1
            arg_t(:,:,k,ch_i) = 0;
        else
            arg_t(:,:,k,ch_i) = arg_t(:,:,k-1,ch_i) + wm(:,:,k-1,ch_i)*rdr.rf.dt;
        end
    end
    '';
end

% ---------------------------------------------------------------------
ifsig_frame = a_b.*exp(1j*(phim + arg_t));
%ifsig_frame = exp(1j*(phim + arg_t));

% final dim: fast, chirp, antena, frame, scattr,
ifsig_frame = permute(ifsig_frame, [3,4,1,5,2]);

ifsig_frame = sum(ifsig_frame,5);
'';
end


