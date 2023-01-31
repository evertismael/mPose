function if_cubes = if_cube_separate_per_tx(if_cube_all_tx, rdr)
    Ntx_seq = size(rdr.frame.tx_seq,2);
    if_cubes = repmat({},Ntx_seq);

    Nchirps = size(if_cube_all_tx,2);
    for c_i = 1:Ntx_seq
        if_cubes{c_i} = if_cube_all_tx(:,c_i:Ntx_seq:Nchirps,:,:);
    end
end