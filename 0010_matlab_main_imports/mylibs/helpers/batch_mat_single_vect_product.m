function Mv_batch = batch_mat_single_vect_product(Mbatch, v)
    assert(size(Mbatch,1)==size(Mbatch,2),'It only works for square matrices');
    assert(size(Mbatch,1)==size(v,1),'M,b not conformable');
    
    m = size(Mbatch,1);
    sz_M = size(Mbatch);
    sz_Mv = [sz_M(1), sz_M(3:end)];

    Mv_batch = zeros(sz_Mv);

    colons = repmat({':'},1, ndims(Mv_batch)-1);
    for row_i = 1:m
        tmp = Mbatch(row_i,:,colons{:});
        tmp_T = permute(tmp,[2:(size(sz_Mv,2)+1), 1]);
        Mv_batch(row_i, colons{:}) = sum(tmp_T.*v,1);
    end
end