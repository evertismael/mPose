% It multiplies a matrix G: m x m;
% by another group of matrices G_batch:  m x m, dim1 ,dim2,dim3...
% output G*G_batch = m x m x dim1, dim2, ...
function G_G_batch = batch_mul_gral(G, G_batch)
    assert(size(G,1)==size(G,2),'It only works for square matrices');
    assert(size(G,2)==size(G_batch,1),'G, G_batch not conformable');

    m = size(G,1);
    G_G_batch = zeros(size(G_batch));

    colons = repmat({':'},1,ndims(G_batch)-2);

    for row_i = 1:m
        for col_i = 1:m
            Gb_col = G_batch(:, col_i, colons{:});
            G_row_T = G(row_i,:).';
            G_G_batch(row_i, col_i, colons{:}) = sum(G_row_T.*Gb_col,1);
        end
    end
    
end