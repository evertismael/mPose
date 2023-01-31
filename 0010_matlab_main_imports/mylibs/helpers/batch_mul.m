% It multiplies a matrix M: m x m;
% by another group of col-vectors: m x dim1 ,dim2,dim3...
% output Mb = m x dim1, dim2
function Mb = batch_mul(M, batch_vector)
    assert(size(M,1)==size(M,2),'It only works for square matrices');
    assert(size(M,1)==size(batch_vector,1),'M,b not conformable');

    m = size(M,1);
    Mb = zeros(size(batch_vector));

    colons = repmat({':'},1,ndims(batch_vector)-1);
    for row_i = 1:m
        Mb(row_i, colons{:}) = sum(M(row_i,:).'.*batch_vector,1);
    end
end