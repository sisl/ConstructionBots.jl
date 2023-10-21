# module Arrays

export
    pad_matrix,
    cross_product_operator,
    one_hot,
    wrap_idx,
    wrap_get

"""
    helper to pad a matrix with some value around the edges
"""
function pad_matrix(mat::Matrix{T}, pad_size::Tuple{Int,Int},pad_val::T) where T
    new_size = [size(mat)...] + 2*[pad_size...]
    A = fill!(typeof(mat)(undef,new_size[1],new_size[2]),pad_val)
    A[
        pad_size[1]+1:pad_size[1]+size(mat,1),
        pad_size[2]+1:pad_size[2]+size(mat,2)
    ] .= mat
    A
end

cross_product_operator(x) = SMatrix{3,3}(
    [0.0    -x[3]   x[2];
     x[3]   0.0     -x[1];
     -x[2]  x[1]    0.0]
)

"""
    one_hot(::Type{T},n::Int,i::Int) where {T}

Returns a Vector{T} of length n such that `v[i] == 1` and `v[j] .== 0` for all
`j != i`.
"""
function one_hot(::Type{T},n::Int,i::Int) where {T}
    @assert (i > 0 && n > 0) "i and n must be positive integers"
    if i > n
        return one_hot(T,i,n)
    end
    v = zeros(T,n)
    v[i] = T(1)
    return v
end
one_hot(n::Int,i::Int) = one_hot(Float64,n,i)

"""
    wrap_idx(n,idx)

Wrap index to a one-dimensional array of length `n`
"""
wrap_idx(n,idx) = mod(idx-1,n)+1

"""
    wrap_get

Index into array `a` by first wrapping the indices `idx`.
"""
function wrap_get(a::A,idxs) where {R,N,A<:AbstractArray{R,N}}
    a[map(i->wrap_idx(size(a,i),idxs[i]),1:N)...]
end

# end
