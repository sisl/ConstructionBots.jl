export transform_iter

"""
    transform_iter(f, it) = Base.Iterators.accumulate((a,b)->f(b), it)

Transform iterator that applies `f` to each element of `it`.
"""
transform_iter(f, it) = (f(v) for v in it)
# transform_iter(f, it) = Base.Iterators.accumulate((a,b)->f(b), it)