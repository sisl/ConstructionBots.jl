#=
This code slightly modified from original code by Luke Stagner (license below):

The MIT License (MIT)
Copyright (c) 2015 Luke Stagner
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Original code found at: https://gist.github.com/lstagner/83f91cc138d4767523a3
=#
# module CubicSplines
#
# using Parameters
# using LinearAlgebra

export CubicSpline, MonotoneCubicSpline, interpolate, deriv, hess, extrapolate

struct CubicSpline{T} # Cubic Hermite Spline
    n::Int        # Number of knots
    x::Vector{T} # x position of knots
    y::Vector{T} # y position of knots
    m::Vector{T} # Tangent at knots
end

function CubicSpline(x::Vector{T}, y::Vector{T}) where {T<:Real}
    issorted(x) || throw(ArgumentError("x points must be in ascending order"))
    nx = length(x)
    m = zeros(nx)
    m[1] = (y[2] - y[1]) / (x[2] - x[1])
    for i = 2:nx-1
        m[i] =
            0.5 * (
                (y[i+1] - y[i]) / (x[i+1] - x[i]) +
                (y[i] - y[i-1]) / (x[i] - x[i-1])
            )
    end
    m[nx] = (y[nx] - y[nx-1]) / (x[nx] - x[nx-1])
    return CubicSpline(nx, x, y, m)
end
"""
  specify tangents explicitly
"""
function CubicSpline(x::Vector{T}, y::Vector{T}, m::Vector{T}) where {T<:Real}
    issorted(x) || throw(ArgumentError("x points must be in ascending order"))
    nx = length(x)
    return CubicSpline(nx, x, y, m)
end

function MonotoneCubicSpline(x::Vector{T}, y::Vector{T}) where {T<:Real}
    issorted(x) || throw(ArgumentError("x points must be in ascending order"))
    nx = length(x)
    m = zeros(nx)
    m[1] = (y[2] - y[1]) / (x[2] - x[1])

    for i = 2:nx-1
        hi = x[i+1] - x[i]
        hi_1 = x[i] - x[i-1]
        di = (y[i+1] - y[i]) / hi
        di_1 = (y[i] - y[i-1]) / hi_1
        m[i] = sign(di) != sign(di_1) ? 0.0 :
            3 *
        (hi_1 + hi) *
        (((2 * hi + hi_1) / di_1) + ((hi + 2 * hi_1) / di))^(-1.0)
    end

    m[nx] = (y[nx] - y[nx-1]) / (x[nx] - x[nx-1])

    return CubicSpline(nx, x, y, m)
end

function interpolate(S::CubicSpline, x::Real)
    xr = searchsorted(S.x, x)
    i1 = max(1, xr.stop)
    i2 = min(length(S.x), xr.start)
    if i1 != i2
        dx = (S.x[i2] - S.x[i1])
        t = (x - S.x[i1]) / dx
        h00 = 2 * t^3 - 3 * t^2 + 1
        h10 = t^3 - 2 * t^2 + t
        h01 = -2 * t^3 + 3 * t^2
        h11 = t^3 - t^2
        return h00 * S.y[i1] +
               h10 * dx * S.m[i1] +
               h01 * S.y[i2] +
               h11 * dx * S.m[i2]
    else
        return S.y[i1]
    end
end
interpolate(S::CubicSpline, x::Vector{T}) where {T<:Real} =
    map(t -> interpolate(S, t), x)

function interpolate(x1,x2,t)
    (1-t)*x1 .+ t*x2
end
function interpolate(x::Vector{T}, t::Real) where {T}
    idx = searchsorted(0:length(x)-1,t)
    # i = min(max(idx.stop,1),length(x)-1)
    # j = max(min(idx.start,length(x)),min(length(x),2))
    # For clipped interpolation:
    i = min(max(idx.stop,1),length(x))
    j = min(max(idx.start,1),length(x))
    interpolate(x[i],x[j],t+1-i)
end

function deriv(S::CubicSpline, x::Real)
    xr = searchsorted(S.x, x)
    i1 = max(1, xr.stop)
    i2 = min(length(S.x), xr.start)
    if i1 != i2
        dx = (S.x[i2] - S.x[i1])
        t = (x - S.x[i1]) / dx
        h00 = (6 * t^2 - 6 * t) / dx
        h10 = (3 * t^2 - 4 * t + 1) / dx
        h01 = (-6 * t^2 + 6 * t) / dx
        h11 = (3 * t^2 - 2 * t) / dx
        return h00 * S.y[i1] +
               h10 * dx * S.m[i1] +
               h01 * S.y[i2] +
               h11 * dx * S.m[i2]
    else
        return S.m[i1]
    end
end
deriv(S::CubicSpline, x::Vector{T}) where {T<:Real} = map(t -> deriv(S, t), x)

function hess(S::CubicSpline, x::Real)
    xr = searchsorted(S.x, x)
    i1 = max(1, xr.stop)
    i2 = min(length(S.x), xr.start)
    if i1 != i2
        dx = (S.x[i2] - S.x[i1])
        t = (x - S.x[i1]) / dx
        h00 = (12 * t - 6) / (dx * dx)
        h10 = (6 * t - 4) / (dx * dx)
        h01 = (-12 * t + 6) / (dx * dx)
        h11 = (6 * t - 2) / (dx * dx)
        return h00 * S.y[i1] +
               h10 * dx * S.m[i1] +
               h01 * S.y[i2] +
               h11 * dx * S.m[i2]
    else
        return 0.0
    end
end
hess(S::CubicSpline, x::Vector{T}) where {T<:Real} = map(t -> hess(S, t), x)

function extrapolate(
    S::CubicSpline{T},
    x::Union{T,Vector{T}},
    order::Int = 0,
) where {T<:Real}
    nx = length(x)
    yout = zeros(nx)
    if order == 0
        coeffs = [0.0, 0.0]
    elseif order == 1
        coeffs = [1.0, 0.0]
    elseif order >= 2
        coeffs = [1.0, 1 / 2]
    end
    xrange = extrema(S.x)
    for (i, xi) in enumerate(x)
        if (xi < xrange[1] || xi > xrange[2])
            if xi < xrange[1]
                idx = 1
            elseif xi > xrange[2]
                idx = length(S.x)
            end
            dy_dx = deriv(S, S.x[idx])[1]
            d2y_dx2 = hess(S, S.x[idx])[1]
            dx = (x[i] - S.x[idx])
            dy = coeffs[1] * dy_dx * dx + coeffs[2] * d2y_dx2 * dx^2
            yout[i] = S.y[idx] + dy # first order extension
        else
            # normal case
            yout[i] = interpolate(S, xi)[1]
        end
    end
    return yout
end

export ParametricSpline

@with_kw struct ParametricSpline{N,T}
    splines::NTuple{N,CubicSpline{T}} = NTuple{0,CubicSpline{Float64}}()
end
function ParametricSpline(t::Vector{T}, x::Array{T,2}) where {T<:Real}
    ParametricSpline(tuple(map(i -> CubicSpline(t, x[:, i]), 1:size(x, 2))...))
end
function ParametricSpline(
    t::Vector{T},
    x::Array{T,2},
    v::Array{T,2},
) where {T<:Real} # with prescribed knots
    ParametricSpline(tuple(map(
        i -> CubicSpline(t, x[:, i], v[:, i]),
        1:size(x, 2),
    )...))
end
interpolate(S::ParametricSpline, t::T) where {T<:Real} =
    [map(s -> interpolate(s, t), S.splines)...]
deriv(S::ParametricSpline, t::T) where {T<:Real} =
    [map(s -> deriv(s, t), S.splines)...]
hess(S::ParametricSpline, t::T) where {T<:Real} =
    [map(s -> hess(s, t), S.splines)...]

interpolate(S::ParametricSpline, t::T) where {T<:Vector} =
    map(k -> interpolate(S, k), t)
deriv(S::ParametricSpline, t::T) where {T<:Vector} = map(k -> deriv(S, k), t)
hess(S::ParametricSpline, t::T) where {T<:Vector} = map(k -> hess(S, k), t)

# interpolate(S::P,t::V) where {P<:ParametricSpline,V<:Vector} = map(k->interpolate(S,k), t)
# deriv(S::P,t) where {P<:ParametricSpline}                 = map(s->deriv(s,t), S.splines)
# hess(S::P,t) where {P<:ParametricSpline}                  = map(s->hess(s,t), S.splines)
#
# interpolate(S::P,t) where {P<:ParametricSpline}           = map(k->interpolate(S,k), S.splines)
# deriv(S::P,t) where {P<:ParametricSpline}                 = map(s->deriv(s,t), S.splines)
# hess(S::P,t) where {P<:ParametricSpline}                  = map(s->hess(s,t), S.splines)
export tangent_rate, curve_turn_rate

"""
    tangent_rate(S::ParametricSpline,t::Real)
Computes the rate (w.r.t. t) at which the tangent vector to a curve is changing.
"""
function tangent_rate(S::ParametricSpline, t::Real)
    dr = deriv(S, t) # position first derivative w.r.t time
    ddr = hess(S, t) # position first derivative w.r.t time
    ds = sqrt(dot(dr, dr)) # arc length first derivative
    dds = sum(dr) / ds # arc length second derivative
    dT = (1 / (ds^2)) * (ds * ddr .- dds * dr)
end

"""
    curve_turn_rate(S::ParametricSpline,t::Real)
computes the turn rate (w.r.t. t) of the tangent vector to a curve.
"""
function curve_turn_rate(S::ParametricSpline, t::Real)
    dr = map(s -> deriv(s, t), S.splines) # position first derivative w.r.t t
    dT = tangent_rate(S, t) # tangent vector first derivative w.r.t. t
    ω = dr × dT
end

extrapolate(S::ParametricSpline, t::T, order = 0) where {T<:Real} =
    map(s -> extrapolate(s, t, order)[1], S.splines)
extrapolate(S::ParametricSpline, t_vec::T, order = 0) where {T<:Vector} =
    map(k -> extrapolate(S, k, order), t_vec)

# end # module
