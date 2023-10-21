let
    n = 5
    tf = 5
    t = collect(0:(tf/n):tf)
    f(t) = sin.(t)
    fdot(t) = cos.(t)

    x = f(t)
    spline = CubicSpline(t,x)
    interpolate(spline, t[1])
    deriv(spline, t[1])
    hess(spline, t[1])
    interpolate(spline, [t[1], t[end]])
    deriv(spline, [t[1], t[end]])
    hess(spline, [t[1], t[end]])

    dt = 1.0
    @test extrapolate(spline, t[end]+dt, 0)[1] == interpolate(spline, t[end])
    @test extrapolate(spline, t[end]+dt, 1)[1] == interpolate(spline, t[end]) + deriv(spline, t[end])*dt
    @test extrapolate(spline, t[1]-dt, 0)[1] == interpolate(spline, t[1])
    @test extrapolate(spline, t[1]-dt, 1)[1] == interpolate(spline, t[1]) - deriv(spline, t[1])*dt

    xdot = fdot(t)
    spline2 = CubicSpline(t,x,xdot) # prescribe knot tangents directly

    # plt = plot(t_vec, interpolate(spline, t_vec))
    # plot!(plt, t_vec, interpolate(spline2, t_vec))
    # plot!(plt, t_vec, f(t_vec))
    # plot!(plt, t, x, seriestype=:scatter)
end
let
    n = 10
    t = collect(0:(1/10):1)
    x = sin.(t)
    y = sin.(t)
    X = hcat(x,y)
    spline = ParametricSpline(t,X)
    interpolate(spline, t[1])
    deriv(spline, t[1])
    hess(spline, t[1])
    interpolate(spline, [t[1], t[end]])
    deriv(spline, [t[1], t[end]])
    hess(spline, [t[1], t[end]])
end
# test turn rate and tangent rate
let
    # circle in the plane
    tvec = collect(0:.01:2π)
    x = cos.(tvec)
    y = sin.(tvec)
    dx = -y
    dy = x
    ddx = -x
    ddy = -y
    z = zeros(length(tvec))
    dz = z
    ddz = z
    r = collect(hcat(x,y,z))
    dr = collect(hcat(dx,dy,dz))
    ddr = collect(hcat(ddx,ddy,ddz))
    spline = ParametricSpline(tvec,r,dr)

    # @show tangent_rate(spline,0.0)
    # @show tangent_rate(spline,π/4)
    #
    # tvec2 = (1/2)*(tvec[2:end].+tvec[1:end-1])
    # pts = interpolate(spline,tvec2)
    # plt = plot(map(pt->pt[1],pts),map(pt->pt[2],pts),aspect_ratio=:equal)
    # pts = deriv(spline,tvec2)
    # plot!(plt,map(pt->pt[1],pts),map(pt->pt[2],pts),aspect_ratio=:equal)
    # pts = hess(spline,tvec2)
    # plot(map(pt->pt[1],pts),map(pt->pt[2],pts),aspect_ratio=:equal)
end
