let
    # Interpolate from one transform to another
    p = SVector(0.0, 1.0, 0.0) # point p starts at origin
    a = CoordinateTransformations.Translation(0, 2, 0) ∘ CoordinateTransformations.LinearMap(RotX(0.25 * π))
    b = CoordinateTransformations.Translation(4, 0, 0) ∘ CoordinateTransformations.LinearMap(RotZ(0.75 * π))
    t = inv(a) ∘ b
    @test array_isapprox(b(p), a(t(p)); atol=1e-6, rtol=1e-6)

    # Jump straight from a to the goal
    dt = 0.2
    twist = optimal_twist(t, Inf, Inf, dt)
    Δ = integrate_twist(twist, dt)
    @test array_isapprox(b(p), a(Δ(p)); atol=1e-6, rtol=1e-6)

    # Now build to the goal transform incrementally
    v_max = 2.0
    ω_max = 1.0
    c = a
    goal = b
    for k in 1:15
        delta = inv(c) ∘ goal
        twist = ConstructionBots.optimal_twist(delta, v_max, ω_max, dt)
        Δ = ConstructionBots.integrate_twist(twist, dt)
        c = c ∘ Δ
    end
    @test array_isapprox(b(p), c(p); atol=1e-6, rtol=1e-6)

end
