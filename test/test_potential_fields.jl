using Test

# Cone potential
let
    p = ConstructionBots.ConePotential(-1.0, 1.0, Point(0.0, 0.0))
    x1 = Point(0.5, 0.5)
    d1 = normalize(Point(-1.0, -1.0))

    y = p(x1)
    dx1 = ConstructionBots.potential_gradient(p, x1)
    @test isapprox(norm(dx1), 1)
    @test isapprox(norm(dx1 - d1), 0.0, atol = 1e-10)

    # out of range
    x2 = Point(2.0, 2.0)
    dx2 = ConstructionBots.potential_gradient(p, x2)
    @test isapprox(norm(dx2), 0)

end
# Rotational Potential
let
    p = ConstructionBots.ConePotential(-1.0, 2.0, Point(0.0, 0.0))
    rp = ConstructionBots.RotationalPotential(p, 1.0, 1.0)
    x = Point(1.0, 1.0)
    ConstructionBots.potential_gradient(p, x)

    ConstructionBots.potential_gradient(rp, x)
    ConstructionBots.potential_gradient([rp], x)

end
# tent potential
let
    p = ConstructionBots.TentPotential(-1.0, 1.0, Point(0.0, 0.0), Point(0.0, 2.0))
    x1 = Point(0.5, 0.5)
    d1 = normalize(Point(-1.0, -0.0))
    dx1 = ConstructionBots.potential_gradient(p, x1)
    @test isapprox(norm(dx1), 1)
    @test isapprox(norm(dx1 - d1), 0.0, atol = 1e-10)

    # out of range
    x2 = Point(2.0, 2.0)
    dx2 = ConstructionBots.potential_gradient(p, x2)
    @test isapprox(norm(dx2), 0)

end
# BarrierPotential
let
    p = ConstructionBots.ConePotential(1.0, 1.0, Point(0.0, 0.0))
    pb = ConstructionBots.BarrierPotential(p, 1.0, 0.1)
    y = ConstructionBots.potential(pb, p.x)
    @test isapprox(y, pb.c / pb.z, atol = 1e-10)

end
# superposed potentials
let
    p = [
        ConstructionBots.ConePotential(-1.0, 1.0, Point(0.0, 0.0)),
        ConstructionBots.TentPotential(-1.0, 1.0, Point(0.0, 0.0), Point(0.0, 2.0)),
    ]
    x1 = Point(0.5, 0.5)
    d1 = normalize(Point(-1.0, -1.0)) + normalize(Point(-1.0, -0.0))
    dx1 = ConstructionBots.potential_gradient(p, x1)
    @test isapprox(norm(dx1 - d1), 0.0, atol = 1e-10)
end
