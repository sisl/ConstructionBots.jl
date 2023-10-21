let
    θ = 1.0*π
    @test θ == wrap_to_pi(θ)
    @test θ == wrap_to_pi(-θ)

    θ = 20.0
    @test -π < wrap_to_pi(θ) <= π
end

let
    v1 = [0,0]
    v2 = [0,1]
    pts = GraphUtils.nearest_points_between_circles(v1,v2,0.0,0.0)
    @test all(isapprox.(pts[1],v1))
    @test all(isapprox.(pts[2],v2))
    pts = GraphUtils.nearest_points_between_circles(v1,v2,0.5,0.5)
    @test all(isapprox.(pts[1],(v1+v2)/2))
    @test all(isapprox.(pts[2],(v1+v2)/2))

    pts = GraphUtils.nearest_points_between_circles(v1,v2,1,4)
    @test all(isapprox.(pts[1],[0.0,-1.0]))
    @test all(isapprox.(pts[2],[0.0,-3.0]))
    pts = GraphUtils.nearest_points_between_circles(v1,v2,4,1)
    @test all(isapprox.(pts[1],[0.0,4.0]))
    @test all(isapprox.(pts[2],[0.0,2.0]))

    pts = GraphUtils.nearest_points_between_circles(v1,v2,sqrt(2)/2,sqrt(2)/2)
    @test isapprox(abs(pts[1][1]),0.5)
    @test isapprox(abs(pts[2][1]),0.5)

end