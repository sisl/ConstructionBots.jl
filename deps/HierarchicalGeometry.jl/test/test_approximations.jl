let
    a = Hyperrectangle([0.0,0.0],[1.0,1.0])
    b = Hyperrectangle([3.0,0.0],[1.0,1.0])
    c = Hyperrectangle([3.0,3.0],[1.0,1.0])
    d = Hyperrectangle([2.0,0.0],[1.5,0.25])
    @test isapprox(LazySets.distance(a,b), 1.0)
    @test isapprox(LazySets.distance(a,c), sqrt(2))
    @test isapprox(LazySets.distance(a,d), -0.5)

    @test isapprox(LazySets.distance(a,b), distance_lower_bound(a,b))

    @test !has_overlap(a,b)
    @test !has_overlap(a,c)
    @test has_overlap(a,d)

    @test !has_overlap(convert(HPolytope,a),convert(HPolytope,c))
    @test has_overlap(convert(HPolytope,a),convert(HPolytope,d))
end
let
    a = Ball2([0.0,0.0],1.0)
    b = Ball2([2.0,0.0],1.0)
    c = Ball2([1.0,0.0],0.5)
    d = Ball2([0.0,0.0],0.5)
    @test isapprox(LazySets.distance(a,b), 0.0)
    @test isapprox(LazySets.distance(a,c), -0.5)
    @test isapprox(LazySets.distance(a,d), -1.5)
    @test isapprox(LazySets.distance(b,d), 0.5)

    @test isapprox(LazySets.distance(a,b), distance_lower_bound(a,b))

    @test isapprox(LazySets.distance(GeometryCollection([a]),GeometryCollection([b,c])), -0.5)
    @test isapprox(distance_lower_bound(GeometryCollection([a]),GeometryCollection([b,c])), -0.5)

    @test has_overlap(a,b)
    @test has_overlap(a,c)
    @test has_overlap(a,d)
    @test !has_overlap(b,d)

end
let
    geom = LazySets.Ball2(zeros(3),4.0)
    hpoly = overapproximate(geom,equatorial_overapprox_model())
    @test is_intersection_empty(hpoly,LazySets.translate(hpoly,[9.0,0.0,0.0]))
    @test !is_intersection_empty(hpoly,LazySets.translate(hpoly,[2.0,0.0,0.0]))
    vpoly = tovrep(hpoly)
    overapproximate(hpoly,Ball2([0.0,0.0,0.0],1.0))
    # overapproximate(hpoly,Ball2)

    m = GridDiscretization(SVector(zeros(3)...),SVector(ones(3)...))
    occupancy = overapproximate(hpoly,m)
    @test !is_intersection_empty(occupancy,occupancy+2)
    @test is_intersection_empty(occupancy,occupancy+8)
end
let
    # a = GeomNode(Hyperrectangle(MArray{Tuple{3}}(0.0,0.0,0.0),MArray{Tuple{3}}(1.0,1.0,1.0)))
    # b = GeomNode(Hyperrectangle(MArray{Tuple{3}}(3.0,0.0,0.0),MArray{Tuple{3}}(1.0,1.0,1.0)))
    # NOTE Hyperrectangle{Float64,MArray{...},MArray{...}} comes out of the
    # transform. May want to bear that in mind when constructing
    a = GeomNode(Hyperrectangle([0.0,0.0,0.0],[1.0,1.0,1.0]))
    b = GeomNode(Hyperrectangle([3.0,0.0,0.0],[1.0,1.0,1.0]))
    @test isapprox(HierarchicalGeometry.distance_lower_bound(a,b), 1.0)
end
# Test overapproximation in 2D (project from 3D first)
let
    T = SVector{3,Float64}
    m = SMatrix{2,3,Float64}(1.0,0.0,0.0,1.0,0.0,0.0)
    t = CoordinateTransformations.LinearMap(m)
    for (i,geom) in enumerate([
        LazySets.Ball2(zeros(T),1.0),
        Hyperrectangle(zeros(T),ones(T)),
        VPolytope{Float64,T,Vector{T}}([T(0.0,0.0,0.0)]),
        ones(SVector{3,Float64}),
        ones(GeometryBasics.Point{3,Float64})
    ])
        g = t(geom)
        HierarchicalGeometry.project_to_2d(geom)
        if i < 3
            overapproximate(t(geom),ngon_overapprox_model(8))
            if i > 1
                overapproximate(t(geom),Ball2{Float64,SVector{2,Float64}})
            end
        end
    end

    geom = HPolytope{Float64,T}()
    addconstraint!(geom,LazySets.HalfSpace(T(0.0,1.0,0.0),1.0))
    addconstraint!(geom,LazySets.HalfSpace(T(1.0,0.0,0.0),1.0))
    addconstraint!(geom,LazySets.HalfSpace(T(0.0,-1.0,0.0),1.0))
    addconstraint!(geom,LazySets.HalfSpace(T(-1.0,0.0,0.0),1.0))
    t(geom)
    overapproximate(t(geom),ngon_overapprox_model(8))
    HierarchicalGeometry.project_to_2d(geom)
end
# Hyperrectangle
let
    T = SVector{3,Float64}
    for geom in [
        [T(0.0,0.0,0.0),T(1.0,1.0,1.0)],
        Ball2(zeros(3),1.0),
        [Ball2(zeros(3),1.0)],
        GeometryBasics.Line(Point3(0.0,0.0,0.0),Point3(1.0,1.0,1.0)),
        [GeometryBasics.Line(Point3(0.0,0.0,0.0),Point3(1.0,1.0,1.0)),]
    ]
        h = overapproximate(geom,Hyperrectangle{Float64,T,T})
    end
    pts = [T(0.0,0.0,0.0),T(1.0,1.0,1.0)]
    h = overapproximate(pts,Hyperrectangle{Float64,T,T})
    p = convert(VPolytope,h)
    overapproximate(p,Hyperrectangle{Float64,T,T})

end
let
    geom = GeomNode(HierarchicalGeometry.project_to_2d(LazySets.Ball2(zeros(3),1.0)))
    g = HierarchicalGeometry.geom_hierarchy(geom)
    add_child_approximation!(g,HierarchicalGeometry.PolygonKey(),HierarchicalGeometry.BaseGeomKey())
    add_child_approximation!(g,HierarchicalGeometry.CircleKey(),HierarchicalGeometry.PolygonKey())

    # get_cached_geom(get_node(g,HierarchicalGeometry.PolygonKey()))

end
# let
#     table = HierarchicalGeometry.CollisionTable{Int}()
#     geoms = map(i->construct_geometry_tree(GeometryHierarchy(),Ball2(zeros(3),1.0))), 1:3)
#     transforms = [[0.0,0.0,0.0],[4.0,0.0,0.0],[0.0,4.5,0.0],]
#
# end
let
    m = GridDiscretization(SVector(zeros(3)...),SVector(ones(3)...))

    GridOccupancy(m,trues(2,2,2))
    o1 = GridOccupancy(m,trues(2,2,2),SVector(0,0,0))
    @test (o1 + 1).offset == SVector(1,1,1)

    o2 = GridOccupancy(m,trues(2,2,2),SVector(1,1,1))
    @test has_overlap(o1,o2)
    @test has_overlap(o1+[0,1,1],o2)
    @test !has_overlap(o1,o2 + 1)
    @test !has_overlap(o1 - 1,o2)
    rect = overapproximate(o1,Hyperrectangle)
    @test m.origin in rect
    @test !isempty(intersection(Singleton(m.origin),rect))
end
let
    m = GridDiscretization(SVector(zeros(3)...),SVector(ones(3)...))
    @test HierarchicalGeometry.cell_indices(m,[0.0,0.0,0.0]) == [0,0,0]
    HierarchicalGeometry.get_hyperrectangle(m,[0,0,0])

    @test HierarchicalGeometry.cell_indices(m,[1.0,0.0,0.0]) == [1,0,0]
    @test HierarchicalGeometry.cell_indices(m,[-1.0,0.0,0.0]) == [-1,0,0]
end
