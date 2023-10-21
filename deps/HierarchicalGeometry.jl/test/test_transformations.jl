# Test Transformations
let
    ball = LazySets.Ball2([1.0,0.0,0.0],1.0)
    bbox = overapproximate(ball,Hyperrectangle)
    hpoly = convert(LazySets.HPolytope,bbox)
    vpoly = convert(LazySets.VPolytope,bbox)
    poly = Polyhedra.polyhedron(hpoly)

    # setobject!(vis[:ball], GeometryBasics.Sphere(ball))
    # setobject!(vis[:bbox], GeometryBasics.HyperRectangle(bbox))
    # setobject!(vis[:polytope], Polyhedra.Mesh(poly), POLYHEDRON_MATERIAL)

    # Translation
    t = CoordinateTransformations.Translation(1.0,2.0,3.0)
    @test array_isapprox(ball.center.+t.translation.data,t(ball).center)
    @test isapprox(ball.radius,t(ball).radius)
    @test array_isapprox(bbox.center.+t.translation.data,t(bbox).center)
    @test isapprox(bbox.radius,transform(bbox,t).radius)
    for (v1,v2) in zip(vertices_list(vpoly),vertices_list(t(vpoly)))
        @test array_isapprox(v1+t.translation,v2)
    end
    for (v1,v2) in zip(vertices_list(hpoly),vertices_list(t(hpoly)))
        @test array_isapprox(v1+t.translation,v2)
    end

    # settransform!(vis[:ball], a)
    # settransform!(vis[:polytope], a)

    # delete!(vis[:transformedball])
    # setobject!(vis[:transformed_ball],GeometryBasics.Sphere(transform(ball,a)))
    # setobject!(vis[:transformed_bbox],GeometryBasics.HyperRectangle(transform(bbox,a)))
    # setobject!(vis[:transformed_polytope],Polyhedra.Mesh(Polyhedra.polyhedron(transform(vpoly,a))), POLYHEDRON_MATERIAL)
    # setobject!(vis[:transformed_polytope],Polyhedra.Mesh(Polyhedra.polyhedron(transform(hpoly,a))), POLYHEDRON_MATERIAL)


    # Rotation
    r = CoordinateTransformations.LinearMap(RotZ(π/4))
    @test array_isapprox(transform(ball.center,r),transform(ball,r).center)
    @test isapprox(ball.radius,transform(ball,r).radius)
    @test array_isapprox(transform(bbox.center,r),transform(bbox,r).center)
    @test isapprox(bbox.radius[1]*sqrt(2),transform(bbox,r).radius[1])
    for (v1,v2) in zip(vertices_list(vpoly),vertices_list(transform(vpoly,r)))
        @test array_isapprox(Vector(r(v1)),v2)
    end
    for (v1,v2) in zip(vertices_list(hpoly),vertices_list(transform(hpoly,r)))
        @test array_isapprox(Vector(r(v1)),v2)
    end

    # settransform!(vis[:ball], t)
    # settransform!(vis[:polytope], t)

    # Combined
    a = CoordinateTransformations.compose(t,r) # Rotate first, then translate
    # a = CoordinateTransformations.compose(r,t) # Rotate first, then translate
    @test array_isapprox(transform(ball.center,a),transform(ball,a).center)
    @test isapprox(ball.radius,transform(ball,a).radius)
    @test array_isapprox(transform(bbox.center,a),transform(bbox,a).center)
    @test isapprox(bbox.radius[1]*sqrt(2),transform(bbox,a).radius[1])
    for (v1,v2) in zip(vertices_list(vpoly),vertices_list(transform(vpoly,a)))
        @test array_isapprox(Vector(a(v1)),v2)
    end
    for (v1,v2) in zip(vertices_list(hpoly),vertices_list(transform(hpoly,a)))
        @test array_isapprox(Vector(a(v1)),v2)
    end
end
# relative transformations
let
    # Show how to compute a relative transform
    # t = inv(a) ∘ b
    # b = a ∘ t
    # a = b ∘ inv(t)
    for i in 1:10
        a = CoordinateTransformations.Translation(rand(3)) ∘ CoordinateTransformations.LinearMap(rand(RotMatrix{3,Float64}))
        b = CoordinateTransformations.Translation(rand(3)) ∘ CoordinateTransformations.LinearMap(rand(RotMatrix{3,Float64}))
        t = inv(a) ∘ b
        p = rand(3)
        @test array_isapprox(HierarchicalGeometry.interpolate_rotation(a.linear,b.linear,1.0), t.linear;atol=1e-6,rtol=1e-6)
        @test isapprox(norm(b(p) - (a ∘ t)(p)),0.0;atol=1e-6,rtol=1e-6)
        @test isapprox(norm(a(p) - (b ∘ inv(t))(p)),0.0;atol=1e-6,rtol=1e-6)
    end

    # Interpolate from one transform to another
    p = SVector(0.0,1.0,0.0) # point p starts at origin
    a = CoordinateTransformations.Translation(0,2,0) ∘ CoordinateTransformations.LinearMap(RotX(0.25*π))
    @show a
    @show a(p) # move point p - rotate first, then translate (i.e, a.translation ∘ a.linear )
    b = CoordinateTransformations.Translation(4,0,0) ∘ CoordinateTransformations.LinearMap(RotZ(0.75*π))
    @show b
    @show b(p)
    # try to build up to global transform b, starting from a
    # t = HierarchicalGeometry.relative_transform(a,b) # parent,child
    t = inv(a) ∘ b
    # b = a ∘ t
    # a = b ∘ inv(t)
    @show t
    @show (a ∘ t)(p) # first transform point by t, then by a, to get the same transform as b
    @show a(t(p)) # first transform point by t, then by a, to get the same transform as b
    # How to move incrementally from a to b with multiple intermediate transformations?
    dt = 0.2
    v_max = 2.0
    ω_max = 1.0
    c = a
    goal = b
    for k in 1:15
        # delta = HierarchicalGeometry.relative_transform(c,goal) # remaining difference between c and b
        delta = inv(c) ∘ goal
        # translation error
        dx = delta.translation
        if norm(dx) <= 1e-4
            vel = SVector(0.0,0.0,0.0)
        else
            vel = normalize(dx) * min(v_max, norm(dx)/dt)
        end
        Δx = vel*dt # translation increment
        # rotation error
        r = Rotations.RotationVec(delta.linear) # rotation vector
        θ = SVector(r.sx,r.sy,r.sz) # convert r to svector
        if norm(θ) <= 1e-4
            # no more rotating to do
            ΔR = SMatrix{3,3,Float64}(I)
        else
            ω = normalize(θ) * min(ω_max, norm(θ)/dt)
            ΔR = exp(cross_product_operator(ω)*dt)
        end
        # First move in the direction of
        # Δ = CoordinateTransformations.LinearMap(ΔR) ∘ CoordinateTransformations.Translation(Δx)
        Δ = CoordinateTransformations.Translation(Δx) ∘ CoordinateTransformations.LinearMap(ΔR)
        # c = Δ ∘ c
        c = c ∘ Δ
        @show c(p)
    end
    @test array_isapprox(a(t(p)), b(p), rtol=1e-12, atol=1e-12)
    @test array_isapprox(a(t(p)), c(p), rtol=1e-12, atol=1e-12)

    # Initialize a rotation matrix
    R = rand(RotMatrix{3,Float64})
    # compute the axis and magnitude of rotation
    r = Rotations.RotationVec(R)
    vec = SVector(r.sx,r.sy,r.sz)
    # reconstruct the matrix
    Rr = exp(cross_product_operator(vec))
    # check that Rr == R
    @test array_isapprox( inv(Rr)*R, SMatrix{3,3,Float64}(I); rtol=1e-5,atol=1e-5)
    # Show that we can chain partial rotations to reconstruct R
    Rr = exp(cross_product_operator(vec*0.25))
    @test array_isapprox( inv(Rr)*inv(Rr)*inv(Rr)*inv(Rr)*R, SMatrix{3,3,Float64}(I); rtol=1e-5,atol=1e-5)


    ω = Rotations.RotationVec(Rotations.QuatRotation(inv(t.linear)))
    vec = SVector(ω.sx, ω.sy, ω.sz)
    exp(cross_product_operator(vec))

    a = CoordinateTransformations.Translation(1,0,0) ∘ CoordinateTransformations.LinearMap(RotXYZ(0.0,0.0,0.0))
    b = CoordinateTransformations.Translation(1,4,0) ∘ CoordinateTransformations.LinearMap(RotXYZ(0.1,0.0,1.0*π))
    t = HierarchicalGeometry.relative_transform(a,b)
    @test array_isapprox(inv(t.linear),b.linear;rtol=1e-12,atol=1e-12)
    @test array_isapprox(a.linear*inv(t.linear),b.linear;rtol=1e-12,atol=1e-12)
end
# Test TransformNode
let
    a = TransformNode()
    b = TransformNode()
    c = TransformNode()
    HierarchicalGeometry.set_parent!(b,a)
    HierarchicalGeometry.set_parent!(c,b)

    @test get_parent(a) === a
    @test get_parent(b) === a
    @test get_parent(c) === b

    t = identity_linear_map() ∘ CoordinateTransformations.Translation(1,0,0)

    set_local_transform!(a,t) # Not connected to tree, so can't globally update
    @test array_isapprox(global_transform(a).translation,t.translation)
    @test array_isapprox(global_transform(b).translation,t.translation)
    @test array_isapprox(global_transform(c).translation,t.translation)

    set_local_transform!(b,t)
    @test array_isapprox(global_transform(a).translation,t.translation)
    @test array_isapprox(global_transform(b).translation,2*t.translation)
    @test array_isapprox(global_transform(c).translation,2*t.translation)

    set_local_transform!(c,t)
    @test array_isapprox(global_transform(a).translation,t.translation)
    @test array_isapprox(global_transform(b).translation,2*t.translation)
    @test array_isapprox(global_transform(c).translation,3*t.translation)

    @test validate_tree(c)
end
# test set_desired_global_transform!(...)
let
    a = TransformNode()
    b = TransformNode()

    set_parent!(b,a) # a is parent of b

    set_local_transform!(a,CoordinateTransformations.LinearMap(rand(RotMatrix{3,Float64})) ∘ CoordinateTransformations.Translation(rand(3)))
    set_local_transform!(b,CoordinateTransformations.LinearMap(rand(RotMatrix{3,Float64})) ∘ CoordinateTransformations.Translation(rand(3)))

    HierarchicalGeometry.set_desired_global_transform!(b,global_transform(a))
    t = relative_transform(a,b) # should be identity
    @test isapprox(norm(t.linear .- one(SMatrix{3,3,Float64})), 0, atol=1e-10)
    @test isapprox(norm(t.translation), 0, atol=1e-10)

end
# Test Transform Tree
let
    a = TransformNode()
    b = TransformNode()
    c = TransformNode()
    tree = GraphUtils.CustomNTree{CustomNode{typeof(a),Symbol},Symbol}()
    add_node!(tree,a,:ONE)
    add_child!(tree,:ONE,b,:TWO)
    add_child!(tree,:TWO,c,:THREE)
    # add_node!(tree,b,:TWO)
    # add_node!(tree,c,:THREE)
    # set_child!(tree,:ONE,:TWO)
    # set_child!(tree,:TWO,:THREE)
    t = CoordinateTransformations.Translation(1.0,0.0,0.0) ∘ CoordinateTransformations.LinearMap(RotZ(0))
    set_local_transform!(tree,1,t)
    @test !GraphUtils.cached_node_up_to_date(a)
    @test !GraphUtils.cached_node_up_to_date(b)
    @test !GraphUtils.cached_node_up_to_date(c)
    set_local_transform!(tree,1,t,true)
    @test GraphUtils.cached_node_up_to_date(a)
    @test GraphUtils.cached_node_up_to_date(b)
    @test GraphUtils.cached_node_up_to_date(c)
    set_local_transform!(tree,1,t)
    @test !GraphUtils.cached_node_up_to_date(a)
end
let
    geom = Ball2(zeros(3),1.0)
    for (a,b,c,d) in [
            (TransformNode(),TransformNode(),TransformNode(),TransformNode()),
            (GeomNode(geom),GeomNode(geom),GeomNode(geom),GeomNode(geom)),
            ]
        tree = GraphUtils.CustomNTree{CustomNode{typeof(a),Symbol},Symbol}()
        add_node!(tree,a,:ONE)
        add_child!(tree,:ONE,b,:TWO)
        add_child!(tree,:TWO,c,:THREE)
        # add_node!(tree,b,:TWO)
        # add_node!(tree,c,:THREE)
        # set_child!(tree,:ONE,:TWO)
        # set_child!(tree,:TWO,:THREE)
        t = CoordinateTransformations.Translation(1.0,0.0,0.0) ∘ CoordinateTransformations.LinearMap(RotZ(0))
        tree2 = deepcopy(tree) # Test deferred computation of transformations
        tree3 = deepcopy(tree) # Test deferred computation of transformations
        for v in Graphs.vertices(tree)
            set_local_transform!(get_node(tree,v),t) # Not connected to tree, so can't globally update
            set_local_transform!(tree2,v,t,true) # update successors immediately
            set_local_transform!(tree3,v,t,false) # defer update of predecessors
        end
        for v in Graphs.vertices(tree)
            # Now transforms will be updated because query is made with tree
            @test array_isapprox(global_transform(tree,v).translation,[v, 0.0, 0.0])
            @test array_isapprox(global_transform(tree2,v).translation,[v, 0.0, 0.0])
            @test array_isapprox(global_transform(tree3,v).translation,[v, 0.0, 0.0])
        end
        for v in Graphs.vertices(tree)
            @test array_isapprox(global_transform(tree,v).translation,[v, 0.0, 0.0])
        end
        # Test `change_parent(...)`
        add_node!(tree,d,:FOUR)
        set_child!(tree,:THREE,:FOUR)
        # set_parent!(d,c)
        # GraphUtils.add_child!(tree,:THREE,d,:FOUR)
        # set_parent!(d,c)
        set_local_transform!(tree,:FOUR,t)
        @test array_isapprox(global_transform(tree,:FOUR).translation,[4.0,0,0])
        set_child!(tree,:TWO,:FOUR)
        @test Graphs.has_edge(tree,:TWO,:FOUR)
        @test !Graphs.has_edge(tree,:THREE,:FOUR)
        @test array_isapprox(global_transform(tree,:FOUR).translation,[4.0,0,0])
        set_local_transform!(tree,:FOUR,t)
        @test array_isapprox(global_transform(tree,:FOUR).translation,[3.0,0,0])
    end
end
# Test GeomNode
let
    geom = Ball2(ones(SVector{3,Float64}),1.0)
    n = GeomNode(geom)
    @test array_isapprox(get_base_geom(n).center,geom.center)
    # set_up_to_date!(n,true)
    # @test is_up_to_date(n)
    t = CoordinateTransformations.Translation(1.0,0,0) ∘ CoordinateTransformations.LinearMap(RotZ(0))
    set_local_transform!(n,t)
    # set_global_transform!(n,t)
    # @test !is_up_to_date(n)
    @test array_isapprox(get_cached_geom(n).center, geom.center .+ [t.translation.data...])
    # @test is_up_to_date(n)
    # set_global_transform!(n,t)
    # @test !is_up_to_date(n)
end
# Test SceneTree
let
    tree = SceneTree()
    geom = Ball2(zeros(SVector{3,Float64}),1.0)

    r = add_node!(tree,RobotNode(RobotID(1),GeomNode(deepcopy(geom))))
    o = add_node!(tree,ObjectNode(ObjectID(1),GeomNode(deepcopy(geom))))
    a = add_node!(tree,AssemblyNode(AssemblyID(1),GeomNode(deepcopy(geom))))
    add_component!(get_node(tree,AssemblyID(1)), node_id(o)=>identity_linear_map())
    n = add_node!(tree,TransportUnitNode(GeomNode(deepcopy(geom)),AssemblyID(1)))
    add_robot!(n,node_id(r)=>identity_linear_map())

    @test_throws AssertionError set_child!(tree,o,r)
    @test_throws AssertionError set_child!(tree,r,o)
    @test_throws AssertionError set_child!(tree,n,o)
    @test_throws AssertionError set_child!(tree,a,r)

    set_child!(tree,a,o)
    @test Graphs.has_edge(tree,a,o)
    set_child!(tree,n,r)
    @test Graphs.has_edge(tree,n,r)
    set_child!(tree,n,a)
    @test Graphs.has_edge(tree,n,a)
    t = CoordinateTransformations.Translation(1.0,0,0) ∘ CoordinateTransformations.LinearMap(RotZ(0))
    set_local_transform!(tree,n,t)
    for v in Graphs.vertices(tree)
        @test array_isapprox(global_transform(tree,v).translation,t.translation)
    end

    base_tree = deepcopy(tree)

    # Verify that edge removal affects graph edges and "hidden" transform tree
    # structure.
    Graphs.rem_edge!(tree,n,a)
    tn = HierarchicalGeometry.get_transform_node(a)
    @test get_parent(tn) === tn
    # Test that nodes don't "jump" when the edge is broken
    for v in Graphs.vertices(tree)
        @test array_isapprox(global_transform(tree,v).translation,t.translation)
    end

    # # test copy behavior of nodes only
    set_local_transform!(base_tree,n,identity_linear_map())
    # base_tree = tree
    # t = CoordinateTransformations.Translation(1.0,0.0,0.0)
    # tree = deepcopy(base_tree)
    # for v in LightGraphs.vertices(tree)
    #     n = get_node(tree,v)
    #     set_up_to_date!(n,true)
    #     n2 = copy(n) # Compare to a copy that's still attached to the tree
    #     n3 = copy(n) # Compare to a copy that's not attached to the tree
    #     set_up_to_date!(n,false)
    #     set_up_to_date!(n3,false)
    #     @test is_up_to_date(n2)
    #     @test is_up_to_date(n) != is_up_to_date(n2)
    #     @test is_up_to_date(n3) != is_up_to_date(n2)
    #     tn = t ∘ global_transform(n)
    #     set_global_transform!(n,tn)
    #     set_global_transform!(n3,tn)
    #     @test isapprox(norm(get_cached_geom(n).center - get_cached_geom(n2).center),norm(t.translation))
    #     @test isapprox(norm(get_cached_geom(n3).center - get_cached_geom(n2).center),norm(t.translation))
    # end

    tree = copy(base_tree)
    for v in Graphs.vertices(tree)
        n = get_node(base_tree,v)
        n2 = get_node(tree,v)
        tn = t ∘ local_transform(n)
        set_local_transform!(base_tree,n,tn)
        @test isapprox(norm(get_cached_geom(n).center - get_cached_geom(n2).center),norm(t.translation))
    end

end
# Test SceneTree some more
let
    tree = SceneTree()
    # Goal:
    # Assembly 1
    #  -- Assembly 2
    #      -- Object 1
    geom = Ball2(zeros(SVector{3,Float64}),1.0)
    a1 = add_node!(tree,AssemblyNode(AssemblyID(1),GeomNode(deepcopy(geom))))
    a2 = add_node!(tree,AssemblyNode(AssemblyID(2),GeomNode(deepcopy(geom))))
    o1 = add_node!(tree,ObjectNode(ObjectID(1),GeomNode(deepcopy(geom))))
    add_component!(a1, node_id(a2)=>identity_linear_map())
    add_component!(a2, node_id(o1)=>identity_linear_map())

    r1 = add_node!(tree,RobotNode(RobotID(1),GeomNode(deepcopy(geom))))
    t1 = add_node!(tree,TransportUnitNode(GeomNode(deepcopy(geom)),
        node_id(a2)=>identity_linear_map()))
    add_robot!(t1,node_id(r1)=>identity_linear_map())

    @test isa(make_edge(tree,t1,r1), HierarchicalGeometry.TemporaryEdge)
    @test isa(make_edge(tree,t1,a2), HierarchicalGeometry.TemporaryEdge)
    @test isa(make_edge(tree,a1,a2), HierarchicalGeometry.PermanentEdge)
    @test isa(make_edge(tree,a2,o1), HierarchicalGeometry.PermanentEdge)

    @test capture_child!(tree,a2,o1) # lock o1 into a2
    @test Graphs.has_edge(tree,a2,o1)
    @test set_child!(tree,t1,a2) # make a2 a temporary child of t1

    set_local_transform!(tree,r1,local_transform(r1) ∘ CoordinateTransformations.Translation(1.0,0.0,0.0))
    @test !capture_child!(tree,t1,r1) # not close enough to be captured
    set_local_transform!(tree,r1,child_transform(t1,node_id(r1)))
    @test capture_child!(tree,t1,r1) # close enough to be captured now

    # Graphs.rem_edge!(tree,t1,a2) # once a2 is in place w.r.t a1, remove edge from t1
    @test capture_child!(tree,a1,a2) # lock a2 into a1. a1 is complete
    @test Graphs.has_edge(tree,a1,a2)
    @test !Graphs.has_edge(tree,t1,r1) # verify that t1 has been disbanded

    @test_throws AssertionError Graphs.rem_edge!(tree,a1,a2) # a1 → a2 should be locked
    @test_throws AssertionError Graphs.rem_edge!(tree,a2,o1) # a2 → o1 should be locked

    @test_throws AssertionError set_child!(tree,o1,r1)
    @test_throws AssertionError set_child!(tree,r1,o1)
    @test_throws AssertionError set_child!(tree,t1,o1)
    @test_throws AssertionError set_child!(tree,a1,r1)
end
# test copy(::SceneNode)
let
    geom = Ball2(zeros(SVector{3,Float64}),1.0)
    t = CoordinateTransformations.Translation(1.0,0.0,0.0)

    n = AssemblyNode(AssemblyID(1),GeomNode(deepcopy(geom)))
    add_component!(n, ObjectID(1)=>identity_linear_map())
    n2 = copy(n)
    assembly_components(n)[ObjectID(1)] = identity_linear_map() ∘ t
    # AssemblyNode compondents should be shared
    @test isapprox(
        norm(HierarchicalGeometry.child_transform(n,ObjectID(1)).translation
        - HierarchicalGeometry.child_transform(n2,ObjectID(1)).translation),
        0.0)

    n = TransportUnitNode(GeomNode(deepcopy(geom)),AssemblyID(1))
    add_robot!(n,RobotID(1)=>identity_linear_map())
    n2 = copy(n)
    add_robot!(n2,RobotID(1)=>identity_linear_map() ∘ t)
    # TransformNode compondents and assembly should be shared
    @test isapprox(
        norm(HierarchicalGeometry.child_transform(n,RobotID(1)).translation
        - HierarchicalGeometry.child_transform(n2,RobotID(1)).translation),
        0.0)
end
# Test SceneNode with GeometryHierarchy
let
    geom = [
        GeometryBasics.Line(zero(Point3{Float64}),ones(Point3{Float64})),
        ]
    n = HierarchicalGeometry.geom_hierarchy(GeomNode(geom))
    ϵ = 2.5
    add_child_approximation!(n,PolyhedronKey(),BaseGeomKey())
    # add_child_approximation!(n,HypersphereKey(),PolyhedronKey())
    # add_child_approximation!(n,HyperrectangleKey(),PolyhedronKey())
    # for v in Graphs.vertices(n)
    #     get_cached_geom(n,get_vtx_id(n,v))
    # end

end
