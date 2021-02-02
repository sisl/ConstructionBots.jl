let
    geom = Ball2(zeros(SVector{3,Float64}),1.0)
    o = ObjectNode(ObjectID(1),GeomNode(deepcopy(geom)))
    r = RobotNode(RobotID(1),GeomNode(deepcopy(geom)))
    a = AssemblyNode(AssemblyID(1),GeomNode(deepcopy(geom)))
    t = TransportUnitNode(node_id(a))

    s = Set{AbstractID}()
    for n in [
        ObjectStart(o),
        RobotStart(r),
        RobotGo(r),
        AssemblyStart(a),
        AssemblyComplete(a),
        LiftIntoPlace(a),
        LiftIntoPlace(o),
        FormTransportUnit(t),
        TransportUnitGo(t),
        DepositCargo(t),
    ]
        @test !(node_id(n) in s)
        push!(s, node_id(n))
    end

end