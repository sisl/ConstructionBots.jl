let
    geom = Ball2(zeros(SVector{3,Float64}), 1.0)
    o = ObjectNode(ConstructionBots.ObjectID(1), GeomNode(deepcopy(geom)))
    r = RobotNode(ConstructionBots.RobotID(1), GeomNode(deepcopy(geom)))
    a = AssemblyNode(ConstructionBots.AssemblyID(1), GeomNode(deepcopy(geom)))
    t = TransportUnitNode(ConstructionBots.node_id(a))

    s = Set{ConstructionBots.AbstractID}()
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
        push!(s, ConstructionBots.node_id(n))
    end
end
