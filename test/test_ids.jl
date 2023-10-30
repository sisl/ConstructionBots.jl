let
    geom = LazySets.Ball2(zeros(SVector{3,Float64}), 1.0)
    o = ConstructionBots.ObjectNode(ConstructionBots.ObjectID(1), ConstructionBots.GeomNode(deepcopy(geom)))
    r = ConstructionBots.RobotNode(ConstructionBots.RobotID(1), ConstructionBots.GeomNode(deepcopy(geom)))
    a = ConstructionBots.AssemblyNode(ConstructionBots.AssemblyID(1), ConstructionBots.GeomNode(deepcopy(geom)))
    t = ConstructionBots.TransportUnitNode(ConstructionBots.node_id(a))

    s = Set{ConstructionBots.AbstractID}()
    for n in [
        ConstructionBots.ObjectStart(o),
        ConstructionBots.RobotStart(r),
        ConstructionBots.RobotGo(r),
        ConstructionBots.AssemblyStart(a),
        ConstructionBots.AssemblyComplete(a),
        ConstructionBots.LiftIntoPlace(a),
        ConstructionBots.LiftIntoPlace(o),
        ConstructionBots.FormTransportUnit(t),
        ConstructionBots.TransportUnitGo(t),
        ConstructionBots.DepositCargo(t),
    ]
        push!(s, ConstructionBots.node_id(n))
    end
end
