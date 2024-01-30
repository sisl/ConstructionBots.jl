function create_node(node_type, id)
    geom = LazySets.Ball2(zeros(SVector{3,Float64}), 1.0)
    return node_type(id, ConstructionBots.GeomNode(deepcopy(geom)))
end

let
    # Create nodes
    object = create_node(ConstructionBots.ObjectNode, ConstructionBots.ObjectID(1))
    robot = create_node(ConstructionBots.RobotNode, ConstructionBots.RobotID(1))
    assembly = create_node(ConstructionBots.AssemblyNode, ConstructionBots.AssemblyID(1))
    transport_unit = ConstructionBots.TransportUnitNode(ConstructionBots.node_id(assembly))

    # Initialize a set for node IDs
    node_ids = Set{ConstructionBots.AbstractID}()

    # Populate with node IDs
    node_ids = Set([
        ConstructionBots.node_id(x) for x in [
            ConstructionBots.ObjectStart(object),
            ConstructionBots.RobotStart(robot),
            ConstructionBots.RobotGo(robot),
            ConstructionBots.AssemblyStart(assembly),
            ConstructionBots.AssemblyComplete(assembly),
            ConstructionBots.LiftIntoPlace(assembly),
            ConstructionBots.LiftIntoPlace(object),
            ConstructionBots.FormTransportUnit(transport_unit),
            ConstructionBots.TransportUnitGo(transport_unit),
            ConstructionBots.DepositCargo(transport_unit),
        ]
    ])
end
