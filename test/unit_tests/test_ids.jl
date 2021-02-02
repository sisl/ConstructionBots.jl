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
# test transform tree embedded in schedule
let

    sched = NGraph{DiGraph,ConstructionPredicate,AbstractID}()

    geom = Ball2(zeros(SVector{3,Float64}),1.0)
    o = ObjectNode(ObjectID(1),GeomNode(deepcopy(geom)))
    r = RobotNode(RobotID(1),GeomNode(deepcopy(geom)))
    a = AssemblyNode(AssemblyID(1),GeomNode(deepcopy(geom)))
    t = TransportUnitNode(node_id(a))

    O = ObjectStart(o,TransformNode())
    R = RobotStart(r,TransformNode())
    A = AssemblyStart(a,TransformNode())
    AC = AssemblyComplete(A)
    @test start_config(A) === start_config(AC)

    l = add_node!(sched,LiftIntoPlace(o,TransformNode(),TransformNode()))
    set_parent!(goal_config(l),start_config(l))

    d = add_node!(sched,    DepositCargo(TransportUnitNode(o),TransformNode(),TransformNode()))
    set_parent!(goal_config(d),start_config(l))
    set_parent!(start_config(d),goal_config(d))

    tgo = add_node!(sched,  TransportUnitGo(entity(node_val(d)),TransformNode(),TransformNode()))
    set_parent!(goal_config(tgo),start_config(d))

    f = add_node!(sched,    FormTransportUnit(entity(node_val(d)),TransformNode(),TransformNode()))
    set_parent!(start_config(tgo),goal_config(f))

end