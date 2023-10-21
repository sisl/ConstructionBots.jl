let
    ObjectID()
    RobotID()
    LocationID()
    ActionID()
    OperationID()

    get_id(ObjectID())

    @test ObjectID(1) < ObjectID(2)
    @test ObjectID(2) > ObjectID(1)
    @test ObjectID(1) + 1 == ObjectID(2)
    @test ObjectID(1) == ObjectID(2) - 1
end
let
    ROBOT_AT(RobotID(1),LocationID(2))
    ROBOT_AT(1,2)
    get_robot_id(ROBOT_AT(1,2))
end
let
    OBJECT_AT(1,2)
    OBJECT_AT(1,[2])
    OBJECT_AT(1,LocationID(2))
    OBJECT_AT(1,[LocationID(2)])

    get_object_id(OBJECT_AT(1,2))
    get_location_id(OBJECT_AT(1,2))
    get_location_ids(OBJECT_AT(1,2))
end
let
    op = Operation()
    set_precondition!(op,OBJECT_AT(1,2))
    @test TaskGraphs.get_dropoff(op,ObjectID(1)) == LocationID(2)
    get_location_id(op)
    get_operation_id(op)
    get_input_ids(op)
    get_output_ids(op)
    duration(op)
    preconditions(op)
    postconditions(op)
    get_id(op)
end
let
    GO()
    node = GO(1,2,3)
    get_initial_location_id(node)
    get_destination_location_id(node)
    get_robot_id(node)
    # GO does not have a single location, so this method should fail
    @test_throws MethodError get_location_id(node)
    @test_throws MethodError get_object_id(node)
end
let
    CARRY()
    node = CARRY(1,2,3,4)
    get_initial_location_id(node)
    get_destination_location_id(node)
    get_object_id(node)
    get_robot_id(node)
    # CARRY does not have a single location, so this method should fail
    @test_throws MethodError get_location_id(node)
end
let
    COLLECT()
    node = COLLECT(1,2,3)
    get_initial_location_id(node)
    get_destination_location_id(node)
    get_location_id(node)
    get_object_id(node)
    get_robot_id(node)
end
let
    DEPOSIT()
    node = DEPOSIT(1,2,3)
    get_initial_location_id(node)
    get_destination_location_id(node)
    get_location_id(node)
    get_object_id(node)
    get_robot_id(node)
end
let
    for node in [OBJECT_AT(1,2),ROBOT_AT(1,2),GO(),COLLECT(),CARRY(),DEPOSIT(),Operation()]
        required_predecessors(node)
        required_successors(node)
        eligible_predecessors(node)
        eligible_successors(node)
        resources_reserved(node)
        for node2 in [OBJECT_AT(1,2),ROBOT_AT(1,2),GO(),COLLECT(),CARRY(),DEPOSIT(),Operation()]
            matches_template(typeof(node),node2)
            matches_template((typeof(node),typeof(node2)),node2)
        end
    end
end
let
    for (T,nodes) in [
        AbstractPlanningPredicate=>[OBJECT_AT(1,2),ROBOT_AT(1,2),GO(),COLLECT(),CARRY(),DEPOSIT(),Operation()],
        AbstractSingleRobotAction=>[GO(),COLLECT(),CARRY(),DEPOSIT()],
        AbstractSingleRobotAction=>[CUB_GO(),CUB_COLLECT(),CUB_CARRY(),CUB_DEPOSIT()],
        ]
        for node in nodes
            @test matches_template(T,node)
            @test matches_template(T,typeof(node))
            @test matches_template((T,T),typeof(node))
            @test matches_template(node,node)
            @test matches_template((node,node),node)
            @test matches_template((node,GO()),node)
            @test matches_template(typeof(node),node)
            @test matches_template((typeof(node),typeof(node)),node)
            @test matches_template(T,ScheduleNode(RobotID(-1),node,PathSpec()))

            @test !matches_template(1,node)
            @test !matches_template((1,2),node)
            @test !matches_template(1,ScheduleNode(RobotID(-1),node,PathSpec()))
        end
    end
end
let
    @test split_node(GO(1,2,3),LocationID(4))[1] == GO(1,2,4)
    @test split_node(GO(1,2,3),LocationID(4))[2] == GO(1,4,3)
    @test split_node(CARRY(1,1,2,3),LocationID(4))[1] == CARRY(1,1,2,4)
    @test split_node(CARRY(1,1,2,3),LocationID(4))[2] == CARRY(1,1,4,3)
    @test split_node(COLLECT(1,2,3),LocationID(4))[1] == COLLECT(1,2,4)
    @test split_node(COLLECT(1,2,3),LocationID(4))[2] == COLLECT(1,2,4)
    @test split_node(DEPOSIT(1,2,3),LocationID(4))[1] == DEPOSIT(1,2,4)
    @test split_node(DEPOSIT(1,2,3),LocationID(4))[2] == DEPOSIT(1,2,4)
end
let
    for n in [GO(),CARRY(),COLLECT(),DEPOSIT()]
        @test TaskGraphs.replace_robot_id(n,RobotID(2)).r == RobotID(2)
    end
end
let
    n1 = GO(1,2,3)
    n2 = GO(-1,-1,4)
    n = align_with_predecessor(n2,n1)
    @test n.r == n1.r
    @test n.x1 == n1.x2
    @test n.x2 == n2.x2
end
let
    n1 = ROBOT_AT(1,2)
    n2 = GO(-1,-1,3)
    n = align_with_predecessor(n2,n1)
    @test n.r == n1.r
    @test n.x1 == n1.x
end
let
    n1 = DEPOSIT(1,2,3)
    n2 = GO(-1,3,4)
    n = align_with_predecessor(n2,n1)
    @test n.r == n1.r
    @test n.x1 == n1.x
end
let
    n1 = GO(1,2,4)
    n2 = COLLECT(-1,3,4)
    n = align_with_predecessor(n2,n1)
    @test n.r == n1.r
    @test n.x == n1.x2
end
let
    n1 = OBJECT_AT(3,4)
    n2 = COLLECT(-1,3,4)
    n = align_with_predecessor(n2,n1)
    @test n.o == n1.o
    @test n.x == n1.x[1]
end
let
    n1 = COLLECT(1,2,3)
    n2 = CARRY(-1,2,3,4)
    n = align_with_predecessor(n2,n1)
    @test n.o == n1.o
    @test n.x1 == n1.x
    @test n.r == n1.r
end
let
    n1 = CARRY(1,2,3,4)
    n2 = DEPOSIT(-1,2,4)
    n = align_with_predecessor(n2,n1)
    @test n.o == n1.o
    @test n.x == n1.x2
    @test n.r == n1.r
end
let
    TEAM_CARRY(instructions=[CARRY(1,2,3,4),CARRY(2,3,4,4)],shape=(2,1))

    sequence = [
        TEAM_COLLECT(instructions=[COLLECT(1,2,3),COLLECT(2,3,4)],shape=(2,1)),
        TEAM_CARRY(instructions=[CARRY(-1,2,3,4),CARRY(-1,3,4,5)],shape=(2,1)),
        TEAM_DEPOSIT(instructions=[DEPOSIT(-1,2,4),DEPOSIT(-1,3,5)],shape=(2,1)),
    ]
    @test all(map(p->get_id(get_robot_id(p)) == -1, sequence[2].instructions))
    sequence[2] = align_with_predecessor(sequence[2],sequence[1])
    @test all(map(p->get_id(get_robot_id(p)) != -1, sequence[2].instructions))

    @test all(map(p->get_id(get_robot_id(p)) == -1, sequence[3].instructions))
    sequence[3] = align_with_predecessor(sequence[3],sequence[2])
    @test all(map(p->get_id(get_robot_id(p)) != -1, sequence[3].instructions))
end
let
    @test validate_edge(ROBOT_AT(1,2),ROBOT_AT(1,2)) == false
    @test validate_edge(ROBOT_AT(1,2),GO(1,2,3))
    @test validate_edge(GO(1,2,3),GO(1,3,3))
    @test validate_edge(GO(1,2,3),COLLECT(1,4,3))
    @test validate_edge(COLLECT(1,2,3),CARRY(1,2,3,4))
    @test validate_edge(CARRY(1,2,3,4),COLLECT(1,2,3)) == false
    @test validate_edge(CARRY(1,2,3,4),DEPOSIT(1,2,4))
    @test validate_edge(DEPOSIT(1,2,4),CARRY(1,2,3,4)) == false
    @test validate_edge(DEPOSIT(1,2,4),GO(1,4,3))
    @test validate_edge(DEPOSIT(1,2,4),DEPOSIT(1,2,4))
    @test validate_edge(COLLECT(1,2,4),DEPOSIT(1,2,4))
    @test validate_edge(COLLECT(1,2,4),COLLECT(1,2,4))
end
# Collaborative
# let
#     vtx_grid = initialize_dense_vtx_grid(4,4)
#     env_graph = construct_factory_env_from_vtx_grid(vtx_grid)

#     sched = OperatingSchedule()

#     object_def = TaskGraphs.LargeObjectDef()
#     object_node = TaskGraphs.LARGE_OBJECT_AT(ObjectID(1),LocationID(1),object_def)
#     o_id = get_object_id(object_node)
#     x0 = get_initial_location_id(object_node)
#     xF = LocationID(4)

#     # Still need to take care of making the PathSpecs 
#     o = add_node!(sched,object_node,o_id)
#     collect_node = add_node!(sched,
#         TaskGraphs.init_collaborative_action(env_graph,COLLECT(-1,o_id,x0),object_def),
#         get_unique_id(ActionID),
#         )
#     add_edge!(sched,o,collect_node)
#     carry_node = add_node!(sched,
#         TaskGraphs.init_collaborative_action(env_graph,CARRY(-1,o_id,x0,xF),object_def),
#         get_unique_id(ActionID),
#         )
#     add_edge!(sched,collect_node,carry_node)
#     deposit_node = add_node!(sched,
#         TaskGraphs.init_collaborative_action(env_graph,DEPOSIT(-1,o_id,xF),object_def),
#         get_unique_id(ActionID),
#         )
#     add_edge!(sched,collect_node,deposit_node)

    
#     collect_node = COLLECT(-1,o_id,x0)

#     carry_node = CARRY(-1,o_id,x0,xF)
#     deposit_node = DEPOSIT(-1,o_id,xF)
#     for n in [collect_node,carry_node,deposit_node]
#         TaskGraphs.init_collaborative_action(env_graph,n,object_def)
#     end

# end