# test reservation table
@time @testset "ReservationTable" begin
let
    table = ReservationTable{Int}(10)
    @test is_valid(table)
    for (interval,val) in [
        ((2,4), true),
        ((0,1), true),
        ((5,6), true),
        ]
        @test is_reserved(table,ResourceReservation(1,2,interval)) == !val
        @test reserve!(table,ResourceReservation(1,2,interval)) == val
        @test is_reserved(table,ResourceReservation(1,2,interval))
    end
    @test is_valid(table)
end
let
    mapf = init_mapf_1()
    env = build_env(mapf,initialize_root_node(mapf),1)
    table = ReservationTable{Float64}(num_states(mapf)+num_actions(mapf))
    S = state_type(mapf)
    A = action_type(mapf)
    n = PathNode(S(1,0),A(Edge(1,2),1),S(2,1))
    create_reservations(env,n)
    @test reserve!(table,env,get_s(n),get_a(n),get_sp(n))
    s2 = get_sp(n)
    a2 = CRCBS.wait(env,get_sp(n))
    sp2 = get_next_state(env,s2,a2)
    n2 = PathNode(s2,a2,sp2)
    @test reserve!(table,env,get_s(n2),get_a(n2),get_sp(n2))
end
let
    mapf = init_mapf_1()
    env = MetaAgentCBS.construct_team_env(
        [build_env(mapf,initialize_root_node(mapf),1)],
        [1]
        )
    S = state_type(mapf)
    A = action_type(mapf)
    n = PathNode(
        MetaAgentCBS.State([S(1,0)]),
        MetaAgentCBS.Action([A(Edge(1,2),1)]),
        MetaAgentCBS.State([S(2,1)])
        )
    s2 = get_sp(n)
    a2 = CRCBS.wait(env,get_sp(n))
    sp2 = get_next_state(env,s2,a2)
    n2 = PathNode(s2,a2,sp2)
    let
        table = ReservationTable{Float64}(num_states(mapf)+num_actions(mapf))
        @test reserve!(table,env,get_s(n),get_a(n),get_sp(n))
        @test !reserve!(table,env,get_s(n),get_a(n),get_sp(n))
        @test reserve!(table,env,get_s(n2),get_a(n2),get_sp(n2))
        @test !reserve!(table,env,get_s(n2),get_a(n2),get_sp(n2))
    end
    let
        table = PIBTReservationTable(num_states(mapf),num_actions(mapf))
        @test reserve!(table,env,get_s(n),get_a(n),get_sp(n))
        @test !reserve!(table,env,get_s(n),get_a(n),get_sp(n))
    end
end
let
    table = ReservationTable{Int}(10)
    @test is_valid(table)
    for (interval,val) in [
            ((1,2), true),
            ((1,2), false),
            ((0,1), true),
            ((0,1), false),
            ((2,3), true),
            ((2,3), false),
            ((4,7), true),
            ((5,8), false),
            ((3,6), false),
            ((5,6), false),
            ((2,6), false),
        ]
        @test is_reserved(table,ResourceReservation(1,2,interval)) == !val
        @test reserve!(table,ResourceReservation(1,2,interval)) == val
        @test is_reserved(table,ResourceReservation(1,2,interval))
        # @test is_available(table,ResourceReservation(1,2,interval))
        @test 2 in reserved_by(table,ResourceReservation(1,2,interval))
    end
    @test is_valid(table)
end
let
    table = ReservationTable{Float64}(10)
    @test is_valid(table)
    for (interval, val) in [
        ((1.0, 2.0), true),
        ((1.0, 2.0), false),
        ((0.0, 1.0), true),
        ((0.0, 1.0), false),
        ((2.0, 3.0), true),
        ((2.0, 3.0), false),
        ((4.0, 5.0), true),
        ((3.5, 4.5), false),
        ((4.5, 5.5), false),
        ((4.5, 4.6), false),
        ((3.5, 5.5), false),
    ]
        @test is_reserved(table, ResourceReservation(1, 2, interval)) == !val
        @test reserve!(table, ResourceReservation(1, 2, interval)) == val
        @test is_reserved(table, ResourceReservation(1, 2, interval))
    end
    @test is_valid(table)
end
end # end testset

# For testing Priority Inheritance with Backtracking (PIBT)
@time @testset "PIBT" begin
let
    mapf = init_mapf_1()
    solver = PIBTPlanner{Float64}()
    cache = CRCBS.pibt_init_cache(solver,mapf)
end
# test PIBT on mapf test problems
let
    solver = PIBTPlanner{Float64}()
    set_iteration_limit!(solver,20)
    set_verbosity!(solver,0)
    for f in mapf_test_problems()
        mapf = f()
        reset_solver!(solver)
        solution, valid = pibt!(solver, mapf)
        @test valid
        reset_solver!(solver)
        solution, cost = solve!(solver, mapf)
        @test cost != typemax(cost_type(mapf))
    end
end
# test PIBT with ragged plans on mapf test problems
let
    solver = PIBTPlanner{Float64}()
    set_iteration_limit!(solver,20)
    set_verbosity!(solver,0)
    for f in mapf_test_problems()
        mapf = f()
        reset_solver!(solver)
        solution, valid = pibt!(solver, mapf)
        @test valid
        # remove the path of robot 1 and replan
        p1 = get_paths(solution)[1]
        @show length(p1)
        # for i in reverse(2:length(p1))
        #     deleteat!(p1.path_nodes,i)
        # end
        empty!(p1.path_nodes)
        set_cost!(p1,get_initial_cost(mapf))
        p2 = deepcopy(get_paths(solution)[2])
        # replan with the ragged plan
        reset_solver!(solver)
        # cache = CRCBS.pibt_init_cache(solver,mapf,solution)
        # @show map(string, cache.states)
        # @show map(string, cache.actions)
        # @show cache.active_countdowns
        # CRCBS.pibt_update_cache!(solver,mapf,cache)
        # @show cache.active_countdowns
        # @show map(p->length(p), get_paths(cache.solution))
        solution, valid = pibt!(solver, mapf)
        @test valid
        p2b = get_paths(solution)[2]
        @test length(p2) == length(p2b)
        @test get_cost(p2) == get_cost(p2b)
        for (n2a,n2b) in zip(p2.path_nodes,p2b.path_nodes)
            @test get_s(n2a) == get_s(n2b)
            @test get_a(n2a) == get_a(n2b)
            @test get_sp(n2a) == get_sp(n2b)
        end
    end
end
# Test PIBT on problem from paper
let
    solver = PIBTPlanner{Float64}()
    mapf = init_mapf_5()
    set_max_iterations!(solver,50)
    # set_verbosity!(solver,4)
    solution, status = pibt!(solver,mapf)
end

end # end testset
