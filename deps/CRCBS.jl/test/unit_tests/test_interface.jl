# Tuple Tests
let
    basetup = (1,2.0,3,4.0,5,6.0)
    for j in 1:length(basetup)
        tup = tuple(basetup[1:j])
        for i in 1:length(tup)
            @test typemax(tup)[i] == typemax(typeof(tup[i]))
            @test typemin(tup)[i] == typemin(typeof(tup[i]))
        end
    end
    @test (1,0,0,0,0) < 2
    @test (1,0,0,0,0) <= 1
    @test (1,0,0,0,0) >= 1
    @test (1,0,0,0,0) > 0

    @test 2 > (1,0,0,0,0)
    @test 1 >= (1,0,0,0,0)
    @test 1 <= (1,0,0,0,0)
    @test 0 < (1,0,0,0,0)
end
let
    S = GraphState
    A = GraphAction
    P = PathNode{S,A}

    Path([P()])
    p = Path{S,A,Float64}()
    @test state_type(p) == S
    @test action_type(p) == A
    @test node_type(p) == P
    @test cost_type(p) == Float64
    get(p,1,node_type(p))
    get(p,1)
    # @test get_cost(p) == 0
    @test length(p.path_nodes) == 0
    push!(p,P())
    @test length(p.path_nodes) == 1
    p = cat(p,P())
    @test length(p.path_nodes) == 2
    @test typeof(p[1]) == P
    p[1] = P()
    @test length(p) == length(p.path_nodes)
    p1 = copy(p)
    @test get_cost(p1) == get_cost(p)
    @test length(p1) == length(p)
end
let
    S = GraphState
    A = GraphAction
    P = PathNode{S,A}

    p = Path{S,A,Float64}(
        s0 = S(1,0)
    )

    get_path_node(p,1)
    get_path_node(p,5)
    get_a(p,1)
    @test length(extend_path(p,5)) == 5
    @test get_end_index(extend_path(p,5)) == 5
    extend_path!(p,5)
    @test length(p) == 5
    @test get_end_index(p) == 5

    get_initial_state(p)
    get_final_state(p)

    cost_model = TravelTime()
    solution = LowLevelSolution(
        cost_model = TravelTime(),
        paths = [Path{S,A,cost_type(cost_model)}()],
        costs = [0.0]
    )
    state_type(solution)
    action_type(solution)
    cost_type(solution)
    node_type(solution)
    copy(solution)
    get_paths(solution)
    get_path_costs(solution)
    get_cost(solution)
    get_cost_model(solution)
    set_solution_path!(solution,get_paths(solution)[1],1)
    set_path_cost!(solution,2.0,1)
end
