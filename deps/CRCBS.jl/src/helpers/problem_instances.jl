export init_mapf_problem

function init_mapf_problem(graph,start_vtxs,goal_vtxs,
        cost_model=SumOfTravelTime(),
        heuristic = PerfectHeuristic(graph,start_vtxs,goal_vtxs),
        args...)
    env = CBSEnv.LowLevelEnv(
        graph = graph,
        cost_model = cost_model,
        heuristic = heuristic
    )
    starts = map(s->CBSEnv.State(vtx=s,t=0),start_vtxs)
    goals = map(s->CBSEnv.State(vtx=s,t=0),goal_vtxs)
    MAPF(env,starts,goals)
end

export
    init_mapf_1,
    init_mapf_2,
    init_mapf_3,
    init_mapf_4,
    init_mapf_5,
    init_mapf_6,
    init_mapf_7,
    init_mapf_8

function init_mapf_1(args...)
    vtx_grid = initialize_dense_vtx_grid(4,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    starts = [1,4]
    goals = [13,16]
    graph = construct_factory_env_from_vtx_grid(vtx_grid)
    init_mapf_problem(graph,starts,goals,args...)
end

"""
    switch places
"""
function init_mapf_2(args...)
    vtx_grid = initialize_dense_vtx_grid(4,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    starts = [1,4]
    goals = [4,1]
    graph = construct_factory_env_from_vtx_grid(vtx_grid)
    init_mapf_problem(graph,starts,goals,args...)
end

"""
    congested
"""
function init_mapf_3(args...)
    vtx_grid = initialize_dense_vtx_grid(4,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    starts = [1,2,3,4,5,6,7,8]
    goals = [13,14,15,16,9,10,11,12]
    graph = construct_factory_env_from_vtx_grid(vtx_grid)
    init_mapf_problem(graph,starts,goals,args...)
end

"""
    almost switch corners. With the fat path heuristic, the paths should be:
    - Robot 1: [1,5,9,13,14,15]
    - Robot 2: [16,12,8,4,3,2]
"""
function init_mapf_4(args...)
    vtx_grid = initialize_dense_vtx_grid(4,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    starts = [1,16]
    goals = [15,2]
    graph = construct_factory_env_from_vtx_grid(vtx_grid)
    init_mapf_problem(graph,starts,goals,args...)
end

"""
    init_mapf_5

PIBT demo from paper. CBS fails to solve it even with 80000 iterations.
"""
function init_mapf_5(args...)
    vtx_grid = initialize_dense_vtx_grid(2,4)
    #  1   2   3   4
    #  5   6   7   8
    starts = [2,5,6,3,4,7,8]
    goals = [4,8,5,7,8,4,3]
    graph = construct_factory_env_from_vtx_grid(vtx_grid)
    init_mapf_problem(graph,starts,goals,args...)
end

"""
    PIBT demo from paper
"""
function init_mapf_6(args...)
    vtx_grid = initialize_dense_vtx_grid(3,3)
    # 1  2  3
    # 4  5  6
    # 7  8  9
    starts = [2,4]
    goals = [8,6]
    graph = construct_factory_env_from_vtx_grid(vtx_grid)
    init_mapf_problem(graph,starts,goals,args...)
end


"""
    init_mapf_7

Robot 3 has to get past robots 1 and 2.
"""
function init_mapf_7(args...)
    vtx_grid = initialize_dense_vtx_grid(2,3)
    # 1  2  3
    # 4  5  6
    starts = [2,4,1]
    goals = [2,4,6]
    graph = construct_factory_env_from_vtx_grid(vtx_grid)
    init_mapf_problem(graph,starts,goals,args...)
end


"""
    init_mapf_8

Robots 1 and 2 have to get past robots 3 and 4. Requires about 660 iterations of
regular CBS.
"""
function init_mapf_8(args...)
    vtx_grid = initialize_dense_vtx_grid(2,3)
    # 1  2  3
    # 4  5  6
    starts = [1,4,2,5]
    goals = [6,3,2,5]
    graph = construct_factory_env_from_vtx_grid(vtx_grid)
    init_mapf_problem(graph,starts,goals,args...)
end

export mapf_test_problems

mapf_test_problems() = [
    init_mapf_1,
    init_mapf_2,
    init_mapf_3,
    init_mapf_4,
    init_mapf_5,
]
