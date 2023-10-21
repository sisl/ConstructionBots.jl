export
    init_env_1,
    init_env_2,
    init_env_3,
    init_env_small

init_env_1() = construct_regular_factory_world(;
    n_obstacles_x=2,
    n_obstacles_y=2,
    obs_width = [4;4], # obs_w = 8/n
    obs_offset = [4;4],
    env_pad = [1;1],
    # env_offset = [1,1],
    env_scale = 1,
    exclude_from_free = true,
)
init_env_2() = construct_regular_factory_world(;
    n_obstacles_x=4,
    n_obstacles_y=4,
    obs_width = [2;2],
    obs_offset = [2;2],
    env_pad = [1;1],
    # env_offset = [1,1],
    env_scale = 1,
    exclude_from_free = true,
)
init_env_3() = construct_regular_factory_world(;
    n_obstacles_x=8,
    n_obstacles_y=8,
    obs_width = [1;1],
    obs_offset = [1;1],
    env_pad = [1;1],
    # env_offset = [1,1],
    env_scale = 1,
    exclude_from_free = true,
)

init_env_small() = construct_regular_factory_world(;
    n_obstacles_x=2,
    n_obstacles_y=2,
    obs_width = [2;2],
    obs_offset = [2;2],
    env_pad = [1;1],
    env_scale = 1,
    exclude_from_free = true,
)

export
    pctapf_problem,
    pctapf_problem_1,
    pctapf_problem_2,
    pctapf_problem_3,
    pctapf_problem_4,
    pctapf_problem_5,
    pctapf_problem_6,
    pctapf_problem_7,
    pctapf_problem_8

function get_zero_initial_conditions(G,N)
    to0_ = Dict{Int,Float64}(v=>0.0 for v in get_all_root_nodes(G))
    tr0_ = Dict{Int,Float64}(i=>0.0 for i in 1:N)
    return to0_, tr0_
end

function pctapf_problem(r0,config;Δt_op=0)
    tasks = config.tasks
    ops = config.ops
    s0 = map(t->t.first,tasks)
    sF = map(t->t.second,tasks)
    project_spec, _ = empty_pctapf_problem(r0,s0,sF)
    for op in ops
        add_operation!(project_spec,construct_operation(project_spec,-1,
            op.inputs,op.outputs,Δt_op))
    end
    def = SimpleProblemDef(project_spec,r0,s0,sF,config.shapes)
end

function empty_pctapf_problem(r0,s0,sF)
    object_ICs = [OBJECT_AT(o,x) for (o,x) in enumerate(s0)] # initial_conditions
    object_FCs = [OBJECT_AT(o,x) for (o,x) in enumerate(sF)] # final conditions
    robot_ICs = [ROBOT_AT(r,x) for (r,x) in enumerate(r0)]
    project_spec = ProjectSpec(object_ICs,object_FCs)
    project_spec, robot_ICs
end

function pctapf_problem(
        solver,
        sched::OperatingSchedule,
        problem_spec::ProblemSpec,
        env_graph,
        args...
    )
    env = construct_search_env(
        solver,
        sched,
        problem_spec,
        env_graph
        )
    prob = PC_TAPF(env)
    return post_process_problem_type(solver,prob)
end
"""
    post_process_problem_type(solver,prob)

An overridable helper function for potentially modifying prob based on solver.
Default behavior is to return `prob` with no changes.
"""
post_process_problem_type(solver,prob) = prob
function pctapf_problem(solver,spec::ProjectSpec,env,robot_ics,prob_spec=ProblemSpec(D=env))
    sched = construct_partial_project_schedule(spec,prob_spec,robot_ics)
    return pctapf_problem(solver,sched,prob_spec,env)
end
function PC_TAPF(solver,def::SimpleProblemDef,env::GridFactoryEnvironment)
    sched, prob_spec = construct_task_graphs_problem(def,env)
    pctapf = pctapf_problem(solver,sched,prob_spec,env)
    return pctapf
end
function PC_MAPF(solver,def::SimplePCMAPFDef,env::GridFactoryEnvironment)
    prob = PC_TAPF(solver,def.pctapf_def,env)
    sched = get_schedule(get_env(prob))
    prob_spec = get_problem_spec(get_env(prob))
    apply_assignment_dict!(sched,def.assignments,prob_spec)
    process_schedule!(sched)
    return PC_MAPF(get_env(prob))
end

function pcta_problem(
        # project_spec::ProjectSpec,
        sched::OperatingSchedule,
        problem_spec::ProblemSpec,
        # robot_ICs,
        env_graph,
        primary_objective = MakeSpan()
    )
    cache=initialize_planning_cache(sched)
    cost_model = typeof(primary_objective)(sched,cache)
    heuristic_model = NullHeuristic()
    layers = construct_environment_layer_dict(sched,env_graph,problem_spec)
    env = SearchEnv(
        schedule=sched,
        cache=cache,
        env_layers=layers,
        cost_model = cost_model,
        heuristic_model = heuristic_model
        )
    PC_TA(env)
end

function PC_TA(def::SimpleProblemDef,env::GridFactoryEnvironment,objective=MakeSpan())
    sched, prob_spec = construct_task_graphs_problem(def,env)
    pcta = pcta_problem(sched,prob_spec,env,objective)
end

# Reusable problem initializers for testing
"""
    pctapf_problem_1

Optimal MakeSpan = 5
Optimal SumOfMakeSpans = 5
"""
function pctapf_problem_1(;cost_function=SumOfMakeSpans(),verbose=false,Δt_op=0,Δt_collect=[0,0,0],Δt_deposit=[0,0,0])
    N = 2                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    r0 = [1,4]
    s0 = [5,8,14]
    sF = [13,12,15]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )
    project_spec, robot_ICs = empty_pctapf_problem(r0,s0,sF)
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], Δt_op))
    assignment_dict = Dict(1=>[1,3],2=>[2])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    sched, problem_spec = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function,Δt_collect=Δt_collect,Δt_deposit=Δt_deposit)
    return sched, problem_spec, env_graph, assignment_dict
end

"""
    pctapf_problem_2(;cost_function=SumOfMakeSpans(),verbose=false)

In this problem robot 1 will first do [1-9-17], then [17-21-35]
robot 2 will do [4-12-32]. The key thing is that robot 1 will need to wait
until robot 2 is finished before robot 1 can do its second task.

Optimal paths:
Optimal MakeSpan = 8
Optimal SumOfMakeSpans = 8
"""
function pctapf_problem_2(;cost_function=SumOfMakeSpans(),verbose=false)
    N = 2                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(8,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    # 17  18  19  20
    # 21  22  23  24
    # 25  26  27  28
    # 29  30  31  32
    r0 = [1,4]
    s0 = [9,12,21]
    sF = [17,32,25]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )
    project_spec, robot_ICs = empty_pctapf_problem(r0,s0,sF)

    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],0))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], 0))
    assignment_dict = Dict(1=>[1,3],2=>[2])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    sched, problem_spec = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function)
    return sched, problem_spec, env_graph, assignment_dict
end

"""
    pctapf_problem_3(;cost_function=SumOfMakeSpans(),verbose=false,Δt_op=0,Δt_collect=[0,0,0,0],Δt_deposit=[0,0,0,0])

In this problem robot 2 will need to yield to let robot 1 through.
First operation:
    robot 1 does [2-2-30]
Second operation:
    robot 1 does [30-30-32]
    robot 2 does [5-7-8]
Third operation:
    robot 2 does [8-12-16]
"""
function pctapf_problem_3(;cost_function=SumOfMakeSpans(),verbose=false,Δt_op=0,Δt_collect=[0,0,0,0],Δt_deposit=[0,0,0,0])
    N = 2                  # num robots
    M = 4                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(8,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    # 17  18  19  20
    # 21  22  23  24
    # 25  26  27  28
    # 29  30  31  32
    r0 = [2,5]
    s0 = [2,30,7,12]
    sF = [30,32,8,16]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )
    project_spec, robot_ICs = empty_pctapf_problem(r0,s0,sF)

    add_operation!(project_spec,construct_operation(project_spec,-1,[1],[2],0.0))
    add_operation!(project_spec,construct_operation(project_spec,-1,[2,3],[4],0.0))
    add_operation!(project_spec,construct_operation(project_spec,-1,[4],  [], 0.0))
    assignment_dict = Dict(1=>[1,2],2=>[3,4])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    sched, problem_spec = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function,Δt_collect=Δt_collect,Δt_deposit=Δt_deposit)

    return sched, problem_spec, env_graph, assignment_dict
end


"""
    pctapf_problem_4(;cost_function=SumOfMakeSpans(),verbose=false)

In this problem the cost of the task assignment problem is lower than the
true cost (which requires that one of the robots is delayed by a single time
step)
First operation:
    robot 1 does [2-2-8]
    robot 2 does [4-4-6]
"""
function pctapf_problem_4(;cost_function=SumOfMakeSpans(),verbose=false)
    N = 2                  # num robots
    M = 2                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(3,3)
    # 1  2  3
    # 4  5  6
    # 7  8  9
    r0 = [2,4]
    s0 = [2,4]
    sF = [8,6]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )

    project_spec, robot_ICs = empty_pctapf_problem(r0,s0,sF)
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[],0))
    assignment_dict = Dict(1=>[1],2=>[2])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    sched, problem_spec = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function)

    return sched, problem_spec, env_graph, assignment_dict
end


"""
    pctapf_problem_5(;cost_function=SumOfMakeSpans(),verbose=false)

In this problem the robots try to pass through each other in such a way that
an edge conflict is generated.

First operation:
    robot 1 does [3-11]
    robot 2 does [15-7]
"""
function pctapf_problem_5(;cost_function=SumOfMakeSpans(),verbose=false)
    N = 2                  # num robots
    M = 2                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,4)
    # 1   2   3   4
    # 5   6   7   8
    # 9  10  11  12
    # 13  14  15  16
    r0 = [3,15]
    s0 = [3,15]
    sF = [11,7]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )

    project_spec, robot_ICs = empty_pctapf_problem(r0,s0,sF)
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[],0))
    assignment_dict = Dict(1=>[1],2=>[2])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    sched, problem_spec = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function)

    return sched, problem_spec, env_graph, assignment_dict
end

"""
    pctapf_problem_6(;cost_function=SumOfMakeSpans(),verbose=false,Δt_op=1,Δt_collect=[0,0,0],Δt_deposit=[0,0,0])

Identical to `pctapf_problem_2`, but process time is non-zero.
In this problem robot 1 will first do [1-5-9], then [9-13-17]
robot 2 will do [4-8-32]. The key thing is that robot 1 will need to wait
until robot 2 is finished before robot 1 can do its second task
"""
function pctapf_problem_6(;cost_function=SumOfMakeSpans(),verbose=false,Δt_op=1,Δt_collect=[0,0,0],Δt_deposit=[0,0,0])
    N = 2                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(8,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    # 17  18  19  20
    # 21  22  23  24
    # 25  26  27  28
    # 29  30  31  32
    r0 = [1,4]
    s0 = [5,12,13]
    sF = [9,32,17]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )

    project_spec, robot_ICs = empty_pctapf_problem(r0,s0,sF)
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], Δt_op))
    assignment_dict = Dict(1=>[1,3],2=>[2])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    sched, problem_spec = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function,Δt_collect=Δt_collect,Δt_deposit=Δt_deposit)

    return sched, problem_spec, env_graph, assignment_dict
end


"""
    pctapf_problem_7(;cost_function=SumOfMakeSpans(),verbose=false,Δt_op=0,Δt_collect=[0,4,0],Δt_deposit=[0,0,0])

Robot 2 will have to sit and wait at the pickup station, meaning that robot 1 
will have to go around if robot 2 is on the critical path
"""
function pctapf_problem_7(;cost_function=SumOfMakeSpans(),verbose=false,Δt_op=0,Δt_collect=[0,4,0],Δt_deposit=[0,0,0])
    N = 2                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    r0 = [2,9]
    s0 = [6,10,15]
    sF = [14,12,16]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )

    project_spec, robot_ICs = empty_pctapf_problem(r0,s0,sF)
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], Δt_op))
    assignment_dict = Dict(1=>[1,3],2=>[2])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    sched, problem_spec = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function,Δt_collect=Δt_collect,Δt_deposit=Δt_deposit)
    return sched, problem_spec, env_graph, assignment_dict
end

"""
    pctapf_problem_8(;cost_function=SumOfMakeSpans(),verbose=false,Δt_op=0,Δt_collect=[0,0,0,0],Δt_deposit=[0,0,0,0])

Two-headed project. Robot 1 does the first half of the first head, and
robot 2 handles the first half of the second head, and then they swap.
Optimal MakeSpan = 8
Optimal SumOfMakeSpans = 16
"""
function pctapf_problem_8(;cost_function=SumOfMakeSpans(),verbose=false,Δt_op=0,Δt_collect=[0,0,0,0],Δt_deposit=[0,0,0,0])
    N = 2                  # num robots
    M = 4                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(8,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    # 17  18  19  20
    # 21  22  23  24
    # 25  26  27  28
    # 29  30  31  32
    r0 = [1,29]
    s0 = [5,25,12,24]
    sF = [8,28,9,21]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )

    project_spec, robot_ICs = empty_pctapf_problem(r0,s0,sF)
    add_operation!(project_spec,construct_operation(project_spec,-1,[1],[4],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[2],[3],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[4],[],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],[],Δt_op))
    assignment_dict = Dict(1=>[1,3],2=>[2,4])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    sched, problem_spec = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function,Δt_collect=Δt_collect,Δt_deposit=Δt_deposit)

    return sched, problem_spec, env_graph, assignment_dict
end

export pctapf_problem_9

"""
    pctapf_problem_9(;cost_function=SumOfMakeSpans(),verbose=false,Δt_op=0,Δt_collect=[0,0],Δt_deposit=[0,0])

Project with station-sharing. Station 5 needs to accessed by both robots for 
picking up their objects.
"""
function pctapf_problem_9(;cost_function=SumOfMakeSpans(),verbose=false,Δt_op=0,Δt_collect=[0,0],Δt_deposit=[0,0])
    N = 2                  # num robots
    M = 2                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    r0 = [1,13]
    s0 = [5,5]
    sF = [8,6]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )

    project_spec, robot_ICs = empty_pctapf_problem(r0,s0,sF)
    add_operation!(project_spec,construct_operation(project_spec,-1,[1],[],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[2],[],Δt_op))
    assignment_dict = Dict(1=>[1],2=>[2])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    sched, problem_spec = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function,Δt_collect=Δt_collect,Δt_deposit=Δt_deposit)

    return sched, problem_spec, env_graph, assignment_dict
end

export pctapf_problem_10

"""
    pctapf_problem_10(;cost_function=MakeSpan(),verbose=false,Δt_op=0,Δt_collect=[0,0,0,0,0,0],Δt_deposit=[0,0,0,0,0,0])

Motivation for backtracking in ISPS
The makespan optimal solution is T = 6. However, the optimistic schedule will 
always prioritize robot 2 over robot 1, causing robot 1 to get stuck waiting for 
3, 4, and 5 to pass all in a row. This creates an unavoidable delay in the 
schedule, leading to a +1 delay that will not be caught without backtracking in
ISPS. Hence, the solver will return a solution with T = 7.
"""
function pctapf_problem_10(;cost_function=MakeSpan(),verbose=false,Δt_op=0,Δt_collect=[0,0,0,0,0,0],Δt_deposit=[0,0,0,0,0,0])
    vtx_grid = initialize_dense_vtx_grid(10,4)
    #   1   2   3   4
    #   5   6   7   8
    #   9  10  11  12
    #  13  14  15  16
    #  17  18  19  20
    #  21  22  23  24
    #  25  26  27  28
    #  29  30  31  32
    #  33  34  35  36
    #  37  38  39  40

    #   .   .  (5)  .  
    #   .   .  (4)  .  
    #   .   .  (3)  .  
    #   .   .   .   .  
    #   .  (2)  .   .  
    #  (1) [1]  .  [6]
    #   .   .  [5]  .  
    #   .   .  [4]  .  
    #   .   .  [3]  .  
    #   .  [2]  .   .  

    r0 = [21,18,11, 7, 3]
    s0 = [21,18,11, 7, 3,22]
    sF = [22,38,35,31,27,24]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )

    project_spec, robot_ICs = empty_pctapf_problem(r0,s0,sF)
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2,3,4,5,6],[],Δt_op))
    assignment_dict = Dict(1=>[1,6],2=>[2],3=>[3],4=>[4],5=>[5])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    sched, problem_spec = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function,Δt_collect=Δt_collect,Δt_deposit=Δt_deposit)

    return sched, problem_spec, env_graph, assignment_dict
end

"""
    pctapf_problem_multi_backtrack(;cost_function=MakeSpan(),verbose=false,Δt_op=0,Δt_collect=[0,0,0,0,0,0],Δt_deposit=[0,0,0,0,0,0])

Motivation for backtracking in ISPS
The makespan optimal solution is T = 8, and requires that robots 2,3, and 4 
allow robot 1 to pass before them. However, the slack-based priority schedme of 
ISPS will always prioritize robot 2, 3, and 4 over robot 1, causing robot 1 to 
get stuck waiting for 5, 6, and 7 to pass all in a row. 
Without backtracking, CBS+ISPS will eventually return a plan with makespan T = 9.
With recursive backtracking (not just a single pass)
"""
function pctapf_problem_multi_backtrack(;cost_function=MakeSpan(),verbose=false,Δt_op=0,Δt_collect=[0,0,0,0,0,0],Δt_deposit=[0,0,0,0,0,0])
    vtx_grid = initialize_dense_vtx_grid(13,7)

    #   1   2   3   4   5   6   7
    #   8   9  10  11  12  13  14
    #  15  16  17  18  19  20  21
    #  22  23  24  25  26  27  28
    #  29  30  31  32  33  34  35
    #  36  37  38  39  40  41  42
    #  43  44  45  46  47  48  49
    #  50  51  52  53  54  55  56
    #  57  58  59  60  61  62  63
    #  64  65  66  67  68  69  70
    #  71  72  73  74  75  76  77
    #  78  79  80  81  82  83  84
    #  85  86  87  88  89  90  91

    #   .   .   .   .  (5)  .   . 
    #   .   .   .   .  (6)  .   . 
    #   .   .   .   .  (7)  .   . 
    #   .   .   .   .   .   .   . 
    #   .   .   .  (4)  .   .   . 
    #   .   .  (3)  .   .   .   . 
    #   .  (2)  .   .   .   .   . 
    #  (1) [1] [8] [9]  .  [10] .
    #   .   .   .   .  [5]  .   . 
    #   .   .   .   .  [6]  .   . 
    #   .   .   .  [4] [7]  .   . 
    #   .   .  [3]  .   .   .   . 
    #   .  [2]  .   .   .   .   . 
    assignment_dict = Dict(1=>[1,8,9,10],2=>[2],3=>[3],4=>[4],5=>[5],6=>[6],7=>[7])

    r0 = [50,44,38,32,5,12,19]
    s0 = [50,44,38,32,5,12,19,51,52,53]
    sF = [51,86,80,74,61,68,75,52,53,55]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )

    project_spec, robot_ICs = empty_pctapf_problem(r0,s0,sF)
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2,3,4,5,6,7,8,9,10],[],Δt_op))

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    sched, problem_spec = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function,Δt_collect=Δt_collect,Δt_deposit=Δt_deposit)

    return sched, problem_spec, env_graph, assignment_dict
end

export pctapf_problem_11

"""
    pctapf_problem_11

Requires collaborative transport: Robots 1 and 2 transport object 1 while
robot 3 transports object 2. Robot 3 will need to move over to let the other
robots pass.
"""
function pctapf_problem_11(;
        cost_function = SumOfMakeSpans(),
        verbose = false,
        Δt_op=0,
        Δt_collect=[0,0,0],
        Δt_deposit=[0,0,0],
    )
    N = 3                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4, 4)
    # 1   2   3   4
    # 5   6   7   8
    # 9  10  11  12
    # 13  14  15  16
    r0 = [1, 4, 15]
    s0 = [2, 11, 13]
    sF = [14, 1, 9]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )
    shapes = [(1, 2), (1, 1), (1, 1)]

    project_spec, robot_ICs = empty_pctapf_problem(r0, s0, sF)
    add_operation!(
        project_spec,
        construct_operation(project_spec, -1, [1, 2], [3], Δt_op),
    )
    add_operation!(
        project_spec,
        construct_operation(project_spec, -1, [3], [], Δt_op),
    )
    assignment_dict = Dict(1 => [1,3], 2 => [1], 3 => [2])

    def = SimpleProblemDef(project_spec,r0,s0,sF,shapes)
    sched, problem_spec = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function,Δt_collect=Δt_collect,Δt_deposit=Δt_deposit)

    return sched, problem_spec, env_graph, assignment_dict
end

export pctapf_problem_12

"""
    pctapf_problem_12(;

Robot 1 will plan a path first, but then that path will need to be extended by
one time step because robot 2 will get delayed by robot 3, which is on the
critical path.
"""
function pctapf_problem_12(;
        cost_function = MakeSpan(),
        verbose = false,
    )
    vtx_grid = initialize_dense_vtx_grid(4, 8)
    #  1   2   3   4   5   6   7   8
    #  9  10  11  12  13  14  15  16
    # 17  18  19  20  21  22  23  24
    # 25  26  27  28  29  30  31  32
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    r0 = [1, 4, 11]

    tasks = [17=>25,4=>20,15=>16]
    shapes = map(o->(1,1),tasks)
    ops = [
        (inputs=[2],outputs=[1]),
        (inputs=[1],outputs=[3]),
        (inputs=[3],outputs=[]),
    ]
    config = (tasks=tasks,shapes=shapes,ops=ops)
    assignment_dict = Dict(1=>[1],2=>[2],3=>[3])
    def = pctapf_problem(r0,config)
    sched, problem_spec = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function)

    return sched, problem_spec, env_graph, assignment_dict
end

export pctapf_problem_13

"""
    pctapf_problem_13

Same as `pctapf_problem_12`, except that there is a 4th robot who must collect 
object 1 with robot 1.
"""
function pctapf_problem_13(;
        cost_function = MakeSpan(),
    )
    vtx_grid = initialize_dense_vtx_grid(4, 8)
    #  1   2   3   4   5   6   7   8
    #  9  10  11  12  13  14  15  16
    # 17  18  19  20  21  22  23  24
    # 25  26  27  28  29  30  31  32
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    r0 = [1, 4, 11, 2]
    tasks = [17=>25,4=>20,15=>16]
    shapes = [(1, 2), (1, 1), (1, 1)]
    ops = [
        (inputs=[2],outputs=[1]),
        (inputs=[1],outputs=[3]),
        (inputs=[3],outputs=[]),
    ]
    config = (tasks=tasks,shapes=shapes,ops=ops)
    assignment_dict = Dict(1=>[1],2=>[2],3=>[3],4=>[1])
    def = pctapf_problem(r0,config)
    sched, problem_spec = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function)

    return sched, problem_spec, env_graph, assignment_dict
end

function pctapf_problem_14(;
        cost_function = MakeSpan(),
    )
    vtx_grid = initialize_dense_vtx_grid(1,4) 
    # 1  2  3  4
    r0 = [1,4]
    s0 = [1,4]
    sF = [3,2]
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    project_spec, robot_ICs = TaskGraphs.empty_pctapf_problem(r0,s0,sF)
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[],0))
    assignment_dict = Dict(1=>[1],2=>[2])
    def = SimpleProblemDef(project_spec,r0,s0,sF)
    sched, problem_spec = construct_task_graphs_problem(
        def,env_graph;cost_function=MakeSpan())

    return sched, problem_spec, env_graph, assignment_dict
end

"""
    pctapf_problem_15(;cost_function=SumOfMakeSpans(),verbose=false)

Trying to stress the multi goal algorithm
"""
function pctapf_problem_15(;cost_function=SumOfMakeSpans(),verbose=false)
    N = 2                  # num robots
    M = 4                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,8)
    #  1   2   3   4   5   6   7   8
    #  9  10  11  12  13  14  15  16
    # 17  18  19  20  21  22  23  24
    # 25  26  27  28  29  30  31  32
    r0 = [1,25]
    s0 = [1,25,5,29]
    sF = [3,28,7,31]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )

    project_spec, robot_ICs = empty_pctapf_problem(r0,s0,sF)
    add_operation!(project_spec,construct_operation(project_spec,-1,[1],[3],0))
    add_operation!(project_spec,construct_operation(project_spec,-1,[2],[4],0))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3,4],[],0))
    assignment_dict = Dict(1=>[1,3],2=>[2,4])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    sched, problem_spec = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function)

    return sched, problem_spec, env_graph, assignment_dict
end

export
    pctapf_test_problems,
    collaborative_pctapf_test_problems

pctapf_test_problems() = [
    pctapf_problem_1,
    pctapf_problem_2,
    pctapf_problem_3,
    pctapf_problem_4,
    pctapf_problem_5,
    pctapf_problem_6,
    pctapf_problem_7,
    pctapf_problem_8,
    pctapf_problem_9,
    pctapf_problem_10,
]
collaborative_pctapf_test_problems() = [
    pctapf_problem_11,
]

function generate_prob(recipe,solver,args...;kwargs...)
    pctapf_problem(solver,recipe(args...;kwargs...)...)
end
function generate_prob(::Type{PC_MAPF},recipe,solver,args...;kwargs...)
    prob = pctapf_problem(solver,recipe(args...;kwargs...)...)
    _,_,_, assignment_dict = recipe()
    apply_assignment_dict!(get_schedule(get_env(prob)),assignment_dict,get_problem_spec(get_env(prob)))
    PC_MAPF(get_env(prob))
end

for op in [
        :pctapf_problem_1,
        :pctapf_problem_2,
        :pctapf_problem_3,
        :pctapf_problem_4,
        :pctapf_problem_5,
        :pctapf_problem_6,
        :pctapf_problem_7,
        :pctapf_problem_8,
        :pctapf_problem_9,
        :pctapf_problem_10,
        :pctapf_problem_multi_backtrack,
        :pctapf_problem_11,
        :pctapf_problem_12,
        :pctapf_problem_13,
        :pctapf_problem_14,
        :pctapf_problem_15,
    ]
    @eval begin
        $op(solver,args...;kwargs...) = pctapf_problem(solver,$op(args...;kwargs...)...)
        function $op(::Type{PC_MAPF},solver,args...;kwargs...) 
            prob = pctapf_problem(solver,$op(args...;kwargs...)...)
            _,_,_, assignment_dict = $op()
            apply_assignment_dict!(get_schedule(get_env(prob)),assignment_dict,get_problem_spec(get_env(prob)))
            PC_MAPF(get_env(prob))
        end
    end
end

################################################################################
############################# Replanning Problems ##############################
################################################################################

export
    replanning_problem,
    replanning_problem_1,
    replanning_problem_2,
    replanning_problem_3,
    replanning_problem_4,
    replanning_test_problems

"""
    replanning_problem

Constructs a replanning problem, consisting of robot initial conditions, an
environment, and a sequence of project requests scheduled to arrive in the
factory at regular intervals.
Args:
- r0: list of integers specifying the start locations of the robots
- defs: a list of tuples, where each tuple is of the form

    `([start_1=>goal_1, ...], [([inputs],[outputs]),...])`

    where the `start=>goal` pairs define the start and end points for each
    object to be delivered, and the `([inputs],[outputs])` pairs define the
    objects that go in and out of each operation.
- env_graph: the environment (presumably a GridFactoryEnvironment)
Outputs:
- requests: a sequence of `ProjectRequest`s
- problem_spec: a `ProblemSpec`
- robot_ICs: Robot initial conditions `ROBOT_AT`
- env_graph: the environment
"""
function replanning_problem(solver,r0,defs,env_graph;
        cost_function=SumOfMakeSpans(),
        spacing=8,
        t0=0,
        Δt_op=0,
        )
    requests = Vector{ProjectRequest}()
    problem_spec = nothing
    robot_ICs = map(i->ROBOT_AT(i,r0[i]),1:length(r0))
    for (i,def) in enumerate(defs)
        s0 = map(i->i.first,def.tasks)
        sF = map(i->i.second,def.tasks)
        spec, _ = empty_pctapf_problem(r0,s0,sF)
        for op in def.ops
            add_operation!(spec,construct_operation(spec,-1,op.inputs,op.outputs,Δt_op))
        end
        if i == 1
            problem_def = SimpleProblemDef(spec,r0,s0,sF)
            _, problem_spec = construct_task_graphs_problem(
                problem_def,env_graph;
                cost_function=cost_function)
        end
        sched = construct_partial_project_schedule(
            spec,
            problem_spec,
            )
        t = t0+spacing*(i-1)
        push!(requests,ProjectRequest(sched,t,t))
    end

    base_schedule = construct_partial_project_schedule(
        robot_ICs,
        problem_spec,
        )
    base_env = construct_search_env(solver,base_schedule,problem_spec,env_graph)
    return RepeatedPC_TAPF(base_env,requests)
end

function replanning_problem_1(solver;kwargs...)
    vtx_grid = initialize_dense_vtx_grid(4,4)
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    r0 = [1,13]
    defs = [
        ( tasks=[1=>4,8=>6],            ops=[ (inputs=[1],outputs=[]), (inputs=[2],outputs=[]) ] ),
        ( tasks=[13=>1,16=>15],         ops=[ (inputs=[1],outputs=[2]), (inputs=[2],outputs=[]) ] ),
        ( tasks=[9=>11,8=>11,7=>16],    ops=[ (inputs=[1,2],outputs=[3]), (inputs=[3],outputs=[]) ] ),
        ( tasks=[2=>14,3=>15,11=>4],    ops=[ (inputs=[1,2],outputs=[3]), (inputs=[3],outputs=[]) ] ),
        ]
    return replanning_problem(solver,r0,defs,env_graph;kwargs...)
end

function replanning_problem_2(solver;kwargs...)
    vtx_grid = initialize_dense_vtx_grid(4,4)
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    r0 = [1,13]
    defs = [
        ( tasks=[13=>1,16=>15],         ops=[ (inputs=[1],outputs=[2]), (inputs=[2],outputs=[]) ] ),
        ( tasks=[1=>4,8=>6],            ops=[ (inputs=[1],outputs=[]), (inputs=[2],outputs=[]) ] ),
        ( tasks=[2=>14,3=>15,11=>4],    ops=[ (inputs=[1,2],outputs=[3]), (inputs=[3],outputs=[]) ] ),
        ( tasks=[9=>11,8=>11,7=>16],    ops=[ (inputs=[1,2],outputs=[3]), (inputs=[3],outputs=[]) ] ),
        ]
    return replanning_problem(solver,r0,defs,env_graph;kwargs...)
end

"""
    The robot should do better if it handles the single task in the second
    project prior to working on the third task of the first project.
"""
function replanning_problem_3(solver;kwargs...)
    vtx_grid = initialize_dense_vtx_grid(4,4)
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    r0 = [1]
    defs = [
        ( tasks=[2=>4,8=>5,12=>9],
            ops=[
                (inputs=[1],outputs=[2]),
                (inputs=[2],outputs=[3]),
                (inputs=[3],outputs=[])
                ] ),
        ( tasks=[13=>16],
            ops=[
                (inputs=[1],outputs=[]),
                ] ),
        ( tasks=[15=>1],
            ops=[
                (inputs=[1],outputs=[]),
                ] ),
        ]
    return replanning_problem(solver,r0,defs,env_graph;kwargs...)
end

"""
    Just intended to take longer so that the tests pass even if Julia hasn't
    finished warming up yet.
"""
function replanning_problem_4(solver;kwargs...)
    vtx_grid = initialize_dense_vtx_grid(4,16)
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    #  1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16
    # 17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
    # 33  34  35  36  37  38  39  40  41  42  43  44  45  46  47  48
    # 49  50  51  52  53  54  55  56  57  58  59  60  61  62  63  64
    r0 = [1]
    defs = [
        ( tasks=[1=>16,32=>17,33=>48],
            ops=[
                (inputs=[1],outputs=[2]),
                (inputs=[2],outputs=[3]),
                (inputs=[3],outputs=[])
                ] ),
        ( tasks=[64=>49],
            ops=[
                (inputs=[1],outputs=[]),
                ] ),
        ]
    return replanning_problem(solver,r0,defs,env_graph;kwargs...)
end
replanning_test_problems() = [
    replanning_problem_1,
    replanning_problem_2,
    replanning_problem_3,
    replanning_problem_4,
]

################################################################################
##################### Random RepeatedPC_TAPF instantiation #####################
################################################################################

export
    random_pctapf_def,
    random_pctapf_def,
    random_repeated_pctapf_def

"""
    instantiate_random_pctapf_def(env,config)

Instantiate a random `PC_TAPF` problem based on the parameters of config.
"""
function random_pctapf_def(env::GridFactoryEnvironment,
        config;
        N                   = get(config,:N,30),
        M                   = get(config,:M,10),
        num_unique_projects = get(config,:num_unique_projects,1),
        max_parents         = get(config,:max_parents,3),
        depth_bias          = get(config,:depth_bias,0.4),
        dt_min              = get(config,:dt_min,0),
        dt_max              = get(config,:dt_max,0),
        dt_collect          = get(config,:dt_collect,0),
        dt_deliver          = get(config,:dt_deliver,0),
        task_sizes          = get(config,:task_sizes,(1=>1.0,2=>0.0,4=>0.0)),
    )

    r0,s0,sF = get_random_problem_instantiation(N,M,get_pickup_zones(env),
        get_dropoff_zones(env),get_free_zones(env))
    shapes = choose_random_object_sizes(M,Dict(task_sizes...))
    project_spec = construct_random_project_spec(M,s0,sF;
        max_parents=max_parents,
        depth_bias=depth_bias,
        Δt_min=dt_min,
        Δt_max=dt_max)
    problem_def = SimpleProblemDef(project_spec,r0,s0,sF,shapes)
end
function random_multihead_pctapf_def(env::GridFactoryEnvironment,
        config;
        N                   = get(config,:N,30),
        M                   = get(config,:M,10),
        num_unique_projects = get(config,:num_unique_projects,1),
        m                   = get(config,:m,Int(M/num_unique_projects)),
        max_parents         = get(config,:max_parents,3),
        depth_bias          = get(config,:depth_bias,0.4),
        dt_min              = get(config,:dt_min,0),
        dt_max              = get(config,:dt_max,0),
        dt_collect          = get(config,:dt_collect,0),
        dt_deliver          = get(config,:dt_deliver,0),
        task_sizes          = get(config,:task_sizes,(1=>1.0,2=>0.0,4=>0.0)),
    )

    if !(m == M/num_unique_projects)
        @warn "!(m == M/num_unique_projects)"
        M = m*num_unique_projects
    end
    r0, s0, sF = get_random_problem_instantiation(N,M,
        get_pickup_zones(env),get_dropoff_zones(env),get_free_zones(env))
    project_spec = ProjectSpec()
    # iterate over s0 and sF in chunks of m at a time
    for (s0_,sF_) in zip(
        Base.Iterators.partition(s0, m),
        Base.Iterators.partition(sF, m),
    )
        new_project_spec = construct_random_project_spec(m,collect(s0_),collect(sF_);
            max_parents=max_parents,
            depth_bias=depth_bias,
            Δt_min=dt_min,
            Δt_max=dt_max)
        merge!(project_spec,new_project_spec)
    end
    shapes = choose_random_object_sizes(M,Dict(task_sizes...))
    problem_def = SimpleProblemDef(project_spec,r0,s0,sF,shapes)
end

"""
    random_pcmapf_def(env,config;objective=MakeSpan(),solver,kwargs...)

Return a random `SimplePCMAPFDef`. Same arguments for `config` as in 
`random_multihead_pctapf_def`.
"""
function random_pcmapf_def(env,config;
        objective=MakeSpan(),
        solver=TaskGraphsMILPSolver(GreedyAssignment(greedy_cost=GreedyFinalTimeCost())),
        kwargs...
        )
    pctapf_def = random_multihead_pctapf_def(env,config;kwargs...)
    pcta = PC_TA(pctapf_def,env,objective)
    sched, cost = solve!(solver,pcta)
    @assert validate(sched)
    return SimplePCMAPFDef(pctapf_def,get_assignment_dict(sched))
end

## Stochastic PCTAPF problems
# """
#     pctapf_problem_1
#
# Optimal MakeSpan = 5
# Optimal SumOfMakeSpans = 5
# """
# function spctapf_problem_1(;cost_function=SumOfMakeSpans(),verbose=false)
#     vtx_grid = initialize_dense_vtx_grid(4,4)
#     env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
#     #  1   2   3   4
#     #  5   6   7   8
#     #  9  10  11  12
#     # 13  14  15  16
#     config = (
#         delivery_bots = [1],
#         cleanup_bots = [13],
#         tasks=[1=>4,8=>16],
#         ops=[ (inputs=[1],outputs=[]), (inputs=[2],outputs=[]) ] )
#     )
#     project_spec, robot_ICs = empty_pctapf_problem(r0,s0,sF)
#
#     add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],0))
#     add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], 0))
#     assignment_dict = Dict(1=>[1,3],2=>[2])
#
#     def = SimpleProblemDef(project_spec,r0,s0,sF)
#     sched, problem_spec = construct_task_graphs_problem(
#         def,env_graph;cost_function=cost_function)
#     return sched, problem_spec, env_graph, assignment_dict
# end
