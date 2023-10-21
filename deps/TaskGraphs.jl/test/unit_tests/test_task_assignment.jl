let
    solver = NBSSolver(
        assignment_model=TaskGraphsMILPSolver(GreedyAssignment())
    )
    prob = pctapf_problem_1(solver)
    env = get_env(prob)
    sched = get_schedule(env)

    # reserved = Set{Int}()
    # reserved = Set{Int}()
    # for (v,vp) in [(9,14)]
    #     add_edge!(get_graph(get_schedule(env)),v,vp)
    # end
    starts = map(id->get_vtx(sched,id), values(robot_tip_map(sched)))

    cache = compute_lower_bound(env,Set{Int}(starts))
    maximum(get_cache(env).tF)
    maximum(cache.tF)
end
