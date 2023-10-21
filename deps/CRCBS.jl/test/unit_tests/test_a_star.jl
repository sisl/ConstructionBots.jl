let
    solver = VanillaAStar()
    set_iteration_limit!(solver,100)
    set_verbosity!(solver,0)
    for f in mapf_test_problems()
        mapf = f()
        s0 = get_start(mapf,1)
        sF = get_goal(mapf,1)
        env = build_env(mapf,initialize_root_node(mapf),1)
        path, cost = CRCBS.a_star!(solver,env,s0)
        @test get_distance(env,s0,sF) == cost
        reset_solver!(solver)
    end
end
