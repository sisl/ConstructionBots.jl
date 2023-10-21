let
    mapf = MAPF(CBSEnv.LowLevelEnv(), [1,2,3,4], [5,6,7,8])
    @test num_agents(mapf) == 4
    @test num_goals(mapf) == 4

    action_type(mapf)
    state_type(mapf)
    cost_type(mapf)
    get_starts(mapf)
    get_goals(mapf)
    i = 1
    env = mapf.env
    get_start(mapf, i)
    get_goal(mapf, i)
    get_start(mapf, env, i)
end
