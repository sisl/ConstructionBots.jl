let 
    env = init_env_2()
    configs = [Dict(:env_id=>"env_2",:M=>2,:N=>2)]
    solver = NBSSolver(
        assignment_model=TaskGraphsMILPSolver(GreedyAssignment(greedy_cost=GreedyFinalTimeCost())),
    )
    base_path = tempname()
    base_prob_path = joinpath(base_path,"problems")
    results_path = joinpath(base_path,"results")
    feats = [
        RunTime(),
        FeasibleFlag(),
        OptimalFlag(),
        OptimalityGap(),
        SolutionCost(),
        PrimaryCost(),
        TaskAssignmentDict(),
    ]
    let 
        loader = PCTA_Loader()
        add_env!(loader,"env_2",env)
        write_problems!(loader,configs,base_prob_path)
        solver_config = (
            solver = assignment_solver(solver),
            results_path = joinpath(results_path,string(problem_type(loader))),
            feats = feats,
            objective = MakeSpan(),
        )
        run_profiling(loader,solver_config,base_prob_path)
        # store assignments with problem definition
        TaskGraphs.write_pcmapf_from_pctapf!(loader,solver_config,base_prob_path)
        loader = PCMAPF_Loader()
        add_env!(loader,"env_2",env)
        solver_config = (
            solver = route_planner(solver),
            results_path = joinpath(results_path,"PC_TA",string(problem_type(loader))),
            feats = feats,
            objective = MakeSpan(),
        )
        run_profiling(loader,solver_config,base_prob_path)
    end
    let 
        loader = PCTAPF_Loader()
        add_env!(loader,"env_2",env)
        write_problems!(loader,configs,base_prob_path)
        solver_config = (
            solver = solver,
            results_path = joinpath(results_path,string(problem_type(loader))),
            feats = feats,
            objective = MakeSpan(),
        )
        run_profiling(loader,solver_config,base_prob_path)
    end
    let 
        loader = PCMAPF_Loader()
        add_env!(loader,"env_2",env)
        write_problems!(loader,configs,base_prob_path)
        solver_config = (
            solver = route_planner(solver),
            results_path = joinpath(results_path,string(problem_type(loader))),
            feats = feats,
            objective = MakeSpan(),
        )
        run_profiling(loader,solver_config,base_prob_path)
    end

end