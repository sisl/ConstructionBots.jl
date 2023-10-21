# Verify that all task assignment models succeed in solving all examples, and
# that all optimal solvers obtain the same lower bound cost
let

    for (i, f) in enumerate([
        pctapf_problem_1,
        pctapf_problem_2,
        pctapf_problem_3,
        pctapf_problem_4,
        pctapf_problem_5,
        pctapf_problem_6,
        pctapf_problem_7,
        pctapf_problem_8,
    ])
        for cost_model in [MakeSpan(), SumOfMakeSpans()]
            costs = Float64[]
            for solver in [
                TaskGraphsMILPSolver(AssignmentMILP()),
                TaskGraphsMILPSolver(SparseAdjacencyMILP()),
                TaskGraphsMILPSolver(GreedyAssignment()),
            ]
                # MILP formulations alone
                sched, problem_spec, _, _ = f(
                    ;
                    cost_function = cost_model,
                    verbose = false,
                )
                model = formulate_milp(
                    solver,
                    sched,
                    problem_spec;
                    cost_model = cost_model,
                )
                optimize!(model)
                @test termination_status(model) == MOI.OPTIMAL
                cost = Int(round(value(objective_function(model))))
                update_project_schedule!(
                    solver,
                    model,
                    sched,
                    problem_spec,
                    get_assignment_matrix(model),
                )
                if !isa(model, GreedyAssignment)
                    push!(costs, cost)
                end
                @test validate(sched)
                @test cost != Inf
            end
                # @show costs
            @test all(costs .== costs[1]) # error because AssignmentMILP is adding a job shop constraint that shouldn't be there?? On toy_problem_3
        end
    end
end
# Root nodes
let

    for (f, costs) in [
        (pctapf_problem_4, [2, 2]),
        (pctapf_problem_8, [8, 16]),
    ]
        for solver in [
            TaskGraphsMILPSolver(AssignmentMILP()),
            TaskGraphsMILPSolver(SparseAdjacencyMILP()),
        ]
            for (i, cost_model) in enumerate([MakeSpan(), SumOfMakeSpans()])
                let
                    sched, problem_spec, env_graph, _ = f(; cost_function = cost_model, verbose = false)
                    let
                        model = formulate_milp(
                            solver,
                            sched,
                            problem_spec;
                            cost_model = cost_model,
                        )
                        optimize!(model)
                        optimal = (termination_status(model) == MathOptInterface.OPTIMAL)
                        optimal_TA_cost = Int(round(value(objective_function(model))))
                        @test optimal == true
                        @test optimal_TA_cost == costs[i]
                    end
                end
            end
        end
    end
end
# Station Sharing
let
        for solver in [
            TaskGraphsMILPSolver(AssignmentMILP()),
            TaskGraphsMILPSolver(SparseAdjacencyMILP()),
        ]
        for (i, dt) in enumerate([0, 1, 2])
            for (cost_model, costs) in [
                (MakeSpan(), [4, 5, 6])
                (SumOfMakeSpans(), [7, 8, 9])
            ]
                let
                    sched, problem_spec, env_graph, _ = pctapf_problem_9(
                        ;
                        cost_function = cost_model,
                        verbose = false,
                        Δt_op = 0,
                        Δt_collect = [dt, 0],
                        Δt_deposit = [0, 0],
                    )
                    model = formulate_milp(
                        solver,
                        sched,
                        problem_spec;
                        cost_model = cost_model,
                    )
                    optimize!(model)
                    optimal = (termination_status(model) == MathOptInterface.OPTIMAL)
                    optimal_TA_cost = Int(round(value(objective_function(model))))
                    # @show i, cost_model
                    @test optimal == true
                    @test optimal_TA_cost == costs[i]
                end
            end
        end
    end
end
# Job shop constraints
let
    sched, problem_spec, env_graph, _ = pctapf_problem_9(;
        verbose=false,Δt_op=0,Δt_collect=[0,0],Δt_deposit=[0,0]
        );
    cost_model=SumOfMakeSpans()
    solver=TaskGraphsMILPSolver(AssignmentMILP())
    model = formulate_milp(solver,sched,problem_spec;cost_model=cost_model)
    optimize!(model)
    @test termination_status(model) == MathOptInterface.OPTIMAL
    assignment_matrix = get_assignment_matrix(model);
    update_project_schedule!(solver,model,sched,problem_spec,assignment_matrix)
    n_edges = ne(get_graph(sched))
    add_job_shop_constraints!(model,sched,problem_spec)
    @test ne(get_graph(sched)) > n_edges
end
