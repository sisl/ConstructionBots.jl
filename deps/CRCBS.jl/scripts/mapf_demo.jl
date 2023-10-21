using CRCBS

## set up the environment
vtx_grid = initialize_dense_vtx_grid(4,4) # 4 x 4 grid world
#  1   2   3   4
#  5   6   7   8
#  9  10  11  12
# 13  14  15  16
env = construct_factory_env_from_vtx_grid(vtx_grid)

## Define the initial conditions of the robots
starts = [1,4]
goals = [13,16]

cost_model = FullCostModel(maximum,TravelTime())

prob = init_mapf_problem(env,starts,goals,cost_model)

## define solver
solver = CBSSolver() # PIBTPlanner()

# solve the problem
solution, cost = solve!(solver,prob)
# check if the problem was solved to optimality
@show feasible_status(solver)
@show optimal_status(solver)