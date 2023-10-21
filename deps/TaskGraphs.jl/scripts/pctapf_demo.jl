using TaskGraphs

## set up the environment
vtx_grid = initialize_dense_vtx_grid(4,4) # 4 x 4 grid world
#  1   2   3   4
#  5   6   7   8
#  9  10  11  12
# 13  14  15  16
env = construct_factory_env_from_vtx_grid(vtx_grid)

## Define the initial conditions of the robots
robot_ics = [
    ROBOT_AT(1,2), # robot 1 starts at vertex 2
    ROBOT_AT(2,9), # robot 2 starts at vertex 9
]

## Define the manufacturing project
spec = ProjectSpec()
# set initial conditions of "raw materials"
set_initial_condition!(spec,OBJECT_AT(1,4)) # object 1 starts at vertex 4
set_initial_condition!(spec,OBJECT_AT(2,16))  # object 2 starts at vertex 16
# define the operations that need to take place
op1 = Operation(Δt=2) # operation 1 has a duration of 2 time steps 
# inputs
set_precondition!(op1,OBJECT_AT(1,8)) # object 1 must be at vertex 8 before op1 can begin
set_precondition!(op1,OBJECT_AT(2,12)) # object 2 must be at vertex 12 before op1 can begin
# outputs
set_postcondition!(op1,OBJECT_AT(3,7)) # object 3 appears at vertex 7 when op1 is completed 
add_operation!(spec,op1)
# add a terminal operation with no outputs
op2 = Operation(Δt=0)
set_precondition!(op2,OBJECT_AT(3,13))
add_operation!(spec,op2)

## define solver
solver = NBSSolver()
# finalize problem construction (the solver is passed as an argument here 
# because it determines what cost model is used for the problem)
prob = pctapf_problem(solver,spec,env,robot_ics)
# solve the problem
solution, cost = solve!(solver,prob)
# check if the problem was solved to optimality
@show feasible_status(solver)
@show optimal_status(solver)