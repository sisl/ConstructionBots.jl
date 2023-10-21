module TaskGraphs

using Parameters
#using LightGraphs 
using Graphs
using Random
using LinearAlgebra
using DataStructures
using JuMP, MathOptInterface
using GLPK
using TOML
using CRCBS
using SparseArrays
using Reexport

export
    DEBUG_PATH,
    EXPERIMENT_DIR,
    ENVIRONMENT_DIR,
    PROBLEM_DIR,
    RESULTS_DIR,
    VIDEO_DIR

global DEBUG_PATH       = joinpath(dirname(pathof(TaskGraphs)),"..","debug")
global EXPERIMENT_DIR   = joinpath(dirname(pathof(TaskGraphs)),"..","experiments")
global ENVIRONMENT_DIR  = joinpath(EXPERIMENT_DIR,"environments")
global PROBLEM_DIR      = joinpath(EXPERIMENT_DIR,"problem_instances")
global RESULTS_DIR      = joinpath(EXPERIMENT_DIR,"results")
global VIDEO_DIR        = joinpath(EXPERIMENT_DIR,"videos")

include("JuMP_interface.jl")
include("planning_predicates.jl")
include("task_graphs_core.jl")
include("utils.jl")
include("task_assignment_solvers.jl")
include("path_planning.jl")
include("pc_tapf_solvers.jl")
include("edge_milp_formulation.jl")
include("disturbances.jl")
include("path_planners/pibt_planner.jl")
include("replanning.jl")
include("helpers/problem_instances.jl")
include("helpers/profiling.jl")
include("experiments/replanning_experiments.jl")

@reexport using CRCBS

# set GLPK as the default optimizer
set_default_milp_optimizer!(GLPK.Optimizer)
set_default_optimizer_attributes!(MOI.TimeLimitSec()=>100, MOI.Silent()=>true)

end
