export
    solve!,
    CBSSolver,
    MetaAgentCBS_Solver

function solve! end
function low_level_search! end

"""
    solve!(solver, args ...)

Run the algorithm represented by `solver` on an instance of a Multi-Agent
Path-Finding problem.
"""
function solve!(solver, args...)
    throw(ArgumentError(string(
        "function CRCBS.solve!(solver::", typeof(solver),",...) not defined.",
        " You must explicitly override CRCBS.solve!() for solvers of type",
        typeof(solver)
        )))
end

export
    BiLevelPlanner,
    AbstractCBSSolver,
    low_level

abstract type BiLevelPlanner end
low_level(solver::BiLevelPlanner) = solver.low_level_planner
abstract type AbstractCBSSolver <: BiLevelPlanner end
function set_best_cost!(solver::AbstractCBSSolver,cost)
    set_best_cost!(get_logger(solver),cost)
    set_best_cost!(low_level(solver),cost)
end
function hard_reset_solver!(solver::AbstractCBSSolver)
    hard_reset_solver!(get_logger(solver))
    hard_reset_solver!(low_level(solver))
end
