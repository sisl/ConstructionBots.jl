export
	SolverStatus,
	time_out_status,
	iteration_max_out_status,
	failed_status,
	set_time_out_status!,
	set_iteration_max_out_status!

@with_kw mutable struct SolverStatus
	timed_out::Bool = false
	iterations_maxed_out::Bool = false
end
solver_status(status::SolverStatus) = status
time_out_status(status) 			= solver_status(status).timed_out
iteration_max_out_status(status) 	= solver_status(status).iterations_maxed_out
failed_status(status) = time_out_status(status) || iteration_max_out_status(status)
function set_time_out_status!(status,val=true)
	solver_status(status).timed_out = val
end
function set_iteration_max_out_status!(status,val=true)
	solver_status(status).iterations_maxed_out = val
end

export SolverLogger

"""
    SolverLogger

A logger type for keeping track of thing like runtime, iterations, optimality
gap (including upper and lower bound), etc. The following methods allow for 
accessing and modifying the `SolverLogger`'s fields, and are extended to solver
types that wrap a `SolverLogger`:

iterations(logger)      
iteration_limit(logger) 
max_iterations(logger)  
start_time(logger)      
runtime_limit(logger)   
deadline(logger)        
JuMP.lower_bound(logger)
best_cost(logger)       
verbosity(logger)       
verbosity(val::Int)		
debug(logger)           
solver_status(logger)

set_iterations!(solver,val)
increment_iteration_count!(logger::SolverLogger)
set_iteration_limit!(solver,val)
set_max_iterations!(solver,val)
set_start_time!(solver,val)
set_runtime_limit!(solver,val)
set_deadline!(solver,val)
set_lower_bound!(solver,val)
set_lower_bound!(logger::SolverLogger{C},val::C) where {C}
set_lower_bound!(logger::SolverLogger{NTuple{N,T}},val::R) where {N,T<:Real,R<:Real}
set_lower_bound!(logger::SolverLogger{T},val::R) where {T<:Tuple,R<:Real}
set_best_cost!(solver,val)
set_best_cost!(logger::SolverLogger,val)
set_best_cost!(logger::SolverLogger{NTuple{N,T}},val::R) where {N,T<:Real,R<:Real}
set_best_cost!(logger::SolverLogger{T},val::R) where {T<:Tuple,R<:Real}
set_verbosity!(solver,val)
set_debug!(solver,val)

The following methods facilitate control flow based on solver status.
check_time(logger)
enforce_time_limit!(logger)
check_iterations(logger)
enforce_iteration_limit!(logger)
"""
@with_kw mutable struct SolverLogger{C}
    iterations      ::Int           = 0
    iteration_limit ::Int           = 100
    max_iterations  ::Int           = 0
    start_time      ::Float64       = time()
    runtime_limit   ::Float64       = 100.0
    deadline        ::Float64       = Inf
    lower_bound     ::C             = typemin(C)
    best_cost       ::C             = typemax(C)
    verbosity       ::Int           = 0
    DEBUG           ::Bool          = false
	status			::SolverStatus	= SolverStatus()
end
cost_type(logger::SolverLogger{C}) where {C} = C
cost_type(solver) = cost_type(get_logger(solver))

export get_logger

get_logger(solver) = solver.logger
get_logger(logger::SolverLogger) = logger

export SolverWrapper

"""
	SolverWrapper

An abstract type whose concrete instances must have a `solver` field.
"""
abstract type SolverWrapper end
get_logger(solver::SolverWrapper) = get_logger(solver.solver)
low_level(solver::SolverWrapper) = low_level(solver.solver)

export
    iterations,
    max_iterations,
    iteration_limit,
    start_time,
    runtime_limit,
    deadline,
    best_cost,
    verbosity,
    debug,
    time_to_deadline

iterations(logger)      = get_logger(logger).iterations
iteration_limit(logger) = get_logger(logger).iteration_limit
max_iterations(logger)  = get_logger(logger).max_iterations
start_time(logger)      = get_logger(logger).start_time
runtime_limit(logger)   = get_logger(logger).runtime_limit
deadline(logger)        = get_logger(logger).deadline
JuMP.lower_bound(logger)= get_logger(logger).lower_bound
best_cost(logger)       = get_logger(logger).best_cost
verbosity(logger)       = get_logger(logger).verbosity
verbosity(val::Int)		= val
verbosity(::Nothing)    = global_verbosity()
debug(logger)           = get_logger(logger).DEBUG
solver_status(logger) 	= get_logger(logger).status

"""
    time_to_deadline(solver)

time to `deadline(solver)` or `runtime_limit(solver)`--whichever is shorter.
"""
time_to_deadline(logger) = min(runtime_limit(logger),deadline(logger)-time())


export
	set_iterations!,
	set_iteration_limit!,
	set_max_iterations!,
	set_start_time!,
	set_runtime_limit!,
	set_deadline!,
	set_lower_bound!,
	set_best_cost!,
	set_verbosity!,
	set_debug!

function set_iterations!(solver,val)
	get_logger(solver).iterations = val
end
function set_iteration_limit!(solver,val)
	get_logger(solver).iteration_limit = val
end
function set_max_iterations!(solver,val)
	get_logger(solver).max_iterations = val
end
function set_start_time!(solver,val)
	get_logger(solver).start_time = val
end
function set_runtime_limit!(solver,val)
	get_logger(solver).runtime_limit = val
end
function set_deadline!(solver,val)
	get_logger(solver).deadline = val
end
function set_lower_bound!(solver,val)
	get_logger(solver).lower_bound = val
end
function set_lower_bound!(logger::SolverLogger{C},val::C) where {C}
    logger.lower_bound = val
end
function set_lower_bound!(logger::SolverLogger{NTuple{N,T}},val::R) where {N,T<:Real,R<:Real}
    logger.lower_bound = NTuple{N,T}((T(val),zeros(T,N-1)...))
end
function set_lower_bound!(logger::SolverLogger{T},val::R) where {T<:Tuple,R<:Real}
    v = cost_type(logger)([val,zeros(length(lower_bound(logger))-1)...])
    set_lower_bound!(logger,v)
end
function set_best_cost!(solver,val)
	set_best_cost!(get_logger(solver), val)
end
function set_best_cost!(logger::SolverLogger,val)
    logger.best_cost = val
end
function set_best_cost!(logger::SolverLogger{NTuple{N,T}},val::R) where {N,T<:Real,R<:Real}
    logger.best_cost = NTuple{N,T}((T(val),zeros(T,N-1)...))
end
function set_best_cost!(logger::SolverLogger{T},val::R) where {T<:Tuple,R<:Real}
    v = cost_type(logger)([val,zeros(length(lower_bound(logger))-1)...])
    set_best_cost!(logger,v)
end
function set_verbosity!(solver,val)
	get_logger(solver).verbosity = val
end
function set_debug!(solver,val)
	get_logger(solver).DEBUG = val
end
for op in [:set_time_out_status!,:set_iteration_max_out_status!,
		:time_out_status,:iteration_max_out_status]
	@eval $op(logger,args...) = $op(get_logger(logger).status,args...)
end

export SolverException

"""
    SolverException

Custom exception type for tracking solver timeouts, etc.
"""
struct SolverException <: Exception
    msg::String
end
SolverException(msg::String,msg2,args...) = SolverException(string(msg,msg2,args...))

export handle_solver_exception

"""
	handle_solver_exception

Takes care of printing `SolverException`s
"""
function handle_solver_exception(solver,e)
    if debug(solver)
        showerror(stdout, e, catch_backtrace())
    else
        printstyled(e,"\n";color=:red)
    end
end

export
	solver_type

solver_type(::SolverLogger) = "Solver"

export
    optimality_gap,
	feasible_status,
	optimal_status

optimality_gap(logger) = best_cost(logger) .- lower_bound(logger)
feasible_status(solver) = best_cost(solver) < typemax(cost_type(solver))
optimal_status(solver) 	= optimality_gap(solver) == 0

export
	check_time,
	check_iterations,
	enforce_time_limit!,
	enforce_iteration_limit!,
    increment_iteration_count!

function check_time(logger)
    t = time()
    if t >= deadline(logger) || t - start_time(logger) >= runtime_limit(logger)
        return true
    end
    return false
end
function enforce_time_limit!(logger)
    if check_time(logger)
		set_time_out_status!(logger,true)
        throw(SolverException("$(solver_type(logger)) time limit exceeded!",
		" deadline was $(deadline(logger)),",
		" runtime_limit was $(runtime_limit(logger))"))
    end
end
function check_iterations(logger)
    if iterations(logger) >= iteration_limit(logger)
		return true
	end
	return false
end
function enforce_iteration_limit!(logger)
    if check_iterations(logger)
		set_iteration_max_out_status!(logger,true)
        throw(SolverException("Solver iterations exceeded! Limit was $(iteration_limit(logger))."))
    end
end

function increment_iteration_count!(logger::SolverLogger)
    logger.iterations += 1
    set_max_iterations!(logger,max(iterations(logger),max_iterations(logger)))
    logger.iterations
end
increment_iteration_count!(solver)  = increment_iteration_count!(get_logger(solver))

export
    reset_solver!,
    soft_reset_solver!,
    hard_reset_solver!

"""
    reset_solver!(solver)

Resets iteration counts and start times, in addition to best cost and lower bound.
"""
function reset_solver!(logger::SolverLogger)
    set_iterations!(logger, 0)
    set_best_cost!(logger,typemax(cost_type(logger)))
    set_lower_bound!(logger,typemin(cost_type(logger)))
    set_start_time!(logger,time())
	set_time_out_status!(logger,false)
	set_iteration_max_out_status!(logger,false)
    logger
end
reset_solver!(solver)               = reset_solver!(get_logger(solver))

"""
    soft_reset_solver!(solver)

Resets iteration counts and start times.
"""
function soft_reset_solver!(logger::SolverLogger)
    set_iterations!(logger, 0)
    set_start_time!(logger,time())
	set_time_out_status!(logger,false)
	set_iteration_max_out_status!(logger,false)
    logger
end
soft_reset_solver!(solver)               = soft_reset_solver!(get_logger(solver))

"""
    hard_reset_solver!(solver)

To be called when no information (other than iteration and time limits) needs to
be stored.
"""
function hard_reset_solver!(logger::SolverLogger)
    reset_solver!(logger)
    set_max_iterations!(logger,0)
end
hard_reset_solver!(solver)          = hard_reset_solver!(get_logger(solver))

# get_infeasible_cost(logger::SolverLogger{C}) where {C} = typemax(C)
# get_infeasible_cost(solver) = get_infeasible_cost(get_logger(solver))


# export
#     global_verbosity,
#     set_global_verbosity!
#
# global VERBOSITY = 0
#
# """
# 	global_verbosity()
#
# Query the global verbosity setting (VERBOSITY ∈ [0,])
# """
# global_verbosity() = copy(VERBOSITY)
#
# """
# 	global_verbosity()
#
# Set the global verbosity to val
# """
# set_global_verbosity!(val::Int) = begin global VERBOSITY = val end
#
# function print_styled_header(header,msg...;bold=true,color=136)
# 	printstyled(header;bold=bold,color=color)
# 	println(msg...)
# end
#
# export @log_info
#
# """
#     @log_info
#
# A helper macro for printing at various verbosity levels.
# Usage:
# 	`@log_info(limit::Int,level::Int,msg...)`
# 	`@log_info(limit::Int,solver,msg...)`
# Args:
# * limit::Int - logging threshold
# * level::Int - the verbosity level
# * msg... - message to print if level > limit
# """
# macro log_info(limit,level,msg...)
# 	filename = split(string(__source__.file),"/")[end]
# 	ex = :(print_styled_header(log_preamble))
#     for x in esc(msg).args[1]
# 		push!(ex.args,esc(x))
#     end
# 	return quote
# 		local log_preamble = string("[ logger ](",$filename,"-",$(__source__.line),"): ")
# 		verbosity($(esc(level))) > $(esc(limit)) || global_verbosity() > $(esc(limit)) ?
# 			$ex : nothing
# 	end
# end
