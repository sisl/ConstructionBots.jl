# TODO(#32): Consider replacing components with direct imports from TaskGraphs

# Used components from TaskGraphs
const path_spec_accessor_interface =
    [:get_t0, :get_tF, :get_slack, :get_local_slack, :get_min_duration, :get_duration]
const path_spec_mutator_interface =
    [:set_min_duration!, :set_t0!, :set_slack!, :set_local_slack!, :set_tF!]
const schedule_node_accessor_interface = [path_spec_accessor_interface..., :get_path_spec]
const schedule_node_mutator_interface = [path_spec_mutator_interface..., :set_path_spec!]

"""
    ProblemSpec{G}

Encodes the relaxed PC-TAPF problem that ignores collision-avoidance constraints.
    
Elements:
- D::T - a distance matrix (or environment) that implements get_distance(D,x1,x2,args...)
- cost_function::F - the optimization objective (default is SumOfMakeSpans)
- Δt_collect::Dict{ObjectID,Int} # duration of COLLECT operations
- Δt_deposit::Dict{ObjectID,Int} # duration of DEPOSIT operations
"""
@with_kw struct ProblemSpec{T,F}
    D::T = zeros(0, 0) # environment or distance matrix
    cost_function::F = SumOfMakeSpans()
    Δt_collect::Dict{ObjectID,Int} = Dict{ObjectID,Int}() # duration of COLLECT operations
    Δt_deposit::Dict{ObjectID,Int} = Dict{ObjectID,Int}() # duration of DEPOSIT operations
end

"""
    PathSpec

Encodes information about the path that must be planned for a particular schedule node.

Fields:
- node_type::Symbol = :EMPTY
- start_vtx::Int = -1
- final_vtx::Int = -1
- min_duration::Int = 0
- agent_id::Int = -1
- object_id::Int = -1
- plan_path::Bool = true - flag indicating whether a path must be planned. For example, 'Operation' nodes do not require any path planning.
- tight::Bool = false - if true, the path may not terminate prior to the beginning of successors. If 'tight == true', local slack == 0. For example, 'GO' must not end before 'COLLECT' can begin, because this would produce empty time between planning phases.
- static::Bool = false - if true, the robot must remain in place for this planning phase (e.g., COLLECT, DEPOSIT).
- free::Bool = false - if true, and if the node is a terminal node, the planning
must go on until all non-free nodes are completed.
- fixed::Bool = false - if true, do not plan path because it is already fixed. Instead, retrieve the portion of the path directly from the pre-existing solution.
"""
@with_kw mutable struct PathSpec
    t0::Float64 = 0
    min_duration::Float64 = 0
    tF::Float64 = t0 + min_duration
    slack::Vector{Float64} = Float64[]
    local_slack::Vector{Float64} = Float64[]
    plan_path::Bool = true
    tight::Bool = false
    static::Bool = false
    free::Bool = false
    fixed::Bool = false
end

"""
    ScheduleNode{I<:AbstractID,V<:AbstractPlanningPredicate}

The node type of the `OperatingSchedule` graph.
"""
mutable struct ScheduleNode{I<:AbstractID,V}
    id::I
    node::V
    spec::PathSpec
end

ScheduleNode(id, node) = ScheduleNode(id, node, PathSpec())
for op in [:(matches_template)]
    @eval $op(template::Type{T}, node::ScheduleNode) where {T} = $op(template, node.node)
end
Base.summary(n::ScheduleNode) = string(string(n.node), " [", summary(n.spec), "]")

# Accessor and Mutator Methods for PathSpec and ScheduleNode
get_t0(spec::PathSpec) = spec.t0
set_t0!(spec::PathSpec, val) = begin
    spec.t0 = val
end
get_tF(spec::PathSpec) = spec.tF
set_tF!(spec::PathSpec, val) = begin
    spec.tF = val
end
get_min_duration(spec::PathSpec) = spec.min_duration
set_min_duration!(spec::PathSpec, val) = begin
    spec.min_duration = val
end
get_duration(spec::PathSpec) = get_tF(spec) - get_t0(spec)
get_slack(spec::PathSpec) = spec.slack
set_slack!(spec::PathSpec, val) = begin
    spec.slack = val
end
get_local_slack(spec::PathSpec) = spec.local_slack
set_local_slack!(spec::PathSpec, val) = begin
    spec.local_slack = val
end

Base.summary(s::PathSpec) = string("t0=", s.t0, ", tF=", s.tF, ", fixed=", s.fixed)
get_path_spec(node::ScheduleNode) = node.spec
Base.copy(n::ScheduleNode) = ScheduleNode(n.id, copy(n.node), deepcopy(n.spec))

function set_path_spec!(node::ScheduleNode, spec)
    node.spec = spec
end

for op in path_spec_accessor_interface
    @eval $op(node::ScheduleNode) = $op(get_path_spec(node))
end
for op in path_spec_mutator_interface
    @eval $op(node::ScheduleNode, val) = $op(get_path_spec(node), val)
end

"""
    OperatingSchedule

Encodes discrete events/activities that need to take place, and the precedence
constraints between them. Each `ScheduleNode` has a corresponding vertex index
and an `AbstractID`. An edge from node1 to node2 indicates a precedence
constraint between them.
"""
@with_kw struct OperatingSchedule <: AbstractCustomNDiGraph{ScheduleNode,AbstractID}
    graph::DiGraph = DiGraph()
    nodes::Vector{ScheduleNode} = Vector{ScheduleNode}()
    vtx_map::Dict{AbstractID,Int} = Dict{AbstractID,Int}()
    vtx_ids::Vector{AbstractID} = Vector{AbstractID}() # maps vertex uid to actual graph node
    terminal_vtxs::Vector{Int} = Vector{Int}() # list of "project heads"
    weights::Dict{Int,Float64} = Dict{Int,Float64}() # weights corresponding to project heads
end

get_terminal_vtxs(sched::P) where {P<:OperatingSchedule} = sched.terminal_vtxs
get_root_node_weights(sched::P) where {P<:OperatingSchedule} = sched.weights

function Base.copy(sched::OperatingSchedule)
    OperatingSchedule(
        graph = deepcopy(get_graph(sched)),
        nodes = map(copy, get_nodes(sched)),
        vtx_map = deepcopy(sched.vtx_map),
        vtx_ids = deepcopy(sched.vtx_ids),
        terminal_vtxs = deepcopy(sched.terminal_vtxs),
        weights = deepcopy(sched.weights),
    )
end

get_vtx(sched::OperatingSchedule, node::ScheduleNode) = get_vtx(sched, node.id)
get_node_from_id(sched::OperatingSchedule, id) = get_node(sched, id).node
get_node_from_vtx(sched::OperatingSchedule, v) = get_node(sched, v).node

# Accessor and mutator interfaces for OperatingSchedule
for op in schedule_node_accessor_interface
    @eval $op(sched::OperatingSchedule, v) = $op(get_node(sched, v))
    @eval $op(sched::OperatingSchedule) =
        map(v -> $op(get_node(sched, v)), Graphs.vertices(sched))
end
for op in schedule_node_mutator_interface
    @eval $op(sched::OperatingSchedule, v, val) = $op(get_node(sched, v), val)
    @eval $op(sched::OperatingSchedule, val) = begin
        for v in Graphs.vertices(sched)
            $op(get_node(sched, v), val)
        end
    end
end

# Additional methods and criteria for ScheduleNode
is_tight(p) = false
is_free(p) = false
is_static(p) = false
needs_path(p) = false
duration_lower_bound(args...) = 0

"""
    replace_in_schedule!(schedule::OperatingSchedule, path_spec::T, pred, id::ID) where {T <: PathSpec, ID <: AbstractID}

Replace the `ScheduleNode` associated with `id` with the new node `pred`, and the accompanying `PathSpec` `path_spec`.
"""
replace_in_schedule!(
    sched::OperatingSchedule,
    node::ScheduleNode,
    id::AbstractID = node.id,
) = replace_node!(sched, node, id)

function replace_in_schedule!(
    sched::P,
    path_spec::T,
    pred,
    id::ID,
) where {P<:OperatingSchedule,T<:PathSpec,ID<:AbstractID}
    replace_in_schedule!(sched, ScheduleNode(id, pred, path_spec))
end

function replace_in_schedule!(
    sched::P,
    spec,
    pred,
    id::ID,
) where {P<:OperatingSchedule,ID<:AbstractID}
    replace_in_schedule!(sched, generate_path_spec(sched, spec, pred), pred, id)
end

function replace_in_schedule!(
    sched::P,
    pred,
    id::ID,
) where {P<:OperatingSchedule,ID<:AbstractID}
    replace_in_schedule!(sched, ProblemSpec(), pred, id)
end

add_node!(sched::OperatingSchedule, node::ScheduleNode) = add_node!(sched, node, node.id)

function validate(sched::OperatingSchedule)
    try
        @assert !is_cyclic(sched) "is_cyclic(G)"
        for e in edges(sched)
            node1 = get_node_from_id(sched, get_vtx_id(sched, e.src))
            node2 = get_node_from_id(sched, get_vtx_id(sched, e.dst))
            @assert(
                validate_edge(node1, node2),
                string(" INVALID EDGE: ", string(node1), " --> ", string(node2))
            )
        end
        for v in Graphs.vertices(sched)
            id = get_vtx_id(sched, v)
            node = get_node_from_id(sched, id)
            @assert(
                outdegree(sched, v) >= sum([0, values(required_successors(node))...]),
                string("outdegree = ", outdegree(sched, v), " for node = ", string(node))
            )
            @assert(
                indegree(sched, v) >= sum([0, values(required_predecessors(node))...]),
                string("indegree = ", indegree(sched, v), " for node = ", string(node))
            )
            if matches_node_type(node, AbstractSingleRobotAction)
                for v2 in outneighbors(sched, v)
                    node2 = get_node_from_vtx(sched, v2)
                    if matches_node_type(node2, AbstractSingleRobotAction)
                        if length(
                            intersect(resources_reserved(node), resources_reserved(node2)),
                        ) == 0  # job shop constraint
                            @assert(
                                get_robot_id(node) == get_robot_id(node2),
                                string(
                                    "robot IDs do not match: ",
                                    string(node),
                                    " => ",
                                    string(node2),
                                )
                            )
                        end
                    end
                end
            end
        end
    catch e
        if typeof(e) <: AssertionError
            bt = catch_backtrace()
            @info "Schedule is invalid"
            log_schedule_edges(sched)
            showerror(stdout, e, bt)
            print(e.msg)
        else
            rethrow(e)
        end
        return false
    end
    return true
end

"""
    update_schedule_times!(sched::OperatingSchedule)

Compute start and end times for all nodes based on the end times of their
inneighbors and their own durations.
"""
function update_schedule_times!(sched::OperatingSchedule)
    G = get_graph(sched)
    for v in topological_sort_by_dfs(G)
        t0 = get_t0(sched, v)
        for v2 in inneighbors(G, v)
            t0 = max(t0, get_tF(sched, v2))
        end
        set_t0!(sched, v, t0)
        set_tF!(sched, v, max(get_tF(sched, v), t0 + get_min_duration(sched, v)))
    end
    return sched
end

function update_slack!(sched::OperatingSchedule)
    G = get_graph(sched)
    n_roots = max(length(sched.terminal_vtxs), 1)
    slack = map(i -> Inf * ones(n_roots), Graphs.vertices(G))
    local_slack = map(i -> Inf * ones(n_roots), Graphs.vertices(G))
    for (i, v) in enumerate(sched.terminal_vtxs)
        slack[v][i] = 0 # only slack for corresponding head is set to 0
    end
    for v in reverse(topological_sort_by_dfs(G))
        for v2 in outneighbors(G, v)
            local_slack[v] = min.(local_slack[v], (get_t0(sched, v2) - get_tF(sched, v)))
            slack[v] = min.(slack[v], slack[v2] .+ (get_t0(sched, v2) - get_tF(sched, v)))
        end
    end
    for v in Graphs.vertices(sched)
        set_slack!(sched, v, slack[v])
        set_local_slack!(sched, v, local_slack[v])
    end
    return sched
end

function process_schedule!(sched)
    update_schedule_times!(sched)
    update_slack!(sched)
end

makespan(sched::OperatingSchedule) = maximum(get_tF(sched))
#################################################
abstract type AbstractPlanningPredicate end
abstract type AbstractRobotAction{R<:AbstractRobotType} <: AbstractPlanningPredicate end
abstract type AbstractSingleRobotAction{R<:AbstractRobotType} <: AbstractRobotAction{R} end
abstract type AbstractTeamRobotAction{R<:AbstractRobotType} <: AbstractRobotAction{R} end

Base.copy(p::AbstractPlanningPredicate) = deepcopy(p)
robot_type(a::AbstractRobotAction{R}) where {R} = R
robot_type(a) = Nothing

# TODO: Check if we need this here, or better someplace else
graph_key() = Symbol(DefaultRobotType)

function graph_key(a)
    if robot_type(a) == Nothing
        return graph_key()
    else
        return Symbol(robot_type(a))
    end
end

has_robot_id(a) = robot_type(a) == Nothing ? false : true

"""
	robot_ids_match(node,node2)

Checks if robot_ids match between the nodes
"""
function robot_ids_match(node, node2)
    if has_robot_id(node) && has_robot_id(node2)
        if get_id(get_robot_id(node)) != -1 && get_id(get_robot_id(node2)) != -1
            status = (get_robot_id(node) == get_robot_id(node2)) ? true : false
            return status
        end
    end
    return true
end

"""
	align_with_predecessor(node,succ)

Modifies a node to match the information encoded by its predecessor. This is
how e.g., robot ids are propagated through an existing operating schedule
after assignments (or re-assignments) have been made.
"""
align_with_predecessor(node, pred) = node

"""
	align_with_successor(node,succ)

Modifies a node to match the information encoded by its successor. This is
how e.g., robot ids are propagated through an existing operating schedule
after assignments (or re-assignments) have been made.
"""
align_with_successor(node, succ) = node
is_valid(id::A) where {A<:AbstractID} = valid_id(id) #get_id(id) != -1
first_valid(a, b) = is_valid(a) ? a : b
align_with_predecessor(graph, node, pred) = align_with_predecessor(node, pred)
align_with_successor(graph, node, pred) = align_with_successor(node, pred)

"""
    matches_node_type(::A,::Type{B}) where {A<:AbstractPlanningPredicate,B}

Returns true if {A <: B}
"""
matches_node_type(::A, ::Type{B}) where {A,B} = A <: B

#################################################
"""
    AbstractCostModel{T}
"""
abstract type AbstractCostModel{T} end

cost_type(model::M) where {T,M<:AbstractCostModel{T}} = T

"""
    aggregate_costs(model, costs::Vector{T}) where {T}

Defines how costs from multiple distinct paths are combined into a single cost
for the whole solution. For example, the aggregation function for a
`SumOfTravelTime` objective is the `sum` of individual cost.
"""
function aggregate_costs end

"""
    a special version of aggregate_costs for the meta_env
"""
aggregate_costs_meta(m::AbstractCostModel, args...) = aggregate_costs(m, args...)

abstract type AggregationFunction end

struct MaxCost <: AggregationFunction end

(f::MaxCost)(costs...) = maximum(costs...)

struct SumCost <: AggregationFunction end
(f::SumCost)(costs...) = sum(costs...)

"""
    LowLevelCostModel{C}

The low level cost model defines the objective to be optimized by the
solver at the low level. An optimal low level solver will return a path if a
feasible path exists) of minimal cost under the objective specified by the
associated LowLevelCostModel.
The parameter 'C' defines the cost_type of the objective. The following
functions must be implemented for a 'LowLevelCostModel' to be used:
- get_initial_cost(model::LowLevelCostModel,env) - returns
the initial_cost for a path
- get_transition_cost(model::LowLevelCostModel{C},path::Path,s::S,a::A,
    sp::S) where {S,A,C}' - defines the cost associated with taking action
    'a' from state 's' to arrive in state 'sp' according to the objective
    defined by 'model' given that 's' is the "tip" of 'path'.
- accumulate_cost(model::LowLevelCostModel{C}, current_cost::C,
    transition_cost::C) - defines how cost accumulates as new PathNodes
    are added to a Path.
"""
abstract type LowLevelCostModel{T} <: AbstractCostModel{T} end

"""
    TravelTime <: LowLevelCostModel{Float64}

Cost model that assigns cost equal to the duration of a path.
"""
struct TravelTime <: LowLevelCostModel{Float64} end

abstract type AbstractDeadlineCost <: LowLevelCostModel{Float64} end

"""
    DeadlineCost

Identical to `TravelTime`, except for the behavior of
`add_heuristic_cost`.

add_heuristic_cost: `c = max(0.0, t + Δt - deadline)`
"""
mutable struct DeadlineCost <: AbstractDeadlineCost
    deadline::Float64 # deadline
    m::TravelTime
end

DeadlineCost(deadline::R) where {R<:Real} = DeadlineCost(deadline, TravelTime())
function set_deadline!(m::DeadlineCost, t_max)
    m.deadline = minimum(t_max)
    return m
end

set_deadline!(m::C, args...) where {C<:AbstractCostModel} = nothing
add_heuristic_cost(m::C, cost, h_cost) where {C<:DeadlineCost} =
    max(0.0, cost + h_cost - m.deadline) # assumes heuristic is PerfectHeuristic
for op in [:accumulate_cost, :get_initial_cost, :get_transition_cost, :compute_path_cost]
    @eval $op(model::AbstractDeadlineCost, args...) = $op(model.m, args...)
end

"""
    MultiDeadlineCost

Combines multiple deadlines according to some specified aggregation function.
"""
struct MultiDeadlineCost{F} <: AbstractDeadlineCost
    f::F # aggregation function
    tF::Vector{Float64}
    root_nodes::Vector{Int} # weights and deadlines correspond to root_nodes only
    weights::Vector{Float64}
    deadlines::Vector{Float64}
    m::TravelTime
end

const SumOfMakeSpans = MultiDeadlineCost{SumCost}
const MakeSpan = MultiDeadlineCost{MaxCost}
SumOfMakeSpans(tF, root_nodes, weights, deadlines) = MultiDeadlineCost(
    SumCost(),
    Float64.(tF),
    root_nodes,
    Float64.(weights),
    Float64.(deadlines),
    TravelTime(),
)
SumOfMakeSpans() =
    MultiDeadlineCost(SumCost(), Float64[], Int[], Float64[], Float64[], TravelTime())
MakeSpan(tF, root_nodes, weights, deadlines) = MultiDeadlineCost(
    MaxCost(),
    Float64.(tF),
    root_nodes,
    Float64.(weights),
    Float64.(deadlines),
    TravelTime(),
)
MakeSpan() =
    MultiDeadlineCost(MaxCost(), Float64[], Int[], Float64[], Float64[], TravelTime())

function set_deadline!(m::M, t_max) where {M<:MultiDeadlineCost}
    m.deadlines .= t_max
    return m
end

add_heuristic_cost(m::C, cost, h_cost) where {C<:MultiDeadlineCost} =
    m.f(m.weights .* max.(0.0, cost .+ h_cost .- m.deadlines)) # assumes heuristic is PerfectHeuristic
aggregate_costs(m::C, costs::Vector{T}) where {T,C<:MultiDeadlineCost} =
    m.f(m.tF[m.root_nodes] .* m.weights)
aggregate_costs_meta(m::C, costs::Vector{T}) where {T,C<:MultiDeadlineCost} = maximum(costs)
#################################################
"""
    TaskGraphsMILP

Concrete subtypes of `TaskGraphsMILP` define different ways to formulate the
sequential assignment portion of a PC-TAPF problem.
"""
abstract type TaskGraphsMILP end

for op in [
    :(JuMP.optimize!),
    :(JuMP.termination_status),
    :(JuMP.objective_function),
    :(JuMP.objective_bound),
    :(JuMP.primal_status),
    :(JuMP.dual_status),
    :(JuMP.set_silent),
]
    @eval $op(milp::TaskGraphsMILP) = $op(milp.model)
end
for op in [
    :(JuMP.set_optimizer_attribute),
    :(JuMP.set_optimizer_attributes),
    :(JuMP.set_time_limit_sec),
]
    @eval $op(milp::TaskGraphsMILP, args...) = $op(milp.model, args...)
end

"""
    AssignmentMILP <: TaskGraphsMILP

Used to formulate a MILP where the decision variable is a matrix `X`, where
`X[i,j] = 1` means that robot `i` is assigned to delivery task `j`. The
dimensionality of `X` is (N+M) × M, where N is the number of robots and M is the
number of delivery tasks. the last M rows of `X` correspond to "dummy robots",
i.e. the N+jth row corresponds to "the robot that already completed task j". The
use of these dummy robot variables allows the sequential assignment problem to
be posed as a one-off assignment problem with inter-task constraints.
"""
@with_kw struct AssignmentMILP <: TaskGraphsMILP
    model::JuMP.Model = Model()
    sched::OperatingSchedule = OperatingSchedule()
    robot_ics::Vector{Pair{RobotID,ScheduleNode}} =
        sort(collect(get_nodes_of_type(sched, BotID)); by = p -> p.first)
    object_ics::Vector{Pair{ObjectID,ScheduleNode}} =
        sort(collect(get_nodes_of_type(sched, ObjectID)); by = p -> p.first)
    operations::Vector{Pair{OperationID,ScheduleNode}} =
        sort(collect(get_nodes_of_type(sched, OperationID)); by = p -> p.first)
    robot_map::Dict{BotID,Int} =
        Dict{BotID,Int}(p.first => k for (k, p) in enumerate(robot_ics))
    object_map::Dict{ObjectID,Int} =
        Dict{ObjectID,Int}(p.first => k for (k, p) in enumerate(object_ics))
    operation_map::Dict{OperationID,Int} =
        Dict{OperationID,Int}(p.first => k for (k, p) in enumerate(operations))
end

AssignmentMILP(model::JuMP.Model) = AssignmentMILP(model = model)

"""
    SparseAdjacencyMILP <: TaskGraphsMILP

Formulates a MILP where the decision variable is a sparse adjacency matrix `X`
    for the operating schedule graph. If `X[i,j] = 1`, there is an edge from
    node `i` to node `j`.
Experiments have shown that the sparse matrix approach leads to much faster
solve times than the dense matrix approach.
"""
@with_kw struct SparseAdjacencyMILP <: TaskGraphsMILP
    model::JuMP.Model = Model()
    Xa::SparseMatrixCSC{VariableRef,Int} =
        SparseMatrixCSC{VariableRef,Int}(0, 0, ones(Int, 1), Int[], VariableRef[]) # assignment adjacency matrix
    Xj::SparseMatrixCSC{VariableRef,Int} =
        SparseMatrixCSC{VariableRef,Int}(0, 0, ones(Int, 1), Int[], VariableRef[]) # job shop adjacency matrix
    job_shop::Bool = false
end

function get_assignment_matrix(model::M) where {M<:JuMP.Model}
    Matrix{Int}(min.(1, round.(value.(model[:X])))) # guarantees binary matrix
end

get_assignment_matrix(model::TaskGraphsMILP) = get_assignment_matrix(model.model)

"""
    preprocess_project_schedule(sched)

Returns information about the eligible and required successors and predecessors
of nodes in `sched`

Arguments:
- `sched::OperatingSchedule`

Outputs:
- missing_successors
- missing_predecessors
- n_eligible_successors
- n_eligible_predecessors
- n_required_successors
- n_required_predecessors
- upstream_vertices
- non_upstream_vertices

TODO: OBJECT_AT nodes should always have the properties that
`indegree(G,v) == n_required_predecessors(v) == n_eligible_predecessors(v)`
`outdegree(G,v) == n_required_successors(v) == n_eligible_successors(v)`
Not sure if this is currently the case. UPDATE: I believe this has already been
    addressed by making each object come from an initial operation.
"""
function preprocess_project_schedule(sched)
    G = get_graph(sched)
    # Identify required and eligible edges
    missing_successors = Dict{Int,Dict}()
    missing_predecessors = Dict{Int,Dict}()
    n_eligible_successors = zeros(Int, nv(G))
    n_eligible_predecessors = zeros(Int, nv(G))
    n_required_successors = zeros(Int, nv(G))
    n_required_predecessors = zeros(Int, nv(G))
    for v in Graphs.vertices(G)
        node = get_node_from_id(sched, get_vtx_id(sched, v))
        for (key, val) in required_successors(node)
            n_required_successors[v] += val
        end
        for (key, val) in required_predecessors(node)
            n_required_predecessors[v] += val
        end
        for (key, val) in eligible_successors(node)
            n_eligible_successors[v] += val
        end
        for (key, val) in eligible_predecessors(node)
            n_eligible_predecessors[v] += val
        end
        missing_successors[v] = eligible_successors(node)
        for v2 in outneighbors(G, v)
            id2 = get_vtx_id(sched, v2)
            node2 = get_node_from_id(sched, id2)
            for key in collect(keys(missing_successors[v]))
                if matches_template(key, typeof(node2))
                    missing_successors[v][key] -= 1
                    break
                end
            end
        end
        missing_predecessors[v] = eligible_predecessors(node)
        for v2 in inneighbors(G, v)
            id2 = get_vtx_id(sched, v2)
            node2 = get_node_from_id(sched, id2)
            for key in collect(keys(missing_predecessors[v]))
                if matches_template(key, typeof(node2))
                    missing_predecessors[v][key] -= 1
                    break
                end
            end
        end
    end
    @assert(!any(n_eligible_predecessors .< n_required_predecessors))
    @assert(!any(n_eligible_successors .< n_required_successors))
    upstream_vertices = map(
        v -> [v, map(e -> e.dst, collect(edges(bfs_tree(G, v; dir = :in))))...],
        Graphs.vertices(G),
    )
    non_upstream_vertices = map(
        v -> collect(setdiff(collect(Graphs.vertices(G)), upstream_vertices[v])),
        Graphs.vertices(G),
    )

    return missing_successors,
    missing_predecessors,
    n_eligible_successors,
    n_eligible_predecessors,
    n_required_successors,
    n_required_predecessors,
    upstream_vertices,
    non_upstream_vertices
end

"""
    formulate_milp(milp_model::AssignmentMILP,sched,problem_spec;kwargs...)

Express the TaskGraphs assignment problem as an `AssignmentMILP` using the JuMP
optimization framework.

Inputs:
    milp_model::T <: TaskGraphsMILP : a milp model that determines how the
        sequential task assignment problem is modeled. Current options are
        `AssignmentMILP`, `SparseAdjacencyMILP` and `GreedyAssignment`.
    sched::OperatingSchedule : a partial operating schedule, where
        some or all assignment edges may be missing.
    problem_spec::ProblemSpec : encodes the distance matrix and other
        information about the problem.

Keyword Args:
    `optimizer` - a JuMP optimizer (e.g., Gurobi.optimizer)
    `cost_model=MakeSpan` - optimization objective, currently either `MakeSpan`
        or `SumOfMakeSpans`. Defaults to the cost_model associated with
        `problem_spec`
Outputs:
    `model::AssignmentMILP` - an instantiated optimization problem
"""
function formulate_milp(
    milp_model::AssignmentMILP,
    sched::OperatingSchedule,
    problem_spec::ProblemSpec;
    optimizer = default_milp_optimizer(),
    cost_model = problem_spec.cost_function,
    Mm = 10000,
    kwargs...,
)
    # SETUP
    # Define optimization model
    model = Model(optimizer_with_attributes(optimizer))
    set_optimizer_attributes(model, default_milp_optimizer_attributes()...)
    milp = AssignmentMILP(model = model, sched = sched)
    @unpack robot_ics, object_ics, operations, robot_map, object_map, operation_map = milp
    N = length(robot_ics)  # number of robots
    M = length(object_ics)  # number of delivery tasks
    Δt_collect = zeros(Int, M)
    Δt_deposit = zeros(Int, M)
    r0 = map(p -> get_id(get_initial_location_id(p.second.node)), robot_ics)  # initial robot locations
    s0 = map(p -> get_id(get_initial_location_id(p.second.node)), object_ics)  # initial object locations
    sF = zeros(Int, M)
    for (v, n) in enumerate(get_nodes(sched))
        if matches_node_type(n, BOT_COLLECT)
            Δt_collect[object_map[get_object_id(n)]] = get_min_duration(n)
        elseif matches_node_type(n, BOT_DEPOSIT)
            Δt_deposit[object_map[get_object_id(n)]] = get_min_duration(n)
            sF[object_map[get_object_id(n)]] = get_id(get_destination_location_id(n))
        end
    end
    # From ProblemSpec
    D = (x, y) -> get_distance(problem_spec, x, y)
    @variable(model, to0[1:M] >= 0.0)  # object availability time
    @variable(model, tor[1:M] >= 0.0)  # object robot arrival time
    @variable(model, toc[1:M] >= 0.0)  # object collection complete time
    @variable(model, tod[1:M] >= 0.0)  # object deliver begin time
    @variable(model, tof[1:M] >= 0.0)  # object termination time
    @variable(model, tr0[1:N+M] >= 0.0)  # robot availability time
    # Assignment matrix x
    @variable(model, X[1:N+M, 1:M], binary = true)  # X[i,j] ∈ {0,1}
    @constraint(model, X * ones(M) .<= 1)  # each robot may have no more than 1 task
    @constraint(model, X' * ones(N + M) .== 1)  # each task must have exactly 1 assignment
    for (id, node) in robot_ics  # robot start times
        @constraint(model, tr0[robot_map[id]] == get_tF(node))
    end
    for (id, node) in object_ics  # task start times
        if is_root_node(sched, id)  # only applies to root tasks (with no prereqs)
            @constraint(model, to0[object_map[id]] == get_tF(node))
        end
    end
    precedence_graph = CustomNDiGraph{Nothing,ObjectID}()
    for (id, _) in object_ics
        add_node!(precedence_graph, nothing, id)
    end
    for (op_id, node) in operations  # get_operations(sched) # precedence constraints on task start time
        op = node.node
        for (_, input) in preconditions(op)
            i = object_map[get_object_id(input)]
            for (_, output) in postconditions(op)
                j = object_map[get_object_id(output)]
                @constraint(model, to0[j] >= tof[i] + duration(op))
                add_edge!(precedence_graph, get_object_id(input), get_object_id(output))
            end
        end
    end
    # Propagate upstream edges through precedence graph
    for v in topological_sort_by_dfs(precedence_graph)
        for v1 in inneighbors(precedence_graph, v)
            for v2 in outneighbors(precedence_graph, v)
                add_edge!(precedence_graph, v1, v2)
            end
        end
        add_edge!(precedence_graph, v, v)
    end
    # Constraints
    r0 = vcat(r0, sF)  # combine to get dummy robot ``spawn'' locations too
    for j = 1:M
        # Constraint on dummy robot start time (corresponds to moment of object delivery)
        @constraint(model, tr0[j+N] == tof[j])
        # Dummy robots can't do upstream jobs
        for v in inneighbors(precedence_graph, j)
            @constraint(model, X[j+N, v] == 0)
        end
        # Lower bound on task completion time (task can't start until it's available).
        @constraint(model, tor[j] >= to0[j])
        for i = 1:N+M
            @constraint(model, tor[j] - (tr0[i] + D(r0[i], s0[j])) >= -Mm * (1 - X[i, j]))
        end
        @constraint(model, toc[j] == tor[j] + Δt_collect[j])
        @constraint(model, tod[j] == toc[j] + D(s0[j], sF[j]))
        @constraint(model, tof[j] == tod[j] + Δt_deposit[j])
        """ "Job-shop" constraints specifying that no station may be double-booked. A station can only support a single COLLECT or DEPOSIT operation at a time, meaning that all the windows for these operations cannot overlap. In the constraints below, t1 and t2 represent the intervals for the COLLECT or DEPOSIT operations of tasks j and j2,
        respectively. If eny of the operations for these two tasks require use of the same station, we introduce a 2D binary variable y. if y = [1,0], the operation for task j must occur before the operation for task j2. The opposite is true for y == [0,1]. We use the big M method here as well to tightly enforce the binary constraints.
        """
        for j2 = j+1:M
            if (s0[j] == s0[j2]) ||
               (s0[j] == sF[j2]) ||
               (sF[j] == s0[j2]) ||
               (sF[j] == sF[j2])
                # @show j, j2
                if s0[j] == s0[j2]
                    t1 = [tor[j], toc[j]]
                    t2 = [tor[j2], toc[j2]]
                elseif s0[j] == sF[j2]
                    t1 = [tor[j], toc[j]]
                    t2 = [tod[j2], tof[j2]]
                elseif sF[j] == s0[j2]
                    t1 = [tod[j], tof[j]]
                    t2 = [tor[j2], toc[j2]]
                elseif sF[j] == sF[j2]
                    t1 = [tod, tof[j]]
                    t2 = [tod, tof[j2]]
                end
                tmax = @variable(model)
                tmin = @variable(model)
                y = @variable(model, binary = true)
                @constraint(model, tmax >= t1[1])
                @constraint(model, tmax >= t2[1])
                @constraint(model, tmin <= t1[2])
                @constraint(model, tmin <= t2[2])

                @constraint(model, tmax - t2[1] <= (1 - y) * Mm)
                @constraint(model, tmax - t1[1] <= y * Mm)
                @constraint(model, tmin - t1[2] >= (1 - y) * -Mm)
                @constraint(model, tmin - t2[2] >= y * -Mm)
                @constraint(model, tmin + 1 - X[j+N, j2] - X[j2+N, j] <= tmax) # NOTE +1 not necessary if the same robot is doing both
            end
        end
    end
    cost = get_objective_expr(milp, cost_model)
    @objective(model, Min, cost)
    return milp
end

"""
    adj_mat_from_assignment_mat(sched,assignment_matrix)

Compute an adjacency matrix from an assignment matrix
"""
function adj_mat_from_assignment_mat(
    model::AssignmentMILP,
    sched::OperatingSchedule,
    assignment_matrix,
)
    M = size(assignment_matrix, 2)
    N = size(assignment_matrix, 1) - M
    @assert N == length(get_robot_ICs(sched))
    @unpack robot_ics, object_ics = model
    assignment_dict = get_assignment_dict(assignment_matrix, N, M)
    G = get_graph(sched)
    adj_matrix = adjacency_matrix(G)
    for (robot_idx, task_list) in assignment_dict
        robot_id = get_robot_id(robot_ics[robot_idx].second)
        robot_node = robot_ics[robot_idx].second
        if is_terminal_node(sched, robot_node)
            v_go = get_vtx(sched, robot_node)
        else
            v_go = outneighbors(sched, robot_node)[1]  # GO_NODE
        end
        if !is_terminal_node(sched, v_go)
            log_schedule_edges(sched)
            @assert is_terminal_node(sched, v_go) "!is_terminal_node($(string(get_node(sched,v_go).node)))"
        end
        for object_idx in task_list
            object_id = get_object_id(object_ics[object_idx].second)
            v_collect = outneighbors(sched, object_id)[1]
            adj_matrix[v_go, v_collect] = 1
            v_carry = outneighbors(sched, v_collect)[1]
            v_deposit = outneighbors(sched, v_carry)[1]
            for v in outneighbors(sched, v_deposit)
                if isa(get_vtx_id(sched, v), ActionID)
                    v_go = v
                    break
                end
            end
        end
    end
    return adj_matrix
end

function formulate_milp(
    milp_model::SparseAdjacencyMILP,
    sched,
    problem_spec;
    warm_start_soln::Union{SparseMatrixCSC,Nothing} = nothing,
    optimizer = default_milp_optimizer(),
    t0_ = Dict{AbstractID,Float64}(),  # dictionary of initial times. Default is empty
    tF_ = Dict{AbstractID,Float64}(),  # dictionary of initial times. Default is empty
    Mm = 10000,  # for big M constraints
    cost_model = SumOfMakeSpans(),
    job_shop = milp_model.job_shop,
    kwargs...,
)
    warm_start = false
    if !isnothing(warm_start_soln)
        warm_start = true
    end
    model = Model(optimizer_with_attributes(optimizer))
    set_optimizer_attributes(model, default_milp_optimizer_attributes()...)
    G = get_graph(sched)
    (
        missing_successors,
        missing_predecessors,
        n_eligible_successors,
        n_eligible_predecessors,
        n_required_successors,
        n_required_predecessors,
        upstream_vertices,
        non_upstream_vertices,
    ) = preprocess_project_schedule(sched)
    Δt = get_min_duration(sched)
    @variable(model, t0[1:nv(sched)] >= 0.0)  # initial times for all nodes
    @variable(model, tF[1:nv(sched)] >= 0.0)  # final times for all nodes
    # Precedence relationships
    Xa = SparseMatrixCSC{VariableRef,Int}(
        nv(sched),
        nv(sched),
        ones(Int, nv(sched) + 1),
        Int[],
        VariableRef[],
    )
    # Set all initial times that are provided
    for (id, t) in t0_
        v = get_vtx(sched, id)
        @constraint(model, t0[v] >= t)
    end
    for (id, t) in tF_
        v = get_vtx(sched, id)
        @constraint(model, tF[v] >= t)
    end
    # Precedence constraints and duration constraints for existing nodes and edges
    for v in Graphs.vertices(sched)
        @constraint(model, tF[v] >= t0[v] + Δt[v])  # NOTE: Δt may change for some nodes
        for v2 in outneighbors(sched, v)
            if warm_start
                Xa[v, v2] = @variable(model, binary = true, start = warm_start_soln[v, v2])
            else
                Xa[v, v2] = @variable(model, binary = true)
            end
            @constraint(model, Xa[v, v2] == 1)  # TODO: this edge already exists--no reason to encode it as a decision variable
            @constraint(model, t0[v2] >= tF[v])  # NOTE: DO NOT CHANGE TO EQUALITY CONSTRAINT. Making this an equality constraint causes the solver to return a higher final value in some cases (e.g., toy problems 2,3,7). Why? Maybe the Big-M constraint forces it to bump up. I though the equality constraint might speed up the solver.
        end
    end
    # Big M constraints
    for v in Graphs.vertices(sched)
        node = get_node_from_id(sched, get_vtx_id(sched, v))
        potential_match = false
        if outdegree(sched, v) < n_eligible_successors[v]  # NOTE: Trying this out to save time on formulation
            for v2 in non_upstream_vertices[v]  # for v2 in Graphs.vertices(sched)
                if indegree(sched, v2) < n_eligible_predecessors[v2]
                    node2 = get_node_from_id(sched, get_vtx_id(sched, v2))
                    for (template, val) in missing_successors[v]
                        if !matches_template(template, typeof(node2))  # possible to add an edge
                            continue
                        end
                        for (template2, val2) in missing_predecessors[v2]
                            if !matches_template(template2, typeof(node))  # possible to add an edge
                                continue
                            end
                            if (val > 0 && val2 > 0)
                                potential_match = true
                                new_node = align_with_successor(node, node2)
                                dt_min =
                                    generate_path_spec(
                                        sched,
                                        problem_spec,
                                        new_node,
                                    ).min_duration
                                if warm_start
                                    Xa[v, v2] = @variable(
                                        model,
                                        binary = true,
                                        start = warm_start_soln[v, v2]
                                    )
                                else
                                    Xa[v, v2] = @variable(model, binary = true)
                                end
                                @constraint(
                                    model,
                                    tF[v] - (t0[v] + dt_min) >= -Mm * (1 - Xa[v, v2])
                                )
                                @constraint(model, t0[v2] - tF[v] >= -Mm * (1 - Xa[v, v2]))
                                break
                            end
                        end
                    end
                end
            end
        end
        if potential_match == false && job_shop == false
            @constraint(model, tF[v] == t0[v] + Δt[v])  # adding this constraint may provide some speedup
        end
    end
    # In the sparse implementation, these constraints must come after all
    # possible edges are defined by a VariableRef
    @constraint(model, Xa * ones(nv(sched)) .<= n_eligible_successors)
    @constraint(model, Xa * ones(nv(sched)) .>= n_required_successors)
    @constraint(model, Xa' * ones(nv(sched)) .<= n_eligible_predecessors)
    @constraint(model, Xa' * ones(nv(sched)) .>= n_required_predecessors)
    for i = 1:nv(sched)
        for j = i:nv(sched)
            # Prevent self-edges and cycles
            @constraint(model, Xa[i, j] + Xa[j, i] <= 1)
        end
    end

    """
    "Job-shop" constraints specifying that no station may be double-booked. A station can only support a single COLLECT or DEPOSIT operation at a time, meaning that all the windows for these operations cannot overlap. In the constraints below, t1 and t2 represent the intervals for the COLLECT or DEPOSIT operations of tasks j and j2, respectively. If eny of the operations for these two tasks require use of the same station, we introduce a 2D binary variable y. if y = [1,0], the operation for task
    j must occur before the operation for task j2. The opposite is true for y == [0,1]. We use the big M method here as well to tightly enforce the binary constraints. 

    Example: job_shop_variables = Dict{Tuple{Int,Int},JuMP.VariableRef}();
    """
    Xj = SparseMatrixCSC{VariableRef,Int}(
        nv(sched),
        nv(sched),
        ones(Int, nv(sched) + 1),
        Int[],
        VariableRef[],
    )
    if job_shop
        for v = 1:nv(sched)
            node = get_node_from_id(sched, get_vtx_id(sched, v))
            for v2 in non_upstream_vertices[v] #v+1:nv(sched)
                if v2 > v &&
                   ~(v in upstream_vertices[v2]) &&
                   ~(has_edge(sched, v, v2) || has_edge(sched, v2, v))
                    node2 = get_node_from_id(sched, get_vtx_id(sched, v2))
                    common_resources =
                        intersect(resources_reserved(node), resources_reserved(node2))
                    if length(common_resources) > 0
                        println(
                            "MILP FORMULATION: adding a job shop constraint between ",
                            v,
                            " (",
                            string(node),
                            ") and ",
                            v2,
                            " (",
                            string(node2),
                            ")",
                        )
                        # Big M constraints
                        Xj[v, v2] = @variable(model, binary = true)
                        Xj[v2, v] = @variable(model, binary = true)  # need both directions to have a valid adjacency matrix
                        @constraint(model, Xj[v, v2] + Xj[v2, v] == 1)
                        @constraint(model, t0[v2] - tF[v] >= -Mm * (1 - Xj[v, v2]))
                        @constraint(model, t0[v] - tF[v2] >= -Mm * (1 - Xj[v2, v]))
                    end
                end
            end
        end
    end
    # Full adjacency matrix
    # TODO: make X an expression rather than a variable
    @expression(model, X, Xa .+ Xj)
    milp = SparseAdjacencyMILP(model, Xa, Xj, milp_model.job_shop)  #, job_shop_variables
    cost1 = get_objective_expr(milp, cost_model, milp.model, sched, tF)
    @objective(milp.model, Min, cost1)
    return milp
end

function get_objective_expr(milp, f::SumOfMakeSpans, model, sched, tF)
    terminal_vtxs = sched.terminal_vtxs
    if isempty(terminal_vtxs)
        @warn "sched.terminal_vtxs is empty. Using get_all_terminal_nodes(sched) instead"
        terminal_vtxs = get_all_terminal_nodes(sched)
    end
    @variable(model, T[1:length(terminal_vtxs)])
    for (i, project_head) in enumerate(terminal_vtxs)
        for v in project_head
            @constraint(model, T[i] >= tF[v])
        end
    end
    cost1 =
        @expression(model, sum(map(v -> tF[v] * get(sched.weights, v, 1.0), terminal_vtxs)))
end

abstract type AbstractGreedyAssignment <: TaskGraphsMILP end
abstract type GreedyCost end
struct GreedyPathLengthCost <: GreedyCost end
struct GreedyFinalTimeCost <: GreedyCost end
struct GreedyLowerBoundCost <: GreedyCost end

"""
    GreedyAssignment{C,M} <: TaskGraphsMILP

GreedyAssignment maintains three sets: The "satisfied set" `C`, the "required
incoming" set `Ai`, and the "available outgoing" set `Ao`.

At each step, the algorithm identifies the nodes `v1 ∈ Ai` and `v2 ∈ Ao` with
shortest "distance" (in the context of `OperatingSchedule`s, this distance
refers to the duration of `v1` if an edge `v1 → v2` is added) and adds an edge
between them. The distance corresponding to an ineligible edge is set to Inf.

After adding the edge, the algorithm sweeps through a topological ordering of
the vertices and updates `C`, `Ai`, and `Ao`. In order for `v` to be placed in
`C`, `v` must have enough incoming edges and all of `v`'s predecessors must
already be in `C`. In order to be added to `Ai`, `v` must have less than the
required number of incoming edges and all of `v`'s predecessors must
already be in `C`. In order for `v` to be added to `Ao`, `v` must have less than
the allowable number of outgoing edges, and must be in `C`.
"""
@with_kw struct GreedyAssignment{C,M,P} <: AbstractGreedyAssignment
    schedule::OperatingSchedule = OperatingSchedule()
    problem_spec::P = ProblemSpec()
    cost_model::C = SumOfMakeSpans()
    greedy_cost::M = GreedyPathLengthCost()
    t0::Vector{Int} = zeros(Int, nv(schedule)) # get_tF(schedule)
end

JuMP.termination_status(::AbstractGreedyAssignment) = MOI.OPTIMAL
JuMP.primal_status(::AbstractGreedyAssignment) = MOI.FEASIBLE_POINT
get_assignment_matrix(model::AbstractGreedyAssignment) =
    adjacency_matrix(get_graph(model.schedule))

"""
    get_best_pair(Ao,Ai,cost_func,filt=(a,b)->true)

Return `argmin v ∈ Ao, v2 ∈ Ai, cost_func(v,v2) s.t. filt(v,v2) == true`
"""
function get_best_pair(Ao, Ai, cost_func, filt = (a, b) -> true)
    cost = Inf
    a = -1
    b = -2
    for v in Ao
        for v2 in Ai
            c = cost_func(v, v2)
            if c < cost && filt(v, v2)
                cost = c
                a = v
                b = v2
            end
        end
    end
    return a, b, cost
end

function greedy_assignment!(model)
    sched = model.schedule
    problem_spec = model.problem_spec
    cache = preprocess_project_schedule(sched, true)
    C = Set{Int}() # Closed set (these nodes have enough predecessors)
    Ai = Set{Int}() # Nodes that don't have enough incoming edges
    Ao = Set{Int}() # Nodes that can have more outgoing edges
    update_greedy_sets!(
        model,
        sched,
        cache,
        Ai,
        Ao,
        C;
        frontier = get_all_root_nodes(sched),
    )
    D = construct_schedule_distance_matrix(sched, problem_spec)
    while length(Ai) > 0
        new_edges = select_next_edges(model, D, Ao, Ai)
        for (v, v2) in new_edges
            setdiff!(Ao, v)
            setdiff!(Ai, v2)
            # Ao[v] = false
            # Ai[v] = false
            add_edge!(sched, v, v2)
            # @info "$(string(node_id(get_node(sched,v)))), $(string(node_id(entity(get_node(sched,v))))) => $(string(node_id(get_node(sched,v2)))), $(string(node_id(entity(get_node(sched,v2)))))"
        end
        update_greedy_sets!(
            model,
            sched,
            cache,
            Ai,
            Ao,
            C;
            frontier = Set{Int}([e[1] for e in new_edges]),
        )
        # update_greedy_sets_vector!(model,sched,cache,Ai,Ao,C;frontier=Set{Int}([e[1] for e in new_edges]))
        update_greedy_cost_model!(model, new_edges)
    end
    set_leaf_operation_vtxs!(sched)
    propagate_valid_ids!(sched, problem_spec)
    model
end
JuMP.optimize!(model::AbstractGreedyAssignment) = greedy_assignment!(model)

function propagate_valid_ids!(sched::OperatingSchedule, problem_spec)
    @assert(is_cyclic(sched) == false, "is_cyclic(sched)")  # string(sparse(adj_matrix))
    # Propagate valid IDs through the schedule
    for v in topological_sort_by_dfs(sched)
        n_id = get_vtx_id(sched, v)
        node = get_node_from_id(sched, n_id)
        for v2 in inneighbors(sched, v)
            node = align_with_predecessor(sched, node, get_node_from_vtx(sched, v2))
        end
        for v2 in outneighbors(sched, v)
            node = align_with_successor(sched, node, get_node_from_vtx(sched, v2))
        end
        path_spec = get_path_spec(sched, v)
        if path_spec.fixed
            replace_in_schedule!(sched, path_spec, node, n_id)
        else
            replace_in_schedule!(sched, problem_spec, node, n_id)
        end
    end
    return true
end

"""
    update_project_schedule!

Args:
- solver
- sched
- adj_matrix - adjacency_matrix encoding the edges that need to be added to
    the project schedule

Adds all required edges to the project graph and modifies all nodes to
reflect the appropriate valid IDs (e.g., `Action` nodes are populated with
the correct `RobotID`s)
Returns `false` if the new edges cause cycles in the project graph.
"""
function update_project_schedule!(
    solver,
    sched::OperatingSchedule,
    problem_spec,
    adj_matrix,
)
    mtx = adjacency_matrix(sched)
    val = update_project_schedule!(sched, problem_spec, adj_matrix)
    val
end

function update_project_schedule!(sched::OperatingSchedule, problem_spec, adj_matrix)
    # Add all new edges to project sched
    G = get_graph(sched)
    # Remove existing edges first, so that there is no carryover between consecutive MILP iterations
    for e in collect(edges(G))
        rem_edge!(G, e)
    end
    # Add all edges encoded by adjacency matrix
    for v in Graphs.vertices(G)
        for v2 in Graphs.vertices(G)
            if adj_matrix[v, v2] >= 1
                add_edge!(G, v, v2)
            end
        end
    end
    try
        propagate_valid_ids!(sched, problem_spec)
        @assert validate(sched)
    catch e
        if isa(e, AssertionError)
            showerror(stdout, e, catch_backtrace())
        else
            rethrow(e)
        end
        return false
    end
    process_schedule!(sched)
    return true
end

"""
    update_project_schedule!(solver,milp_model::M,sched,problem_spec,
        adj_matrix) where {M<:TaskGraphsMILP}

Args:
- milp_model <: TaskGraphsMILP
- sched::OperatingSchedule
- problem_spec::ProblemSpec
- adj_matrix : an adjacency_matrix or (in the case where
    `milp_model::AssignmentMILP`), an assignment matrix

Adds all required edges to the schedule graph and modifies all nodes to
reflect the appropriate valid IDs (e.g., `Action` nodes are populated with
the correct `RobotID`s)
Returns `false` if the new edges cause cycles in the project graph.
"""
function update_project_schedule!(
    solver,
    model::TaskGraphsMILP,
    sched,
    prob_spec,
    adj_matrix = get_assignment_matrix(model),
)
    update_project_schedule!(solver, sched, prob_spec, adj_matrix)
end

function update_project_schedule!(
    solver,
    model::AssignmentMILP,
    sched,
    prob_spec,
    assignment_matrix = get_assignment_matrix(model),
)
    adj_matrix = adj_mat_from_assignment_mat(model, sched, assignment_matrix)
    update_project_schedule!(solver, sched, prob_spec, adj_matrix)
end
#################################################
# Structs for PathNode and Conflict
"""
    PathNode{S,A}

Includes current state `s`, action `a`, next state `sp`.
"""
@with_kw struct PathNode{S,A}
    s::S = S() # state
    a::A = A() # action
    sp::S = S() # next state
end

"""
    Conflict{P1<:PathNode,P2<:PathNode}

Represents a conflict between two path nodes.
"""
@with_kw struct Conflict{P1<:PathNode,P2<:PathNode}
    conflict_type::Symbol = :NullConflict
    agent1_id::Int = -1
    agent2_id::Int = -1
    node1::P1 = P1()
    node2::P2 = P2()
    t::Int = -1
end

# Accessor Methods for PathNode and Conflict
get_s(p::P) where {P<:PathNode} = p.s
get_a(p::P) where {P<:PathNode} = p.a
get_sp(p::P) where {P<:PathNode} = p.sp
state_type(p::PathNode{S,A}) where {S,A} = S
action_type(p::PathNode{S,A}) where {S,A} = A
Base.string(p::PathNode) =
    "$(string(get_s(p))) -- $(string(get_a(p))) -- $(string(get_sp(p)))"
is_valid(c::C) where {C<:Conflict} =
    (agent1_id(c) != agent2_id(c)) && (agent1_id(c) != -1) && (agent2_id(c) != -1)

"""
    PlanningCache

Cache used during planning to store information about processed and active nodes.
"""
@with_kw_noshow struct PlanningCache
    closed_set::Set{Int} = Set{Int}()  # nodes that are completed
    active_set::Set{Int} = Set{Int}()  # active nodes
    node_queue::PriorityQueue{Int,Tuple{Int,Float64}} =
        PriorityQueue{Int,Tuple{Int,Float64}}()  # active nodes prioritized by slack
end

function sprint_cache(io::IO, cache::PlanningCache; label_pad = 14, pad = 5)
    lpad(str) = sprint_padded(str; pad = label_pad, leftaligned = true)
    rpad(str) = sprint_padded(str; pad = label_pad, leftaligned = false)
    spad(str; kwargs...) =
        sprint_padded_list(str; pad = pad, leftaligned = false, kwargs...)
    print(io, "PlanningCache:", "\n")
    print(io, "\t", lpad("closed_set:  "), cache.closed_set, "\n")
    print(io, "\t", lpad("active_set:  "), cache.active_set, "\n")
    print(io, "\t", lpad("node_queue:  "), cache.node_queue, "\n")
end

function Base.show(io::IO, cache::PlanningCache)
    sprint_cache(io, cache)
end

function isps_queue_cost(sched::OperatingSchedule, v::Int)
    path_spec = get_path_spec(sched, v)
    return (Int(path_spec.plan_path), minimum(get_slack(sched, v)))
end

function initialize_planning_cache(sched::OperatingSchedule)
    cache = PlanningCache()
    for v in get_all_root_nodes(sched)
        push!(cache.active_set, v)
        enqueue!(cache.node_queue, v => isps_queue_cost(sched, v))  # need to store slack
    end
    cache
end

"""
    `reset_cache!(cache,sched)`

Resets the cache so that a solution can be repaired (otherwise calling low_level_search!() will return immediately because the cache says it's complete)
"""
function reset_cache!(cache::PlanningCache, sched::OperatingSchedule)
    process_schedule!(sched)
    empty!(cache.closed_set)
    empty!(cache.active_set)
    empty!(cache.node_queue)
    for v in Graphs.vertices(get_graph(sched))
        if is_root_node(get_graph(sched), v)
            push!(cache.active_set, v)
            enqueue!(cache.node_queue, v => isps_queue_cost(sched, v))
        end
    end
    cache
end

"""
update_planning_cache!

Update cache continually. After a call to this function, the start and end times
of all schedule nodes will be updated to reflect the progress of active schedule
nodes (i.e., if a robot had not yet completed a GO task, the predicted final
time for that task will be updated based on the robot's current state and
distance to the goal). All active nodes that don't require planning will be automatically marked as
complete.
"""
function update_planning_cache!(
    solver,
    sched::OperatingSchedule,
    cache::PlanningCache,
    v::Int,
    t = -1,
)
    active_set = cache.active_set
    closed_set = cache.closed_set
    node_queue = cache.node_queue
    Δt = t - get_tF(sched, v)
    if Δt > 0
        set_tF!(sched, v, t)
        process_schedule!(sched)
    end
    # Update closed_set
    activated_vtxs = Int[]
    push!(closed_set, v)
    # Update active_set
    setdiff!(active_set, v)
    for v2 in outneighbors(sched, v)
        active = true
        for v1 in inneighbors(sched, v2)
            if !(v1 in closed_set)
                active = false
                break
            end
        end
        if active
            push!(activated_vtxs, v2)
            push!(active_set, v2)  # add to active set
        end
    end
    # Update priority queue
    for v2 in active_set
        node_queue[v2] = isps_queue_cost(sched, v2)
    end
    return cache
end
#################################################
# Functions Related to MILP Optimization
global MILP_OPTIMIZER = nothing
global DEFAULT_MILP_OPTIMIZER_ATTRIBUTES =
    Dict{Union{String,MOI.AbstractOptimizerAttribute},Any}()

"""
    default_milp_optimizer()

Returns the black box optimizer to be use when formulating JuMP models.
"""
default_milp_optimizer() = MILP_OPTIMIZER

"""
    set_default_milp_optimizer!(optimizer)

Set the black box optimizer to be use when formulating JuMP models.
"""
function set_default_milp_optimizer!(optimizer)
    global MILP_OPTIMIZER = optimizer
end

"""
    default_milp_optimizer_attributes()

Return a dictionary of default optimizer attributes.
"""
default_milp_optimizer_attributes() = DEFAULT_MILP_OPTIMIZER_ATTRIBUTES

"""
    set_default_milp_optimizer_attributes!(vals)

Set default optimizer attributes.
e.g. `set_default_milp_optimizer_attributes!(Dict("PreSolve"=>-1))`
"""
function set_default_milp_optimizer_attributes!(pair::Pair, pairs...)
    push!(DEFAULT_MILP_OPTIMIZER_ATTRIBUTES, pair)
    set_default_milp_optimizer_attributes!(pairs...)
end

set_default_milp_optimizer_attributes!(d::Dict) =
    set_default_milp_optimizer_attributes!(d...)
set_default_milp_optimizer_attributes!() = nothing

"""
    clear_default_milp_optimizer_attributes!()

Clear the default optimizer attributes.
"""
function clear_default_milp_optimizer_attributes!()
    empty!(DEFAULT_MILP_OPTIMIZER_ATTRIBUTES)
end
