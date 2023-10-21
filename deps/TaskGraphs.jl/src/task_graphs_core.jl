export
    get_debug_file_id,
    reset_debug_file_id!,
    validate

DEBUG_ID_COUNTER = 0
get_debug_file_id() = Int(global DEBUG_ID_COUNTER += 1)
function reset_debug_file_id!()
    global DEBUG_ID_COUNTER = 0
end

function validate end


export
    ProblemSpec

"""
    ProblemSpec{G}

Encodes the the relaxed PC-TAPF problem that ignores collision-avoidance
constraints.

Elements:
- D::T - a distance matrix (or environment) that implements get_distance(D,x1,x2,args...)
- cost_function::F - the optimization objective (default is SumOfMakeSpans)
- Δt_collect::Dict{ObjectID,Int} # duration of COLLECT operations
- Δt_deposit::Dict{ObjectID,Int} # duration of DEPOSIT operations
"""
@with_kw struct ProblemSpec{T,F}
    D::T                            = zeros(0,0) # environment or distance matrix
    cost_function::F                = SumOfMakeSpans()
    Δt_collect::Dict{ObjectID,Int}  = Dict{ObjectID,Int}() # duration of COLLECT operations
    Δt_deposit::Dict{ObjectID,Int}  = Dict{ObjectID,Int}() # duration of DELIVER operations
end
GraphUtils.get_distance(spec::ProblemSpec,args...) = get_distance(spec.D,args...)
GraphUtils.get_distance(spec::ProblemSpec,a::LocationID,b::LocationID,args...) = get_distance(spec.D,get_id(a),get_id(b),args...)

export
    ProjectSpec,
    add_operation!,
    set_initial_condition!,
    set_final_condition!,
    get_initial_condition,
    get_final_condition,
    get_duration_vector,
    construct_operation

"""
    ProjectSpec <: AbstractCustomNDiGraph{Union{OBJECT_AT,Operation},AbstractID}

Encodes a set of operations and the prescribed initial and final locations
of the objects that form the inputs and outputs of those operations.

Elements:
- all standard fields of a concrete `GraphUtils.CustomNDiGraph` type
- initial_conditions::Dict{ObjectID,OBJECT_AT}
- final_conditions::Dict{ObjectID,OBJECT_AT}
"""
@with_kw struct ProjectSpec <: AbstractCustomNDiGraph{Union{OBJECT_AT,Operation},AbstractID}
    graph               ::DiGraph               = DiGraph()
    nodes               ::Vector{Union{OBJECT_AT,Operation}} = Vector{Union{OBJECT_AT,Operation}}()
    vtx_map             ::Dict{AbstractID,Int}  = Dict{AbstractID,Int}()
    vtx_ids             ::Vector{AbstractID}    = Vector{AbstractID}()
    initial_conditions  ::Dict{ObjectID,OBJECT_AT} = Dict{ObjectID,OBJECT_AT}()
    final_conditions    ::Dict{ObjectID,OBJECT_AT} = Dict{ObjectID,OBJECT_AT}()
end
get_initial_conditions(spec::ProjectSpec)       = spec.initial_conditions
get_final_conditions(spec::ProjectSpec)         = spec.final_conditions
get_initial_condition(spec::ProjectSpec,id::ObjectID)   = get_initial_conditions(spec)[id]
get_final_condition(spec::ProjectSpec,id::ObjectID)     = get_final_conditions(spec)[id]
initial_conditions_vector(spec::ProjectSpec)    = sort(collect(values(spec.initial_conditions));by=get_object_id)
final_conditions_vector(spec::ProjectSpec)      = sort(collect(values(spec.final_conditions));by=get_object_id)
function set_initial_condition!(spec,object_id,pred)
    spec.initial_conditions[object_id] = pred
end
function set_final_condition!(spec,object_id,pred)
    spec.final_conditions[object_id] = pred
end
for op in [:set_initial_condition!,:set_final_condition!]
    @eval $op(spec,pred) = $op(spec,get_object_id(pred),pred)
end
function ProjectSpec(ics::V,fcs::V) where {V<:Vector{OBJECT_AT}}
    spec = ProjectSpec(
        initial_conditions=Dict{ObjectID,OBJECT_AT}(get_object_id(p)=>p for p in ics),
        final_conditions=Dict{ObjectID,OBJECT_AT}(get_object_id(p)=>p for p in fcs),
        )
    for (k,o) in get_initial_conditions(spec)
        add_node!(spec,o,get_object_id(o))
        set_initial_condition!(spec,get_object_id(o),o)
    end
    return spec
end
get_pre_deps(spec::ProjectSpec, op_id::OperationID) = Set{ObjectID}(map(v->get_vtx_id(spec,v),inneighbors(spec,op_id))) #get(spec.pre_deps, op_id, Set{Int}())
get_post_deps(spec::ProjectSpec, op_id::OperationID) = Set{ObjectID}(map(v->get_vtx_id(spec,v),outneighbors(spec,op_id))) #get(spec.pre_deps, op_id, Set{Int}())
get_operations(spec::ProjectSpec) = Vector{Operation}(filter(n->isa(n,Operation), get_nodes(spec)))
get_num_delivery_tasks(spec::ProjectSpec) = length(get_nodes_of_type(spec,ObjectID))
function get_terminal_operation_ids(spec::ProjectSpec)
    ops = Set{OperationID}()
    for v in get_all_terminal_nodes(spec)
        id = get_vtx_id(spec,v)
        if isa(id,OperationID)
            push!(ops,id)
        end
    end
    return ops
end
function get_pickup_wait_time(spec::ProjectSpec,o::ObjectID)
    if !is_root_node(spec,o)
        parent = get_node(spec,inneighbors(spec,o)[1])
        @assert isa(parent,Operation)
        return duration(parent)
    end
    return 0.0
end
function Base.merge!(spec1::ProjectSpec,spec2::ProjectSpec)
    remap_object_ids!(spec2,spec1)
    for v in vertices(spec2)
        id = get_vtx_id(spec2,v)
        node = get_node(spec2,v)
        add_node!(spec1,node,id)
    end
    for e in edges(spec2)
        add_edge!(spec1,get_vtx_id(spec2,e.src),get_vtx_id(spec2,e.dst))
    end
    merge!(get_initial_conditions(spec1),get_initial_conditions(spec2))
    merge!(get_final_conditions(spec1),get_final_conditions(spec2))
    spec1
end
"""
    get_duration_vector(spec::ProjectSpec)

Return a vector `Δt` such that `Δt[i]` is the amount of time that must elapse
before object `i` can be picked up after its parent operation is performed.
"""
function get_duration_vector(spec::ProjectSpec)
    # Δt = Dict{ObjectID,Float64}()
    # for (v,n) in enumerate(get_nodes(spec))
    #     if isa(n,OBJECT_AT)
    #         Δt[get_object_id(n)] = get_pickup_wait_time(spec,get_object_id(n))
    #     end
    # end
    # Δt
    map(n->get_pickup_wait_time(spec,get_object_id(n)), initial_conditions_vector(spec))
end

"""
    construct_operation(spec::ProjectSpec, station_id, input_ids, output_ids, Δt, id=get_unique_operation_id())
"""
function construct_operation(
        spec::ProjectSpec,
        station_id::LocationID,
        input_ids::Vector{ObjectID},
        output_ids::Vector{ObjectID},
        Δt,
        id=get_unique_id(OperationID)
        )
    op = Operation(
        pre = Dict{ObjectID,OBJECT_AT}(map(id->id=>get_final_condition(spec,id), input_ids)),
        post = Dict{ObjectID,OBJECT_AT}(map(id->id=>get_initial_condition(spec,id), output_ids)),
        Δt = Δt,
        station_id = LocationID(station_id),
        id = id
    )
end
function construct_operation(
        spec::ProjectSpec,
        station_id::Int,
        input_ids::Vector,
        output_ids::Vector,args...)
    construct_operation(spec,
        LocationID(station_id),
        Vector{ObjectID}(convert.(ObjectID,input_ids)),
        Vector{ObjectID}(convert.(ObjectID,output_ids)),
        args...
        )
end
function add_operation!(spec::ProjectSpec, op::Operation)
    add_node!(spec,op,get_operation_id(op))
    op_id = get_operation_id(op)
    for (object_id,pred) in preconditions(op)
        if !has_vertex(spec,object_id)
            add_node!(spec,pred,object_id)
        end
        add_edge!(spec, object_id, op_id)
        set_final_condition!(spec,object_id,pred)
    end
    for (object_id,pred) in postconditions(op)
        if !has_vertex(spec,object_id)
            add_node!(spec,pred,object_id)
        else
            # want the graph nodes to represent the initial conditions of the objects
            replace_node!(spec,pred,object_id)
        end
        add_edge!(spec, op_id, object_id)
        set_initial_condition!(spec,object_id,pred)
    end
    # empty!(spec.terminal_vtxs)
    # union!(spec.terminal_vtxs, get_all_terminal_nodes(spec.graph))
    validate(spec,op)
    spec
end


export
    read_operation,
    read_project_spec

parse_object(pred::OBJECT_AT,dict=Dict{String,Any}()) = merge!(dict,Dict("type"=>"OBJECT_AT","data"=>[get_id(get_object_id(pred)),get_id(get_location_id(pred))]))
TOML.parse(pred::OBJECT_AT) = parse_object(pred)
read_object(dict) = OBJECT_AT(dict["data"]...)
read_object(arr::Vector) = OBJECT_AT(arr...)
function TOML.parse(op::Operation)
    toml_dict = Dict()
    toml_dict["pre"] = map(pred->TOML.parse(pred), collect(values(preconditions(op))))
    toml_dict["post"] = map(pred->TOML.parse(pred), collect(values(postconditions(op))))
    toml_dict["dt"] = duration(op)
    toml_dict["station_id"] = get_id(op.station_id)
    toml_dict["id"] = get_id(op.id)
    return toml_dict
end
function read_op_dict_from_array(toml_dict)
    dict = Dict{ObjectID,OBJECT_AT}()
    for arr in toml_dict
        o = read_object(arr)
        dict[get_object_id(o)] = o
    end
    return dict
end
function read_operation(toml_dict::Dict,keep_id=false)
    op_id = OperationID(get(toml_dict,"id",-1))
    if get_id(op_id) == -1 || keep_id == false
        op_id = get_unique_operation_id()
    end
    op = Operation(
        pre     = read_op_dict_from_array(toml_dict["pre"]),
        post    = read_op_dict_from_array(toml_dict["post"]),
        Δt      = get(toml_dict,"dt",get(toml_dict,"Δt",0.0)),
        station_id = LocationID(get(toml_dict,"station_id",-1)),
        id = op_id
        )
end
function TOML.parse(project_spec::ProjectSpec)
    toml_dict = Dict()
    toml_dict["operations"] = map(op->TOML.parse(op),get_operations(project_spec))
    toml_dict["initial_conditions"] = map(pred->TOML.parse(pred), initial_conditions_vector(project_spec))
    toml_dict["final_conditions"] = map(pred->TOML.parse(pred), final_conditions_vector(project_spec))
    toml_dict
end
function read_project_spec(toml_dict::Dict)
    project_spec = ProjectSpec()
    ics = toml_dict["initial_conditions"]
    fcs = toml_dict["final_conditions"]
    ics = isa(ics,Dict) ? collect(values(ics)) : ics
    fcs = isa(fcs,Dict) ? collect(values(fcs)) : fcs
    for arr in ics
        o = read_object(arr)
        set_initial_condition!(project_spec,get_object_id(o),o)
    end
    for arr in fcs
        o = read_object(arr)
        set_final_condition!(project_spec,get_object_id(o),o)
    end
    for op_dict in toml_dict["operations"]
        op = read_operation(op_dict)
        add_operation!(project_spec, op)
    end
    return project_spec
end
function read_project_spec(io)
    read_project_spec(TOML.parsefile(io))
end

export
    SimpleProblemDef,
    SimplePCMAPFDef,
    read_problem_def,
    read_pcmapf_problem_def

@with_kw struct SimpleProblemDef
    project_spec::ProjectSpec       = ProjectSpec()
    r0::Vector{Int}                 = Int[]
    s0::Vector{Int}                 = Int[]
    sF::Vector{Int}                 = Int[]
    shapes::Vector{Tuple{Int,Int}}  = Vector{Tuple{Int,Int}}([(1,1) for o in s0])
end
SimpleProblemDef(project_spec,r0,s0,sF) = SimpleProblemDef(project_spec=project_spec,r0=r0,s0=s0,sF=sF)

"""
    SimplePCMAPFDef

Simple definition of a PC_MAPF problem (PC_TAPF) with assignments already made.
"""
@with_kw struct SimplePCMAPFDef
    pctapf_def::SimpleProblemDef
    assignments::Dict{Int,Vector{Int}}
end

function TOML.parse(def::SimpleProblemDef)
    toml_dict = TOML.parse(def.project_spec)
    toml_dict["r0"] = def.r0
    toml_dict["s0"] = def.s0
    toml_dict["sF"] = def.sF
    toml_dict["shapes"] = map(s->[s...], def.shapes)
    toml_dict
end
function TOML.parse(def::SimplePCMAPFDef)
    dict = TOML.parse(def.pctapf_def)
    dict["assignments"] = Dict(string(k)=>v for (k,v) in def.assignments)
    dict
end
function read_problem_def(toml_dict::Dict)
    SimpleProblemDef(
        read_project_spec(toml_dict),
        toml_dict["r0"],
        toml_dict["s0"],
        toml_dict["sF"],
        map(s->tuple(s...), get(toml_dict,"shapes",[[1,1] for o in toml_dict["s0"]]) )
    )
end
read_problem_def(io) = read_problem_def(TOML.parsefile(io))
function read_pcmapf_problem_def(toml_dict::Dict)
    pctapf_def = read_problem_def(toml_dict)
    assignments = Dict(parse(Int,k)=>v for (k,v) in toml_dict["assignments"])
    SimplePCMAPFDef(pctapf_def,assignments)
end
read_pcmapf_problem_def(io) = read_pcmapf_problem_def(TOML.parsefile(io))

# export
#     DeliveryTask,
#     DeliveryGraph

# """
#     DeliveryTask
# """
# struct DeliveryTask
#     o::ObjectID
#     s1::LocationID
#     s2::LocationID
# end

# """
#     DeliveryGraph{G}
# """
# struct DeliveryGraph{G}
#     tasks::Vector{DeliveryTask}
#     graph::G
# end

# export
#     construct_delivery_graph

# """
#     construct_delivery_graph(project_spec::ProjectSpec,M::Int)

# Assumes that initial station ids correspond to object ids.
# """
# function construct_delivery_graph(project_spec::ProjectSpec,M::Int)
#     delivery_graph = DeliveryGraph(Vector{DeliveryTask}(),MetaDiGraph(M))
#     object_id_map = Dict{ObjectID,Int}(
#         get_object_id(o)=>i for (i,o) in enumerate(initial_conditions_vector(project_spec)))
#     # for op in project_spec.operations
#     for op in get_operations(project_spec)
#         for (id,pred) in preconditions(op)
#             idx = object_id_map[id]
#             pre = get_initial_condition(project_spec,id)
#             post = get_final_condition(project_spec,id)
#             push!(delivery_graph.tasks,DeliveryTask(id,get_location_id(pre),get_location_id(post)))
#             for j in get_output_ids(op)
#                 idx2 = object_id_map[j]
#                 add_edge!(delivery_graph.graph, idx, idx2)
#             end
#         end
#     end
#     delivery_graph
# end

export
    PathSpec,
    generate_path_spec,
    get_t0,
    get_tF,
    set_t0!,
    set_tF!,
    get_slack,
    set_slack!,
    get_local_slack,
    set_local_slack!,
    get_min_duration,
    set_min_duration!

"""
    PathSpec

Encodes information about the path that must be planned for a particular
schedule node.

Fields:
* `node_type::Symbol = :EMPTY`
* `start_vtx::Int = -1`
* `final_vtx::Int = -1`
* `min_duration::Int = 0`
* `agent_id::Int = -1`
* `object_id::Int = -1`
* `plan_path::Bool = true` - flag indicating whether a path must be planned.
    For example, `Operation` nodes do not require any path planning.
* `tight::Bool = false` - if true, the path may not terminate prior to the
    beginning of successors. If `tight == true`, local slack == 0. For example,
    `GO` must not end before `COLLECT` can begin, because this would produce
    empty time between planning phases.
* `static::Bool = false` - if true, the robot must remain in place for this
    planning phase (e.g., COLLECT, DEPOSIT).
* `free::Bool = false` - if true, and if the node is a terminal node, the
    planning must go on until all non-free nodes are completed.
* `fixed::Bool = false` - if true, do not plan path because it is already fixed.
    Instead, retrieve the portion of the path directly from the pre-existing
    solution.
"""
@with_kw mutable struct PathSpec
    # temporal
    t0              ::Float64           = 0
    min_duration    ::Float64           = 0
    tF              ::Float64           = t0 + min_duration
    slack           ::Vector{Float64}   = Float64[]
    local_slack     ::Vector{Float64}   = Float64[]
    # instructions
    plan_path       ::Bool              = true
    tight           ::Bool              = false
    static          ::Bool              = false
    free            ::Bool              = false
    fixed           ::Bool              = false
end
get_t0(spec::PathSpec) = spec.t0
set_t0!(spec::PathSpec,val) = begin spec.t0 = val end
get_tF(spec::PathSpec) = spec.tF
set_tF!(spec::PathSpec,val) = begin spec.tF = val end
get_min_duration(spec::PathSpec) = spec.min_duration
set_min_duration!(spec::PathSpec,val) = begin spec.min_duration = val end
get_duration(spec::PathSpec) = get_tF(spec) - get_t0(spec)
get_slack(spec::PathSpec) = spec.slack
set_slack!(spec::PathSpec,val) = begin spec.slack = val end
get_local_slack(spec::PathSpec) = spec.local_slack
set_local_slack!(spec::PathSpec,val) = begin spec.local_slack = val end
Base.summary(s::PathSpec) = string("t0=",s.t0,", tF=",s.tF,", fixed=",s.fixed)

const path_spec_accessor_interface = [
    :get_t0,
    :get_tF,
    :get_slack,
    :get_local_slack,
    :get_min_duration,
    :get_duration,
    ]
const path_spec_mutator_interface = [
    :set_min_duration!,
    :set_t0!,
    :set_slack!,
    :set_local_slack!,
    :set_tF!,
    ]

export
    ScheduleNode,
    get_path_spec,
    set_path_spec!

"""
    ScheduleNode{I<:AbstractID,V<:AbstractPlanningPredicate}

The node type of the `OperatingSchedule` graph.
"""
mutable struct ScheduleNode{I<:AbstractID,V}
    id::I
    node::V
    spec::PathSpec
end
ScheduleNode(id,node) = ScheduleNode(id,node,PathSpec())
get_path_spec(node::ScheduleNode) = node.spec
Base.copy(n::ScheduleNode) = ScheduleNode(n.id,copy(n.node),deepcopy(n.spec))
function set_path_spec!(node::ScheduleNode,spec)
    node.spec = spec
end
for op in path_spec_accessor_interface
    @eval $op(node::ScheduleNode) = $op(get_path_spec(node))
end
for op in path_spec_mutator_interface
    @eval $op(node::ScheduleNode,val) = $op(get_path_spec(node),val)
end
for op in predicate_accessor_interface
    @eval $op(node::ScheduleNode) = $op(node.node)
end
for op in [:(GraphUtils.matches_template)]
    @eval $op(template::Type{T},node::ScheduleNode) where {T} = $op(template,node.node)
end
Base.summary(n::ScheduleNode) = string(string(n.node)," [",summary(n.spec),"]")

const schedule_node_accessor_interface = [
    path_spec_accessor_interface...,
    :get_path_spec]
const schedule_node_mutator_interface = [
    path_spec_mutator_interface...,
    :set_path_spec!]

matches_node_type(n::ScheduleNode,b::Type{B}) where {B} = matches_node_type(n.node,b)

export
    OperatingSchedule,
    get_object_ICs,
    get_object_FCs,
    get_robot_ICs,
    get_actions,
    get_operations,
    get_vtx_ids,
    get_node_from_id,
    get_node_from_vtx,
    get_completion_time,
    get_duration,
    get_vtx,
    get_vtx_id,
    replace_in_schedule!,
    add_to_schedule!,
    process_schedule

"""
    OperatingSchedule

Encodes discrete events/activities that need to take place, and the precedence
constraints between them. Each `ScheduleNode` has a corresponding vertex index
and an `AbstractID`. An edge from node1 to node2 indicates a precedence
constraint between them.
"""
@with_kw struct OperatingSchedule <: AbstractCustomNDiGraph{ScheduleNode,AbstractID}
    graph               ::DiGraph               = DiGraph()
    nodes               ::Vector{ScheduleNode}  = Vector{ScheduleNode}()
    vtx_map             ::Dict{AbstractID,Int}  = Dict{AbstractID,Int}()
    vtx_ids             ::Vector{AbstractID}    = Vector{AbstractID}() # maps vertex uid to actual graph node
    terminal_vtxs       ::Vector{Int}           = Vector{Int}() # list of "project heads"
    weights             ::Dict{Int,Float64}     = Dict{Int,Float64}() # weights corresponding to project heads
end
get_terminal_vtxs(sched::P) where {P<:OperatingSchedule}     = sched.terminal_vtxs
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

GraphUtils.get_vtx(sched::OperatingSchedule,node::ScheduleNode) = get_vtx(sched,node.id)
get_node_from_id(sched::OperatingSchedule,id)                   = get_node(sched,id).node
get_node_from_vtx(sched::OperatingSchedule,v)                   = get_node(sched,v).node

get_object_ICs(sched::P) where {P<:OperatingSchedule}  = Dict(k=>v.node for (k,v) in get_nodes_of_type(sched,ObjectID))
get_robot_ICs(sched::P) where {P<:OperatingSchedule}   = Dict(k=>v.node for (k,v) in get_nodes_of_type(sched,BotID))
get_actions(sched::P) where {P<:OperatingSchedule}     = Dict(k=>v.node for (k,v) in get_nodes_of_type(sched,ActionID))
get_operations(sched::P) where {P<:OperatingSchedule}  = Dict(k=>v.node for (k,v) in get_nodes_of_type(sched,OperationID))

for op in schedule_node_accessor_interface
    @eval $op(sched::OperatingSchedule,v) = $op(get_node(sched,v))
    @eval $op(sched::OperatingSchedule) = map(v->$op(get_node(sched,v)), vertices(sched))
end
for op in schedule_node_mutator_interface
    @eval $op(sched::OperatingSchedule,v,val) = $op(get_node(sched,v),val)
    @eval $op(sched::OperatingSchedule,val) = begin
        for v in vertices(sched)
            $op(get_node(sched,v),val)
        end
    end
end

# Defining default node parameters
is_tight(p) = false
is_tight(::BOT_GO) = true
is_free(p) = false
is_free(p::BOT_AT) = true
is_free(p::BOT_GO) = !CRCBS.is_valid(get_destination_location_id(p))
is_static(p) = false
is_static(p::Union{BOT_COLLECT,BOT_DEPOSIT}) = true
for op in [:is_free,:is_static,:is_tight]
    @eval $op(p::TEAM_ACTION) = $op(sub_nodes(p)[1])
end
needs_path(p) = false
needs_path(p::Union{BOT_AT,AbstractRobotAction}) = true
duration_lower_bound(args...) = 0
duration_lower_bound(op::Operation,spec) = duration(op)
duration_lower_bound(p::BOT_COLLECT,spec) = get(spec.Δt_collect,get_object_id(p),0)
duration_lower_bound(p::BOT_DEPOSIT,spec) = get(spec.Δt_deposit,get_object_id(p),0)
function duration_lower_bound(p::Union{BOT_AT,AbstractRobotAction},spec)
    x1 = get_initial_location_id(p)
    x2 = get_destination_location_id(p)
    return get_distance(spec,x1,x2)
end
function duration_lower_bound(p::TEAM_ACTION,spec)
    x1 = get_initial_location_id(p.instructions[1])
    x2 = get_destination_location_id(p.instructions[1])
    return get_distance(spec,x1,x2,team_configuration(p))
end

"""
    generate_path_spec(spec,node)

Generates a `PathSpec` struct that encodes information about the path to be
planned for `node`.

Arguments:
* spec::ProblemSpec
* node::T <: AbstractPlanningPredicate
"""
function generate_path_spec(spec::ProblemSpec,a)
    PathSpec(
        min_duration=duration_lower_bound(a,spec),
        tight=is_tight(a),
        static=is_static(a),
        free=is_free(a),
        plan_path=needs_path(a)
        )
end
generate_path_spec(sched::OperatingSchedule,spec::ProblemSpec,pred) = generate_path_spec(spec,pred)
generate_path_spec(sched::OperatingSchedule,pred) = generate_path_spec(sched,ProblemSpec(),pred)

"""
    replace_in_schedule!(schedule::OperatingSchedule,path_spec::T,pred,id::ID) where {T<:PathSpec,ID<:AbstractID}

Replace the `ScheduleNode` associated with `id` with the new node `pred`, and
the accompanying `PathSpec` `path_spec`.
"""
replace_in_schedule!(sched::OperatingSchedule,node::ScheduleNode,id::AbstractID=node.id) = replace_node!(sched,node,id)
function replace_in_schedule!(sched::P,path_spec::T,pred,id::ID) where {P<:OperatingSchedule,T<:PathSpec,ID<:AbstractID}
    replace_in_schedule!(sched,ScheduleNode(id,pred,path_spec))
end
function replace_in_schedule!(sched::P,spec,pred,id::ID) where {P<:OperatingSchedule,ID<:AbstractID}
    replace_in_schedule!(sched,generate_path_spec(sched,spec,pred),pred,id)
end
function replace_in_schedule!(sched::P,pred,id::ID) where {P<:OperatingSchedule,ID<:AbstractID}
    replace_in_schedule!(sched,ProblemSpec(),pred,id)
end

GraphUtils.add_node!(sched::OperatingSchedule,node::ScheduleNode) = add_node!(sched,node,node.id)

function GraphUtils.make_node(g::OperatingSchedule,pred::AbstractPlanningPredicate,args...)
   make_node(g,PathSpec(),pred,args...)
end
function GraphUtils.make_node(g::OperatingSchedule,spec::ProblemSpec,pred,args...)
   make_node(g,generate_path_spec(g,spec,pred),pred,args...)
end
function GraphUtils.make_node(g::OperatingSchedule,spec::PathSpec,pred::BOT_AT,id=get_robot_id(pred))
    ScheduleNode(id,pred,spec)
end
function GraphUtils.make_node(g::OperatingSchedule,spec::PathSpec,pred::OBJECT_AT,id=get_object_id(pred))
    ScheduleNode(id,pred,spec)
end
# function GraphUtils.make_node(g::OperatingSchedule,spec::PathSpec,pred::Union{OBJECT_AT,BOT_AT,Operation},id=node_id(pred))
#     ScheduleNode(id,pred,spec)
# end
# function GraphUtils.make_node(g::OperatingSchedule,spec::PathSpec,pred::A,id=node_id(pred)) where {A<:AbstractRobotAction}
function GraphUtils.make_node(g::OperatingSchedule,spec::PathSpec,pred::AbstractRobotAction,id=get_unique_action_id())
    ScheduleNode(id,pred,spec)
end
# function GraphUtils.make_node(g::OperatingSchedule,spec::PathSpec,pred::A,id=get_unique_id(TemplatedID{A})) where {A<:BOT_GO}
#     ScheduleNode(id,pred,spec)
# end
function GraphUtils.make_node(g::OperatingSchedule,spec::PathSpec,pred::Operation,id=get_operation_id(pred))
    ScheduleNode(id,pred,spec)
end

export
    get_leaf_operation_vtxs,
    set_leaf_operation_vtxs!

function get_leaf_operation_vtxs(sched::OperatingSchedule)
    terminal_vtxs = Int[]
    for v in get_all_terminal_nodes(sched)
        if matches_node_type(get_node(sched,v),Operation)
            push!(terminal_vtxs,v)
        end
    end
    return terminal_vtxs
end
function set_leaf_operation_vtxs!(sched::OperatingSchedule)
    empty!(get_terminal_vtxs(sched))
    empty!(get_root_node_weights(sched))
    for vtx in get_leaf_operation_vtxs(sched)
        push!(get_terminal_vtxs(sched),vtx)
        get_root_node_weights(sched)[vtx] = 1.0
    end
    sched
end

export backtrack_node

"""
    `backtrack_node(sched::OperatingSchedule,v::Int)`

Find the closest ancestor of `v` with overlapping `RobotID`s.
"""
function backtrack_node(sched::OperatingSchedule,v::Int)
    robot_ids = get_valid_robot_ids(sched,get_vtx_id(sched,v),v)
    vtxs = Int[]
    if isempty(robot_ids)
        return vtxs
    end
    for vp in inneighbors(sched,v)
        if !isempty(intersect(get_valid_robot_ids(sched,vp),robot_ids))
            push!(vtxs,vp)
        end
    end
    return vtxs
end

export
    validate,
    sanity_check

function validate(path::Path, msg::String)
    @assert( !any(convert_to_vertex_lists(path) .== -1), msg )
    return true
end
function validate(path::Path, v::Int)
    validate(path, string("invalid path with -1 vtxs: v = ",v,", path = ",convert_to_vertex_lists(path)))
end
function validate(path::Path, v::Int, cbs_env)
    validate(path, string("v = ",v,", path = ",convert_to_vertex_lists(path),", goal: ",cbs_env.goal))
end
function sanity_check(sched::OperatingSchedule,append_string="")
    try
        @assert !is_cyclic(sched) "is_cyclic(sched)"
        for v in vertices(sched)
            node = get_node_from_vtx(sched, v)
            id = get_vtx_id(sched,v)
            if matches_node_type(node,COLLECT)
                @assert(get_location_id(node) != -1, string("get_location_id(node) != -1 for node id ", id))
            end
            if matches_node_type(node,Operation)
                # input_ids = Set(map(o->get_id(get_object_id(o)),collect(preconditions(node)))))
                input_ids = Set(collect(keys(preconditions(node))))
                for v2 in inneighbors(sched,v)
                    node2 = get_node_from_vtx(sched, v2)
                    @assert(get_object_id(node2) in input_ids, string(string(node2), " should not be an inneighbor of ",string(node), " whose inputs should be ",input_ids))
                end
                @assert(
                    indegree(sched,v) == length(preconditions(node)),
                    string("Operation ",string(node),
                        " needs edges from objects ",collect(input_ids),
                        " but only has edges from objects ",map(v2->get_id(get_object_id(get_node_from_vtx(sched, v2))),inneighbors(sched,v))
                    )
                    )
            end
        end
    catch e
        if typeof(e) <: AssertionError
            bt = catch_backtrace()
            showerror(stdout,e,bt)
            print(string(e.msg, append_string))
        else
            rethrow(e)
        end
        return false
    end
    return true
end
function validate(spec::ProjectSpec,op)
    for (id,pred) in preconditions(op)
        @assert get_final_condition(spec,id) == pred
    end
    for (id,pred) in postconditions(op)
        @assert get_initial_condition(spec,id) == pred
    end
    return true
end
function validate(sched::OperatingSchedule)
    try
        @assert !is_cyclic(sched) "is_cyclic(G)"
        for e in edges(sched)
            node1 = get_node_from_id(sched, get_vtx_id(sched, e.src))
            node2 = get_node_from_id(sched, get_vtx_id(sched, e.dst))
            @assert(validate_edge(node1,node2), string(" INVALID EDGE: ", string(node1), " --> ",string(node2)))
        end
        for v in vertices(sched)
            id = get_vtx_id(sched, v)
            node = get_node_from_id(sched, id)
            if matches_node_type(node,COLLECT)
                @assert(get_location_id(node) != -1, string("get_location_id(node) != -1 for node id ", id))
            end
            @assert( outdegree(sched,v) >= sum([0, values(required_successors(node))...]) , string("outdegree = ",outdegree(sched,v), " for node = ", string(node)))
            @assert( indegree(sched,v) >= sum([0, values(required_predecessors(node))...]), string("indegree = ",indegree(sched,v), " for node = ", string(node)))
            if matches_node_type(node, AbstractSingleRobotAction)
                for v2 in outneighbors(sched,v)
                    node2 = get_node_from_vtx(sched, v2)
                    if matches_node_type(node2, AbstractSingleRobotAction)
                        if length(intersect(resources_reserved(node),resources_reserved(node2))) == 0 # job shop constraint
                            @assert( get_robot_id(node) == get_robot_id(node2), string("robot IDs do not match: ",string(node), " => ", string(node2)))
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
            showerror(stdout,e,bt)
            print(e.msg)
        else
            rethrow(e)
        end
        return false
    end
    return true
end
function log_schedule_edges(sched,v=-1;
        show_upstream = true,
        show_downstream = true,
        show_all = true,
    )
    if has_vertex(sched,v)
        if show_upstream
            @info "upstream from $(summary(get_node(sched,v))):"
            for e in edges(bfs_tree(sched,v;dir=:in))
                @info "$(summary(get_node(sched,e.dst))) => $(summary(get_node(sched,e.src)))"
            end
        end
        if show_downstream
            @info "downstream from $(summary(get_node(sched,v))):"
            for e in edges(bfs_tree(sched,v;dir=:out))
                @info "$(summary(get_node(sched,e.src))) => $(summary(get_node(sched,e.dst)))"
            end
        end
    end
    if show_all
        for e in edges(sched)
            @info "$(summary(get_node(sched,e.src))) => $(summary(get_node(sched,e.dst))),"
        end
    end
end
function validate(node::ScheduleNode,paths::Vector{Vector{Int}};
        quiet::Bool=false,
    )
    if matches_template(TEAM_ACTION,node)
        for n in sub_nodes(node)
            sub_node = ScheduleNode(node.id,n,node.spec)
            if !validate(sub_node,paths)
                return false
            end
        end
    else
        agent_id = get_id(get_default_robot_id(node))
        if agent_id != -1
            path = paths[agent_id]
            start_vtx = get_id(get_initial_location_id(node))
            final_vtx = get_id(get_destination_location_id(node))
            try
                t0 = Int(round(get_t0(node)))
                tF = Int(round(get_tF(node)))
                @assert abs(t0-get_t0(node)) <= 0.01
                @assert abs(tF-get_tF(node)) <= 0.01
                @assert(length(path) > t0, string("length(path) == $(length(path)), should be greater than get_t0(node) == $(get_t0(node)) in node ",string(node)))
                @assert(length(path) > tF, string("length(path) == $(length(path)), should be greater than get_tF(node) == $(get_tF(node)) in node ",string(node)))
                if start_vtx != -1
                    if length(path) > t0
                        @assert(path[t0 + 1] == start_vtx, string("node: ",string(node.node), ", start_vtx: ",start_vtx, ", t0+1: ",t0+1,", path[get_t0(node) + 1] = ",path[t0 + 1],", path: ", path))
                    end
                end
                if final_vtx != -1
                    if length(path) > tF
                        @assert(path[tF + 1] == final_vtx, string("node: ",string(node.node), ", final vtx: ",final_vtx, ", tF+1: ",tF+1,", path[get_tF(node) + 1] = ",path[tF + 1],", path: ", path))
                    end
                end
            catch e
                if typeof(e) <: AssertionError
                    if !quiet
                        println(e.msg)
                    end
                else
                    rethrow(e)
                end
                return false
            end
        end
    end
    return true
end
function validate(sched::OperatingSchedule,paths::Vector{Vector{Int}};
        kwargs...
    )
    for v in vertices(sched)
        node = get_node(sched,v)
        if !validate(node,paths;kwargs...)
            return false
        end
    end
    return true
end

export
    add_single_robot_delivery_task!

function add_headless_delivery_task!(
        sched::OperatingSchedule,
        problem_spec::ProblemSpec,
        object_id::ObjectID,
        op_id::OperationID,
        robot_type::Type{R}=DeliveryBot,
        pickup_station_id::LocationID=get_location_id(get_node_from_id(sched,object_id)),
        dropoff_station_id::LocationID=get_dropoff(get_node_from_id(sched,op_id),object_id),
        ;
        t0::Int=0,
    ) where {R<:AbstractRobotType}

    robot_id = BotID{R}(-1)
    prev_id = object_id
    for p in [
        BOT_COLLECT(robot_id, object_id, pickup_station_id),
        BOT_CARRY(robot_id, object_id, pickup_station_id, dropoff_station_id),
        BOT_DEPOSIT(robot_id, object_id, dropoff_station_id),
        BOT_GO(robot_id, dropoff_station_id,LocationID(-1)),
        ]
        node = add_node!(sched, make_node(sched, problem_spec, p))
        set_t0!(sched,node.id,t0)
        add_edge!(sched, prev_id, node.id)
        if matches_node_type(node,BOT_DEPOSIT)
            add_edge!(sched, node.id, op_id)
        end
        prev_id = node.id
    end

    return
end
function add_headless_team_delivery_task!(
        sched::OperatingSchedule,
        problem_spec::ProblemSpec,
        object_id::ObjectID,
        op_id::OperationID,
        robot_type::Type{R}=DeliveryBot,
        pickup_station_ids::Vector{LocationID}=get_location_ids(get_node_from_id(sched,object_id)),
        dropoff_station_ids::Vector{LocationID}=get_dropoffs(get_node_from_id(sched,op_id),object_id)
        ) where {R<:AbstractRobotType}

    robot_id = BotID{R}(-1)
    @assert length(pickup_station_ids) == length(dropoff_station_ids) "$(summary(object_id)): pickup_station_ids = $(pickup_station_ids), but dropoff_station_ids = $(dropoff_station_ids)"
    n = length(pickup_station_ids)

    object_node = get_node_from_id(sched,object_id)
    shape = object_node.shape
    prev_id = object_id
    for team_action in [
            TEAM_COLLECT(
            instructions = map(x->BOT_COLLECT(robot_id, object_id, x), pickup_station_ids),
            shape=shape
            ),
            TEAM_CARRY(
            instructions = [BOT_CARRY(robot_id, object_id, x1, x2) for (x1,x2) in zip(pickup_station_ids,dropoff_station_ids)],
            shape = shape
            ),
            TEAM_DEPOSIT(
            instructions = map(x->BOT_DEPOSIT(robot_id, object_id, x), dropoff_station_ids),
            shape=shape
            )
        ]
        node = add_node!(sched, make_node(sched,problem_spec,team_action))
        add_edge!(sched,prev_id,node.id)
        prev_id = node.id
        if matches_node_type(node,TEAM_COLLECT)
            for x in pickup_station_ids
                subnode = add_node!(sched,make_node(sched,problem_spec,BOT_GO(robot_id,x,x)))
                add_edge!(sched,subnode.id,node.id)
            end
        elseif matches_node_type(node,TEAM_DEPOSIT)
            add_edge!(sched,node.id,op_id)
            for x in dropoff_station_ids
                subnode = add_node!(sched,make_node(sched,problem_spec,BOT_GO(robot_id,x,-1)))
                add_edge!(sched, node.id, subnode.id)
            end
        end
    end
    return
end
function add_single_robot_delivery_task!(
        sched::OperatingSchedule,
        spec::ProblemSpec,
        robot_id::BotID{R},
        object_id::ObjectID,
        op_id::OperationID,
        go_id::ActionID,
        x1::LocationID=get_location_id(get_node_from_id(sched,object_id)),
        x2::LocationID=get_dropoff(get_node_from_id(sched,op_id),object_id),
        args...
    ) where {R<:AbstractRobotType}
    add_headless_delivery_task!(sched,spec,object_id,op_id,R,x1,x2)
    collect_id = get_vtx_id(sched,outneighbors(sched,get_vtx(sched,object_id))[1])
    add_edge!(sched,go_id,collect_id)
    propagate_valid_ids!(sched,spec)
end

export
    construct_partial_project_schedule

function add_new_robot_to_schedule!(sched,pred::BOT_AT,problem_spec=ProblemSpec())
    n1 = add_node!(sched, make_node(sched,problem_spec,pred)) #, pred, robot_id)
    action = BOT_GO(r=get_robot_id(pred),x1=get_location_id(pred))
    n2 = add_node!(sched, make_node(sched,problem_spec,action)) #, action, action_id)
    add_edge!(sched, n1.id, n2.id)
    return sched
end

"""
    construct_partial_project_schedule

Constructs a partial project graph
"""
function construct_partial_project_schedule(spec::ProjectSpec,problem_spec::ProblemSpec,robot_ICs=Vector{BOT_AT}())
    object_ICs = initial_conditions_vector(spec)
    object_FCs = final_conditions_vector(spec)
    operations = get_operations(spec)
    terminal_ops = sort(collect(get_terminal_operation_ids(spec)))

    # Construct Partial Project Schedule
    sched = OperatingSchedule()
    for op in operations
        add_node!(sched, make_node(sched,problem_spec,op))
    end
    for pred in object_ICs
        add_node!(sched, make_node(sched,problem_spec,pred))
    end
    for pred in robot_ICs
        add_new_robot_to_schedule!(sched,pred,problem_spec)
    end
    # add root nodes
    for op_id in terminal_ops
        v = get_vtx(sched, op_id)
        push!(sched.terminal_vtxs, v)
        sched.weights[v] = 1.0
    end
    # Fill in gaps in project schedule (except for GO assignments)
    for op in operations
        op_id = get_operation_id(op)
        for object_id in get_input_ids(op)
            # add action sequence
            object_ic           = get_node_from_id(sched, object_id)
            if length(get_location_ids(object_ic)) > 1 # COLLABORATIVE TRANSPORT
                add_headless_team_delivery_task!(sched,problem_spec,
                    ObjectID(object_id),op_id)
            else # SINGLE AGENT TRANSPORT
                add_headless_delivery_task!(sched,problem_spec,
                    ObjectID(object_id),op_id)
            end
        end
        for object_id in get_output_ids(op)
            add_edge!(sched, op_id, object_id)
        end
    end
    # NOTE: A hack to get speed up on SparseAdjacencyMILP. Not sure if it will work
    # for (id, pred) in get_object_ICs(sched)
    #     v = get_vtx(sched, ObjectID(id))
    #     if indegree(get_graph(sched),v) == 0
    #         op_id = get_unique_operation_id()
    #         op = Operation(id=op_id)
    #         set_postcondition!(op,pred)
    #         add_node!(sched,make_node(sched,problem_spec,op))
    #         add_edge!(sched,op_id,ObjectID(id))
    #     end
    # end
    set_leaf_operation_vtxs!(sched)
    process_schedule!(sched)
    sched
end
function construct_partial_project_schedule(
        robot_ICs::Vector{R},
        prob_spec::ProblemSpec=ProblemSpec(),
        ) where {R<:BOT_AT}
    construct_partial_project_schedule(ProjectSpec(),prob_spec,robot_ICs)
end

"""
    process_schedule(schedule::P) where {P<:OperatingSchedule}

Compute the optimistic start and end times, along with the slack associated
with each vertex in the `schedule`. Slack for each vertex is represented as
a vector in order to handle multi-headed projects.

Args:
* schedule::OperatingSchedule
* [OPTIONAL] t0::Vector{Int}: default = zeros(Int,nv(schedule))
* [OPTIONAL] tF::Vector{Int}: default = zeros(Int,nv(schedule))
"""
function process_schedule(sched::P,t0=zeros(nv(sched)),
        tF=zeros(nv(sched))
    ) where {P<:OperatingSchedule}

    G = get_graph(sched)
    traversal = topological_sort_by_dfs(G)
    n_roots = max(length(sched.terminal_vtxs),1)
    slack = map(i->Inf*ones(n_roots), vertices(G))
    local_slack = map(i->Inf*ones(n_roots), vertices(G))
    # max_deadlines = map(i->typemax(Int), vertices(G))
    # True terminal nodes
    for (i,v) in enumerate(sched.terminal_vtxs)
        slack[v] = Inf*ones(n_roots)
        slack[v][i] = 0 # only slack for corresponding head is set to 0
    end
    ########## Compute Lower Bounds Via Forward Dynamic Programming pass
    for v in traversal
        path_spec = get_path_spec(sched,v)
        Δt_min = path_spec.min_duration
        for v2 in inneighbors(G,v)
            t0[v] = max(t0[v], tF[v2])
        end
        tF[v] = max(tF[v], t0[v] + Δt_min)
    end
    ########### Compute Slack Via Backward Dynamic Programming pass
    for v in reverse(traversal)
        for v2 in outneighbors(G,v)
            local_slack[v] = min.(local_slack[v], (t0[v2] - tF[v]) )
            slack[v] = min.(slack[v], slack[v2] .+ (t0[v2] - tF[v]) )
        end
    end
    t0,tF,slack,local_slack
end
"""
    update_schedule_times!(sched::OperatingSchedule)

Compute start and end times for all nodes based on the end times of their
inneighbors and their own durations.
"""
function update_schedule_times!(sched::OperatingSchedule)
    G = get_graph(sched)
    for v in topological_sort_by_dfs(G)
        t0 = get_t0(sched,v)
        for v2 in inneighbors(G,v)
            t0 = max(t0,get_tF(sched,v2))
        end
        set_t0!(sched,v,t0)
        set_tF!(sched,v,max(get_tF(sched,v),t0+get_min_duration(sched,v)))
    end
    return sched
end
"""
    update_schedule_times!(sched::OperatingSchedule,frontier::Set{Int},local_only=true)

Compute start and end times for all nodes in `frontier` and their descendants.
If `local_only == true`, descendants of nodes with unchanged final time will not
be updated.
"""
function update_schedule_times!(sched::OperatingSchedule,frontier::Set{Int};
        local_only::Bool=true,
        )
    iter = GraphUtils.SortedBFSIterator(graph=sched,frontier=collect(frontier),replace=true)
    while !isempty(iter)
        v = pop!(iter)
        t0 = get_t0(sched,v)
        for v2 in inneighbors(sched,v)
            t0 = max(t0,get_tF(sched,v2))
        end
        set_t0!(sched,v,t0)
        tF = t0+get_min_duration(sched,v)
        if tF >= get_tF(sched,v) || !local_only
            set_tF!(sched,v,tF)
            GraphUtils.update_iterator!(iter,v)
        end
    end
    return sched
end
function update_slack!(sched::OperatingSchedule)
    G = get_graph(sched)
    n_roots = max(length(sched.terminal_vtxs),1)
    slack = map(i->Inf*ones(n_roots), vertices(G))
    local_slack = map(i->Inf*ones(n_roots), vertices(G))
    for (i,v) in enumerate(sched.terminal_vtxs)
        slack[v][i] = 0 # only slack for corresponding head is set to 0
    end
    for v in reverse(topological_sort_by_dfs(G))
        for v2 in outneighbors(G,v)
            local_slack[v] = min.(local_slack[v], (get_t0(sched,v2) - get_tF(sched,v)))
            slack[v] = min.(slack[v], slack[v2] .+ (get_t0(sched,v2) - get_tF(sched,v)))
        end
    end
    for v in vertices(sched)
        set_slack!(sched,v,slack[v])
        set_local_slack!(sched,v,local_slack[v])
    end
    return sched
end
function process_schedule!(sched)
    update_schedule_times!(sched)
    update_slack!(sched)
end
function reset_schedule_times!(sched::OperatingSchedule,ref)
    for v in vertices(sched)
        set_t0!(sched,v,get_t0(ref,get_vtx_id(sched,v)))
        set_tF!(sched,v,get_tF(ref,get_vtx_id(sched,v)))
    end
    process_schedule!(sched)
end
makespan(sched::OperatingSchedule) = maximum(get_tF(sched))

export
    get_collect_node,
    get_deposit_node

function forward_search(sched,v,f)
    id = v
    node = get_node(sched,v)
    while !f(node)
        id = get_vtx_id(sched, outneighbors(sched,id)[1])
        node = get_node(sched, id)
    end
    return id, node
end
get_collect_node(sched::OperatingSchedule,id::ObjectID) = forward_search(sched,id,n->matches_node_type(n,BOT_COLLECT))
get_deposit_node(sched::OperatingSchedule,id::ObjectID) = forward_search(sched,id,n->matches_node_type(n,BOT_DEPOSIT))

export get_assignment_dict

"""
    get_assignment_dict(assignment_matrix,N,M)

Returns dictionary that maps each robot id to a sequence of tasks.
"""
function get_assignment_dict(assignment_matrix,N,M)
    assignment_dict = Dict{Int,Vector{Int}}()
    for i in 1:N
        vec = get!(assignment_dict,i,Int[])
        k = i
        j = 1
        while j <= M
            if assignment_matrix[k,j] == 1
                push!(vec,j)
                k = j + N
                j = 0
            end
            j += 1
        end
    end
    assignment_dict
end

"""
    get_assignment_dict(sched::OperatingSchedule)

Return a dictionary mapping robot id to a sequence of object_ids.
"""
function get_assignment_dict(sched::OperatingSchedule)
    dict = Dict{Int,Vector{Int}}()
    for n in node_iterator(sched,topological_sort_by_dfs(sched))
        if matches_template(BOT_COLLECT,n)
            node = n.node
            r = get_id(get_robot_id(node))
            o = get_id(get_object_id(node))
            push!(get!(dict,r,Int[]),o)
        end
    end
    return dict
end

"""
    apply_assignment_dict!(sched::OperatingSchedule,assignment_dict)

Make the assignments encoded in assignment_dict.
"""
function apply_assignment_dict!(sched::OperatingSchedule,assignment_dict,prob_spec)
    tips = robot_tip_map(sched)
    collect_nodes = Dict(get_object_id(n.node)=>n for n in get_nodes(sched) if matches_template(BOT_COLLECT,n))
    for (robot_idx,itinerary) in assignment_dict
        r = RobotID(robot_idx)
        for object_idx in itinerary
            tip = get_node(sched,tips[r])
            o = ObjectID(object_idx)
            c = collect_nodes[o]
            add_edge!(sched,tip,c)
            propagate_valid_ids!(sched,prob_spec)
            # update tips
            tips = robot_tip_map(sched)
        end
    end
    process_schedule!(sched)
    sched
end
