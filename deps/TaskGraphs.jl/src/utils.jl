export to_string_dict

to_string_dict(dict::Dict) = Dict{String,Any}(string(k)=>v for (k,v) in dict)
TOML.print(io::IO,dict::Dict{Symbol,V}) where {V} = TOML.print(io,to_string_dict(dict))

export to_symbol_dict
to_symbol_dict(el) = el
to_symbol_dict(dict::Dict) = Dict{Symbol,Any}(Symbol(k)=>to_symbol_dict(v) for (k,v) in dict)
to_symbol_dict(v::Vector) = map(to_symbol_dict, v)

read_assignment(toml_dict::Dict) = toml_dict["assignment"]
read_assignment(io) = read_assignment(TOML.parsefile(io))

function TOML.parse(mat::SparseMatrixCSC{T,Int64}) where {T}
    I,J,_ = findnz(mat)
    return Dict("edge_list" => map(idx->[idx...], zip(I,J)),
        "size_i" => size(mat,1),
        "size_j" => size(mat,2)
        )
end
function read_sparse_matrix(toml_dict::Dict)
    edge_list   = toml_dict["edge_list"]
    size_i      = toml_dict["size_i"]
    size_j      = toml_dict["size_j"]
    mat = spzeros(Int,size_i,size_j)
    for e in edge_list
        mat[e[1],e[2]] = 1
    end
    mat
end
read_sparse_matrix(io) = read_sparse_matrix(TOML.parsefile(io))

export read_abstract_id

parse_abstract_id(id::A,dict=Dict{String,Any}()) where {A<:AbstractID} = merge!(dict,Dict("type"=>string(A),"id"=>get_id(id)))
TOML.parse(id::A) where {A<:AbstractID} = parse_abstract_id(id)
function read_abstract_id(dict)
    type_string = dict["type"]
    for t in [ActionID(),ObjectID(),BotID{DeliveryBot}(),BotID{CleanUpBot}()]
        if type_string == string(typeof(t))
            return typeof(t)(dict["id"])
        end
    end
    return nothing
end
function parse_predicate(a::A,dict=Dict{String,Any}()) where {A<:AbstractPlanningPredicate}
    dict["type"] = string(A)
    dict["id"] = TOML.parse()
    return dict
end
# function TOML.parse(sched::OperatingSchedule)
#
# end

export
    remap_object_id,
    remap_object_ids!

# utilities for remapping object ids
remap_object_id(x,args...) = x
remap_object_id(id::ObjectID,max_obj_id)    = ObjectID(get_id(id) + max_obj_id)
# remap_object_id(spec::PathSpec,max_obj_id)  = PathSpec(spec,object_id=spec.object_id + max_obj_id)
remap_object_id(node::OBJECT_AT,args...)    = OBJECT_AT(remap_object_id(get_object_id(node),args...), get_initial_location_id(node))
remap_object_id(node::A,args...) where {A<:Union{COLLECT,DEPOSIT}} = A(o=remap_object_id(get_object_id(node),args...),r=get_robot_id(node),x=get_location_id(node))
remap_object_id(node::A,args...) where {A<:CARRY} = A(o=remap_object_id(get_object_id(node),args...),r=node.r,x1=node.x1,x2=node.x2)
remap_object_id(node::A,args...) where {A<:TEAM_ACTION} = A(instructions=map(i->remap_object_id(i,args...),node.instructions),shape=node.shape)
remap_object_id(node::ScheduleNode,args...) = ScheduleNode(
    remap_object_id(node.id,args...),
    remap_object_id(node.node,args...),
    remap_object_id(node.spec,args...)
    )
function remap_object_ids!(vec::Vector,args...)
    map!(i->remap_object_id(i,args...),vec,vec)
end
function remap_object_ids!(dict::Dict,args...)
    for k in collect(keys(dict))
        val = remap_object_id(dict[k],args...)
        delete!(dict, k)
        dict[remap_object_id(k,args...)] = val
    end
    dict
end
function remap_object_ids!(node::Operation,args...)
    remap_object_ids!(preconditions(node),args...)
    remap_object_ids!(postconditions(node),args...)
    return node
end
remap_object_id(node::Operation,args...)    = remap_object_ids!(deepcopy(node),args...)
function remap_object_ids!(sched::OperatingSchedule,args...)
    remap_object_ids!(get_vtx_ids(sched),args...)
    remap_object_ids!(get_nodes(sched),args...)
    remap_object_ids!(get_vtx_map(sched),args...)
    @assert sanity_check(sched," after remap_object_ids!()")
    sched
end
function remap_object_ids!(spec::ProjectSpec,args...)
    remap_object_ids!(get_vtx_ids(spec),args...)
    remap_object_ids!(get_nodes(spec),args...)
    remap_object_ids!(get_vtx_map(spec),args...)
    remap_object_ids!(get_initial_conditions(spec),args...)
    remap_object_ids!(get_final_conditions(spec),args...)
    # @assert sanity_check(sched," after remap_object_ids!()")
    spec
end
for T in [:OperatingSchedule,:ProjectSpec]
    @eval begin
        function remap_object_ids!(new_schedule::$T,old_schedule::$T)
            max_obj_id = 0
            for id in get_vtx_ids(old_schedule)
                if typeof(id) <: ObjectID
                    max_obj_id = max(get_id(id),max_obj_id)
                end
            end
            remap_object_ids!(new_schedule,max_obj_id)
        end
    end
end

export get_valid_robot_ids

"""
    get_valid_robot_ids(sched::OperatingSchedule,n_id::AbstractID)

Returns vector of all robot ids associated with the schedule node referenced by
n_id.
"""
function get_valid_robot_ids(sched::OperatingSchedule,n_id::A,v=get_vtx(sched,n_id)) where {A<:Union{ActionID,BotID}}
    ids = Vector{BotID}()
    node = get_node(sched,n_id).node
    if matches_template(TEAM_ACTION,node)
        for n in sub_nodes(node)
            r = get_robot_id(n)
            if get_id(r) != -1
                push!(ids,r)
            end
        end
    else
        push!(ids,get_default_robot_id(node))
    end
    return filter(CRCBS.is_valid,ids)
end
function get_valid_robot_ids(sched::OperatingSchedule,n_id::A,v=get_vtx(sched,n_id)) where {A<:Union{ObjectID,OperationID}}
    return Vector{BotID}()
end
get_valid_robot_ids(s::OperatingSchedule,v::Int) = get_valid_robot_ids(s,get_vtx_id(s,v),v)
get_valid_robot_ids(node) = BotID[]
get_valid_robot_ids(node::Union{BOT_AT,AbstractRobotAction}) = [get_robot_id(node)]
get_valid_robot_ids(node::TEAM_ACTION) = filter(CRCBS.is_valid,map(n->get_robot_id(n),sub_nodes(node)))

export robot_tip_map

"""
    robot_tip_map(sched::OperatingSchedule)

Returns a `Dict{RobotID,AbstractID}` mapping `RobotID` to the terminal node of
the `sched` corresponding to the robot's last assigned task.
"""
function robot_tip_map(sched::OperatingSchedule,vtxs=get_all_terminal_nodes(sched))
    robot_tips = Dict{BotID,AbstractID}()
    for v in vtxs
        n_id = get_vtx_id(sched,v)
        for robot_id in get_valid_robot_ids(sched,n_id,v)
            if get_id(robot_id) != -1
                if haskey(robot_tips,robot_id)
                    current_tip = get_node(sched,robot_tips[robot_id])
                    new_tip = get_node(sched,n_id)
                    @error "Just found tip $(string(new_tip.node)), but robot tip map already has $(robot_id) => $(string(current_tip.node))"
                    GraphUtils.log_graph_edges(sched,current_tip,show_all=false)
                    GraphUtils.log_graph_edges(sched,new_tip,show_all=false)
                    # @assert validate(sched)
                    @assert !haskey(robot_tips,robot_id)
                end
                robot_tips[robot_id] = n_id
            end
        end
    end
    # @assert length(robot_tips) == length(get_robot_ICs(sched)) "length(robot_tips) == $(length(robot_tips)), but should be $(length(get_robot_ICs(sched)))"
    robot_tips
end

"""
    extract_robot_itinerary(sched::OperatingSchedule,id::BotID)

Return the sequence of tasks assigned to `id`
"""
function extract_robot_itinerary(sched::OperatingSchedule,id::BotID)
    itinerary = Vector{ScheduleNode}()
    if !has_vertex(sched,id)
        return itinerary 
    end
    n = get_node(sched,id) # Robot start node
    push!(itinerary,n)
    while outdegree(sched,n) > 0
        done = true
        for np in node_iterator(sched,outneighbors(sched,n))
            if has_robot_id(np) && get_robot_id(np) == id
                push!(itinerary,np)
                n = np
                done = false
                break
            end
        end
        if done
            break
        end
    end
    return itinerary
end

export construct_task_graphs_problem
"""
    `construct_task_graphs_problem`
"""
function construct_task_graphs_problem(
        project_spec::ProjectSpec,
        r0::Vector{Int},
        s0::Vector{Int},
        sF::Vector{Int},
        env;
        Δt_collect=zeros(length(s0)),
        Δt_deposit=zeros(length(sF)),
        cost_function=SumOfMakeSpans(),
        task_shapes=map(o->(1,1),s0),
        shape_dict=Dict{Int,Dict{Tuple{Int,Int},Vector{Int}}}(
            s=>Dict{Tuple{Int,Int},Vector{Int}}() for s in vcat(s0,sF)
            )
        ) 
    N = length(r0)
    M = length(s0)
    for j in 1:M
        get!(shape_dict,s0[j],Dict{Tuple{Int,Int},Vector{Int}}())
        get!(shape_dict,sF[j],Dict{Tuple{Int,Int},Vector{Int}}())
        @assert haskey(shape_dict, s0[j]) "shape_dict has no key for s0[$j] = $(s0[j])"
        @assert haskey(shape_dict, sF[j]) "shape_dict has no key for sF[$j] = $(sF[j])"
        @assert length(task_shapes) >= j "task_shapes has no key for j = $j"
    end
    robot_ICs = [ROBOT_AT(r,x) for (r,x) in enumerate(r0)] # initial robot conditions
    object_ICs = [OBJECT_AT(j, get(shape_dict[x], s, x), s) for (j,(x,s)) in enumerate(zip(s0,task_shapes))] # initial object conditions
    object_FCs = [OBJECT_AT(j, get(shape_dict[x], s, x), s) for (j,(x,s)) in enumerate(zip(sF,task_shapes))] # initial object conditions
    new_project_spec = ProjectSpec(object_ICs,object_FCs)
    for op in get_operations(project_spec)
        add_operation!(
            new_project_spec,
            construct_operation(
                new_project_spec,
                op.station_id,
                collect(keys(preconditions(op))),
                collect(keys(postconditions(op))),
                duration(op),
                op.id
            )
        )
    end

    problem_spec = ProblemSpec(
        D=env,
        cost_function=cost_function,
        Δt_collect = Dict{ObjectID,Int}(
            get_object_id(n)=>t for (n,t) in zip(object_ICs,Δt_collect)),
        Δt_deposit = Dict{ObjectID,Int}(
            get_object_id(n)=>t for (n,t) in zip(object_ICs,Δt_deposit)),
        )

    sched = construct_partial_project_schedule(new_project_spec,problem_spec,robot_ICs)
    return sched, problem_spec
end
function construct_task_graphs_problem(
    def::SimpleProblemDef,
    env::GridFactoryEnvironment;
    task_shapes = def.shapes,
    kwargs...,
)
    construct_task_graphs_problem(
        def.project_spec,
        def.r0,
        def.s0,
        def.sF,
        env;
        task_shapes = task_shapes,
        shape_dict = env.expanded_zones,
        kwargs...,
    )
end

export
    construct_random_project_spec,
    get_random_problem_instantiation,
    construct_random_task_graphs_problem

"""
    construct_random_project_spec(M::Int;max_children=1)

    Inputs:
        `M` - number of objects involved in the operation
        `max_parents` - determines the max number of inputs to any operation
        `depth_bias` ∈ [0,1] - hyperparameter for tuning depth.
            If `depth_bias` == 1.0, the project_spec graph will always be depth
            balanced (all paths through the tree will be of the same length).
            For `depth_bias` == 0.0, the graph will be as "strung out" as
            possible.
"""
function construct_random_project_spec(M::Int,object_ICs::Vector{OBJECT_AT},object_FCs::Vector{OBJECT_AT};
        max_parents=1,
        depth_bias=1.0,
        Δt_min=0,
        Δt_max=0,
        zone_map=Dict{Int,Vector{Int}}(), # maps vtx to vector of vtxs for collaborative tasks
    )
    project_spec = ProjectSpec(object_ICs,object_FCs)
    @assert length(object_ICs) == M
    object_map = map(get_object_id, initial_conditions_vector(project_spec))
    # fill with random operations going backwards
    i = M-1
    frontier = PriorityQueue{Int,Int}([M=>1])
    while i > 0
        depth = 1
        while true
            if (rand() > depth_bias) && (depth < length(frontier))
                depth += 1
            else
                break
            end
        end
        pairs = Vector{Pair{Int,Int}}()
        # pair = Pair{Int,Int}(0,0)
        for d in 1:depth
            push!(pairs, dequeue_pair!(frontier))
        end
        for p in pairs[1:end-1]
            enqueue!(frontier,p)
        end
        output_idx = pairs[end].first
        station_id = output_idx
        input_idxs = collect(max(1,1+i-rand(1:max_parents)):i)
        i = i - length(input_idxs)
        Δt=rand(Δt_min:Δt_max)
        input_ids = map(idx->object_map[idx], input_idxs)
        output_ids = map(idx->object_map[idx], [output_idx])
        add_operation!(project_spec,construct_operation(project_spec, station_id, input_ids, output_ids, Δt))
        for idx in input_idxs
            enqueue!(frontier, idx=>M-i)
        end
    end
    Δt=0
    final_idx = object_map[M]
    add_operation!(project_spec,construct_operation(project_spec, -1, [final_idx], [], Δt))
    project_spec
end
# function construct_random_project_spec(M::Int,s0::Vector{Int},sF::Vector{Int};
function construct_random_project_spec(M::Int,s0::Vector{V},sF::Vector{V};kwargs...) where {V<:Union{Int,Vector{Int}}}
    object_ICs = Vector{OBJECT_AT}([OBJECT_AT(o,s) for (o,s) in enumerate(s0)])
    object_FCs = Vector{OBJECT_AT}([OBJECT_AT(o,s) for (o,s) in enumerate(sF)])
    construct_random_project_spec(M,object_ICs,object_FCs;
        kwargs...)
end

export
    choose_random_object_sizes,
    choose_and_shuffle_object_sizes

"""
    Tool for randomly selecting how many robots (and in what configuration)
        should deliver each task.
"""
function choose_random_object_sizes(M,probs::Dict{Tuple{Int,Int},Float64})
    k = sort(collect(keys(probs)))
    v = cumsum(map(key->probs[key], k))
    v = v / sum(collect(values(probs)))
    sizes = Vector{Tuple{Int,Int}}()
    for j in 1:M
        n = rand()
        i = 1
        while n > v[i]
            i += 1
        end
        push!(sizes,k[i])
    end
    sizes
end
function choose_and_shuffle_object_sizes(M,probs::Dict{Tuple{Int,Int},Float64})
    k = sort(collect(keys(probs)),by=i->i[1]*i[2])
    v = map(key->probs[key], k)
    v = M * v / sum(v)
    v_rounded = Int.(round.(v))
    while sum(v_rounded) < sum(v)
        idx = sort(collect(1:length(v)),by=i->abs(v[i]-v_rounded[i]))[end]
        v_rounded[idx] = v_rounded[idx] + 1
    end
    while sum(v_rounded) > sum(v)
        idx = sort(collect(1:length(v)),by=i->abs(v[i]-v_rounded[i]))[end]
        v_rounded[idx] = v_rounded[idx] - 1
    end
    sizes = Vector{Tuple{Int,Int}}()
    for (idx,n) in enumerate(v_rounded)
        for i in 1:n
            push!(sizes,k[idx])
        end
    end
    shuffle(sizes)
end
function choose_random_object_sizes(M,probs::Dict{Int,Float64}=Dict(1=>1.0,2=>0.0,4=>0.0),choices=[(1,1),(2,1),(1,2),(2,2)])
    k = sort(collect(keys(probs)))
    size_probs = Dict{Tuple{Int,Int},Float64}(s=>probs[s[1]*s[2]] for s in choices)
    for ksize in k
        counts = sum(map(s->s[1]*s[2]==ksize, choices))
        for s in choices
            size_probs[s] = size_probs[s] / counts
        end
    end
    choose_and_shuffle_object_sizes(M,size_probs)
end

"""
    `get_random_problem_instantiation`

    Args:
    - `N`: number of robots
    - `M`: number of delivery tasks
    - `robot_zones`: list of possible start locations for robots
    - `pickup_zones`: list of possible start locations for objects
    - `dropoff_zones`: list of possible destinations for objects
"""
function get_random_problem_instantiation(N::Int,M::Int,pickup_zones,dropoff_zones,robot_zones)
    ##### Random Problem Initialization #####
    r0 = robot_zones[sortperm(rand(length(robot_zones)))][1:N]
    pickup_idxs = sortperm(rand(length(pickup_zones)))
    while length(pickup_idxs) < M
        pickup_idxs = vcat(pickup_idxs,sortperm(rand(length(pickup_zones))))
    end
    s0 = map(i->pickup_zones[i], pickup_idxs[1:M])
    # s0 = pickup_zones[sortperm(rand(length(pickup_zones)))][1:M]
    dropoff_idxs = sortperm(rand(length(dropoff_zones)))
    while length(dropoff_idxs) < M
        dropoff_idxs = vcat(dropoff_idxs,sortperm(rand(length(dropoff_zones))))
    end
    sF = map(i->dropoff_zones[i], dropoff_idxs[1:M])
    # sF = dropoff_zones[sortperm(rand(length(dropoff_zones)))][1:M]
    return r0,s0,sF
end

"""
    `construct_randomd_task_graphs_problem`
"""
function construct_random_task_graphs_problem(N::Int,M::Int,
    pickup_vtxs::Vector{Int},dropoff_vtxs::Vector{Int},free_vtxs::Vector{Int},dist_matrix,
    Δt_collect::Vector{Float64}=zeros(M),
    Δt_deposit::Vector{Float64}=zeros(M)
    )
    # select subset of pickup, dropoff and free locations to instantiate objects and robots
    r0,s0,sF        = get_random_problem_instantiation(N,M,pickup_vtxs,dropoff_vtxs,free_vtxs)
    project_spec    = construct_random_project_spec(M,s0,sF;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)

    construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix;Δt_collect=Δt_collect,Δt_deposit=Δt_deposit)
end

################################################################################
################################### Rendering ##################################
################################################################################

export
    get_display_metagraph


function get_display_metagraph(project_schedule::OperatingSchedule;
    verbose=true,
    f=(v,p)->title_string(p,verbose),
    object_color="orange",
    robot_color="lime",
    clean_up_bot_color="yellow",
    action_color="cyan",
    operation_color="red",
    remove_leaf_robots=false
    )
    graph = MetaDiGraph(deepcopy(project_schedule.graph))
    for (id,pred) in get_object_ICs(project_schedule)
        v = get_vtx(project_schedule, get_object_id(pred))
        set_prop!(graph, v, :vtype, :object_ic)
        set_prop!(graph, v, :text, f(v,pred))
        set_prop!(graph, v, :color, object_color)
        set_prop!(graph, v, :vtx_id, v)
    end
    for (id,op) in get_operations(project_schedule)
        v = get_vtx(project_schedule, OperationID(id))
        set_prop!(graph, v, :vtype, :operation)
        set_prop!(graph, v, :text, f(v,op))
        set_prop!(graph, v, :color, operation_color)
        set_prop!(graph, v, :vtx_id, v)
    end
    for (id,a) in get_actions(project_schedule)
        v = get_vtx(project_schedule, ActionID(id))
        set_prop!(graph, v, :vtype, :action)
        set_prop!(graph, v, :text, f(v,a))
        set_prop!(graph, v, :color, action_color)
        set_prop!(graph, v, :vtx_id, v)
    end
    for (id,pred) in get_robot_ICs(project_schedule)
        v = get_vtx(project_schedule, get_robot_id(pred))
        set_prop!(graph, v, :vtype, :robot_ic)
        set_prop!(graph, v, :text, f(v,pred))
        if isa(id,CleanUpBotID)
            set_prop!(graph, v, :color, clean_up_bot_color)
        else
            set_prop!(graph, v, :color, robot_color)
        end
        set_prop!(graph, v, :vtx_id, v)
    end
    if remove_leaf_robots == true
        for v in reverse(vertices(graph))
            # if get_prop(graph,v,:vtype) ==:robot_ic
            if typeof(get_vtx_id(project_schedule,v)) <: RobotID
                if length(outneighbors(graph,v)) == 0
                    rem_vertex!(graph,v)
                end
            end
        end
    end
    graph
end

################################################################################
######################### COMPARISONS AND VERIFICATION #########################
################################################################################
function Base.:(==)(o1::OBJECT_AT,o2::OBJECT_AT)
    try
        @assert(o1.o == o2.o)
        @assert(o1.x == o2.x)
    catch e
        if isa(e,AssertionError)
            # println(e.msg)
        else
            throw(e)
        end
        return false
    end
    return true
end
function Base.:(==)(op1::Operation,op2::Operation)
    try
        @assert(preconditions(op1) == preconditions(op2))
        @assert(postconditions(op1) == postconditions(op2))
        @assert(op1.Δt == op2.Δt)
        @assert(op1.station_id == op2.station_id)
    catch e
        if isa(e,AssertionError)
            # println(e.msg)
        else
            throw(e)
        end
        return false
    end
    return true
end
function Base.:(==)(spec1::ProjectSpec,spec2::ProjectSpec)
    try
        @assert(spec1.initial_conditions == spec2.initial_conditions)
        @assert(spec1.final_conditions == spec2.final_conditions)
    catch e
        if isa(e,AssertionError)
            # println(e.msg)
        else
            throw(e)
        end
        return false
    end
    return true
end


export
    get_object_paths

function get_object_paths(
        solution,
        sched, 
        robot_paths = convert_to_vertex_lists(solution),
        )
    tF = maximum(map(length, robot_paths))
    object_paths = Vector{Vector{Int}}()
    object_intervals = Vector{Vector{Int}}()
    object_ids = Int[]
    path_idxs = Int[]
    for v in vertices(sched.graph)
        node = get_node_from_id(sched,get_vtx_id(sched,v))
        if isa(node, Union{CARRY,TEAM_CARRY})
            if isa(node, CARRY)
                object_id = get_object_id(node)
                agent_id_list = [get_id(get_robot_id(node))]
                s0_list = [get_id(get_initial_location_id(node))]
                sF_list = [get_id(get_destination_location_id(node))]
            else
                object_id = get_object_id(node.instructions[1])
                agent_id_list = map(n->get_id(get_robot_id(n)), node.instructions)
                s0_list = map(n->get_id(get_initial_location_id(n)), node.instructions)
                sF_list = map(n->get_id(get_destination_location_id(n)), node.instructions)
            end
            object_vtx = get_vtx(sched,object_id)
            for (idx,(agent_id,s0,sF)) in enumerate(zip(agent_id_list,s0_list,sF_list))
                if get_id(agent_id) != -1
                    push!(object_paths,[
                        map(t->s0,
                            0:Int(round(get_t0(sched,v)))-1)...,
                        map(t->robot_paths[agent_id][t],
                            min(Int(round(get_t0(sched,v)))+1,tF):min(Int(round(get_tF(sched,v)))+1,
                            tF,
                            length(robot_paths[agent_id])))...,
                        map(t->sF,
                            min(Int(round(get_tF(sched,v)))+1,tF):tF)...
                    ])
                    push!(object_intervals,[
                        get_t0(sched,object_vtx),
                        get_tF(sched,v)+1])
                    push!(object_ids, get_id(object_id))
                    push!(path_idxs, idx)
                end
            end
        end
    end
    object_paths, object_intervals, object_ids, path_idxs
end

export
    fill_object_path_dicts!,
    convert_to_path_vectors

function fill_object_path_dicts!(solution,project_schedule,cache,
        object_path_dict = Dict{Int,Vector{Vector{Int}}}(),
        object_interval_dict = Dict{Int,Vector{Int}}()
    )
    object_paths, object_intervals, object_ids, path_idxs = get_object_paths(solution,project_schedule)
    for (path,interval,id,idx) in zip(object_paths, object_intervals, object_ids, path_idxs)
        if !haskey(object_path_dict,id)
            object_path_dict[id] = Vector{Vector{Int}}()
        end
        if length(object_path_dict[id]) >= idx
            object_path_dict[id][idx] = path
        else
            push!(object_path_dict[id], path)
        end
        object_interval_dict[id] = interval
    end
    object_path_dict, object_interval_dict
end

function convert_to_path_vectors(object_path_dict, object_interval_dict)
    object_paths = Vector{Vector{Int}}()
    object_intervals = Vector{Vector{Int}}()
    for id in sort(collect(keys(object_path_dict)))
        paths = object_path_dict[id]
        for path in paths
            push!(object_intervals, object_interval_dict[id])
            push!(object_paths, path)
        end
    end
    object_paths, object_intervals
end

function print_schedule_node_details(io::IO,sched,n)
    print(io,sprint_padded(string(n.node);pad=20)," - ")
    s = "t0: $(get_t0(n)), tF: $(get_tF(n)), "
    print(io,s)
    print(io,"fixed: $(n.spec.fixed), ")
    print(io,"inneighbors: ",[string(get_node(sched,vp).node) for vp in inneighbors(sched,n)]...)
    print(io,", outneighbors: ",[string(get_node(sched,vp).node) for vp in outneighbors(sched,n)]...)
    print(io,"\n")
end