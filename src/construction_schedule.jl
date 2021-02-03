export  
    start_config,
    goal_config,
    entity,
    ConstructionPredicate,
    EntityConfigPredicate,
    RobotStart,
    ObjectStart,
    AssemblyComplete,
    AssemblyStart,
    EntityGo,
    RobotGo,
    TransportUnitGo,
    LiftIntoPlace,
    FormTransportUnit,
    DepositCargo,
    BuildPhasePredicate,
    OpenBuildStep,
    CloseBuildStep,
    ProjectComplete


"""
    abstract type ConstructionPredicate

Abstract type for ConstructionSchedule nodes.
"""
abstract type ConstructionPredicate end
start_config(n)     = n.start_config
goal_config(n)      = n.goal_config
entity(n)           = n.entity

for op in (:start_config,:goal_config,:entity)
    @eval begin
        $op(n::CustomNode) = $op(node_val(n))
    end
end

abstract type EntityConfigPredicate{E,T} <: ConstructionPredicate end
start_config(n::EntityConfigPredicate)   = n.config
goal_config(n::EntityConfigPredicate)    = n.config

struct RobotStart{T} <: EntityConfigPredicate{RobotNode,T}
    entity::RobotNode
    config::T # local or global transform? Leaning toward global
end
struct ObjectStart{T} <: EntityConfigPredicate{ObjectNode,T}
    entity::ObjectNode
    config::T
end
struct AssemblyComplete{T} <: EntityConfigPredicate{AssemblyNode,T}
    entity::AssemblyNode
    config::T
end
struct AssemblyStart{T} <: EntityConfigPredicate{AssemblyNode,T}
    entity::AssemblyNode
    config::T
end


"""
    abstract type EntityGo{E,S,G}

Encodes going from state start_config(n) to state goal_config(n).
"""
abstract type EntityGo{E,S,G} <: ConstructionPredicate end

struct RobotGo{S,G} <: EntityGo{RobotNode,S,G}
    entity::RobotNode
    start_config::S
    goal_config::G
end

struct TransportUnitGo{S,G} <: EntityGo{TransportUnitNode,S,G}
    entity::TransportUnitNode
    start_config::S
    goal_config::G
end

struct LiftIntoPlace{C,S,G} <: EntityGo{C,S,G}
    entity::C # AssemblyNode or ObjectNode
    start_config::S
    goal_config::G
end
# GraphUtils.node_id(n::LiftIntoPlace{C,S,G}) where {C,S,G} = TemplatedID{Tuple{LiftIntoPlace,C}}(get_id(node_id(entity(n))))


"""
    abstract type BuildPhasePredicate <: ConstructionPredicate

References a building phase. Interface: 
- `get_assembly(::BuildPhasePredicate)` => ::AssemblyNode - the assembly to be 
modified by this build phase
- `assembly_components(::BuildPhasePredicate)` => TransformDict{Union{ObjectID,AssemblyID}} 
- the components to be added to the assembly during this phase
"""
abstract type BuildPhasePredicate <: ConstructionPredicate end

get_assembly(n::BuildPhasePredicate) = n.assembly
HierarchicalGeometry.assembly_components(n::BuildPhasePredicate) = n.components
HierarchicalGeometry.assembly_components(n::ConstructionPredicate) = assembly_components(entity(n))

"""
    extract_building_phase(n::BuildingStep,tree::SceneTree,model_spec,id_map)

extract the assembly and set of subcomponents from a BuildingStep.
"""
function extract_building_phase(n,tree::SceneTree,model_spec,id_map)
    @assert matches_template(BuildingStep,n)
    assembly_id = id_map[node_val(n).parent]
    @assert isa(assembly_id, AssemblyID)
    assembly = get_node(tree,assembly_id)
    parts = typeof(assembly_components(assembly))()
    for child_id in get_build_step_components(model_spec,id_map,n)
        @assert has_component(assembly, child_id) "$child_id is not a child of assembly $assembly"
        parts[child_id] = child_transform(assembly,child_id)
    end
    # for vp in inneighbors(model_spec,n)
    #     child_id = get_referenced_component(model_spec,id_map,get_vtx_id(model_spec,vp))
    #     # child_id = get(id_map,get_vtx_id(model_spec,vp),nothing)
    #     if !(child_id === nothing)
    #         @assert has_component(assembly, child_id) "$child_id is not a child of assembly $assembly"
    #         parts[child_id] = child_transform(assembly,child_id)
    #     end
    # end
    return assembly, parts 
end

"""
    OpenBuildStep <: ConstructionPredicate

Marks the beginning of a new building phase.
Fields:
- assembly::AssemblyNode - the assembly to be modified by this build phase
- components::Dict{Union{AssemblyID,ObjectID}} - the components to be added to
the assembly during this phase
"""
struct OpenBuildStep <: BuildPhasePredicate
    assembly::AssemblyNode 
    components::TransformDict{Union{AssemblyID,ObjectID}}
end
struct CloseBuildStep <: BuildPhasePredicate
    assembly::AssemblyNode 
    components::TransformDict{Union{AssemblyID,ObjectID}}
end
for T in (:OpenBuildStep,:CloseBuildStep)
    @eval begin 
        $T(n::CustomNode,args...) = $T(extract_building_phase(n,args...)...)
        $T(n::BuildPhasePredicate) = $T(n.assembly,n.components)
    end
end

struct DepositCargo{S,G} <: ConstructionPredicate
    entity::TransportUnitNode
    start_config::S # of assembly or transport unit?
    goal_config::G
end
struct FormTransportUnit{S,G} <: ConstructionPredicate
    entity::TransportUnitNode
    start_config::S
    goal_config::G
end

for T in (:RobotStart,:ObjectStart,:AssemblyComplete,:AssemblyStart)
    @eval begin
        $T(n::SceneNode) = $T(n,global_transform(n))
        $T(n::ConstructionPredicate) = $T(entity(n),goal_config(n))
        function $T(n::EntityConfigPredicate{A,C}) where {A,C<:TransformNode}
            node = $T(entity(n),start_config(n))
            # node = $T(entity(n),TransformNode())
            # set_parent!(goal_config(node),start_config(n))
            # node
        end
        set_start_config(n::$T,c) = $T(entity(n),c)
        set_goal_config(n::$T,c) = $T(entity(n),c)
    end
end
for T in (:RobotGo,:TransportUnitGo,:LiftIntoPlace)
    @eval begin
        function $T(n::EntityConfigPredicate{A,C},obj) where {A,C<:TransformNode}
            node = $T(obj,TransformNode(),TransformNode())
            set_parent!(goal_config(node),start_config(n))
            set_parent!(start_config(node),goal_config(node))
            node
        end
        function $T(n::EntityGo{E,S,G},obj) where {E,S<:TransformNode,G}
            node = $T(obj,TransformNode(),TransformNode())
            set_parent!(goal_config(node),start_config(n))
            set_parent!(start_config(node),goal_config(node))
            node
        end
        set_start_config(n::$T,c) = $T(entity(n),c,goal_config(n))
        set_goal_config(n::$T,c) = $T(entity(n),start_config(n),goal_config(n))
    end
end
for T in (:RobotGo,:TransportUnitGo,:LiftIntoPlace,:DepositCargo,:FormTransportUnit)
    @eval $T(n::SceneNode) = $T(n,global_transform(n),global_transform(n))
end

HierarchicalGeometry.robot_team(n::ConstructionPredicate) = robot_team(entity(n))
cargo_node_type(n::TransportUnitNode) = cargo_type(n) == AssemblyNode ? AssemblyComplete : ObjectStart
cargo_node_type(n::ConstructionPredicate) = cargo_node_type(entity(n))

struct ProjectComplete <: ConstructionPredicate 
    project_id::Int
end
function ProjectComplete()
    id = get_id(get_unique_id(TemplatedID{ProjectComplete}))
    ProjectComplete(id)
end
GraphUtils.node_id(n::ProjectComplete) = TemplatedID{ProjectComplete}(n.project_id)

GraphUtils.required_predecessors(::ConstructionPredicate)   = Dict()
GraphUtils.required_successors(::ConstructionPredicate)     = Dict()
GraphUtils.eligible_predecessors(n::ConstructionPredicate)  = required_predecessors(n)
GraphUtils.eligible_successors(n::ConstructionPredicate)    = required_successors(n)

GraphUtils.required_predecessors(   ::RobotStart)       = Dict()
GraphUtils.required_successors(     ::RobotStart)       = Dict(RobotGo=>1)

GraphUtils.required_predecessors(   ::RobotGo)          = Dict(Union{RobotStart,DepositCargo,RobotGo}=>1)
GraphUtils.required_successors(     ::RobotGo)          = Dict()
GraphUtils.eligible_successors(     ::RobotGo)          = Dict(Union{RobotGo,FormTransportUnit}=>1)

GraphUtils.required_predecessors(  n::FormTransportUnit) = Dict(RobotGo=>length(robot_team(n)),cargo_node_type(n)=>1,Union{ObjectStart,AssemblyComplete}=>1)
GraphUtils.required_successors(     ::FormTransportUnit) = Dict(TransportUnitGo=>1)

GraphUtils.required_predecessors(   ::TransportUnitGo)  = Dict(FormTransportUnit=>1)
GraphUtils.required_successors(     ::TransportUnitGo)  = Dict(DepositCargo=>1)

GraphUtils.required_predecessors(   ::DepositCargo)     = Dict(TransportUnitGo=>1,OpenBuildStep=>1)
GraphUtils.required_successors(    n::DepositCargo)     = Dict(LiftIntoPlace=>1,RobotGo=>length(robot_team(n)))

GraphUtils.required_predecessors(   ::LiftIntoPlace)    = Dict(DepositCargo=>1)
GraphUtils.required_successors(     ::LiftIntoPlace)    = Dict(CloseBuildStep=>1)

GraphUtils.required_predecessors(  n::CloseBuildStep)   = Dict(LiftIntoPlace=>num_components(n))
GraphUtils.required_successors(     ::CloseBuildStep)   = Dict(Union{OpenBuildStep,AssemblyComplete}=>1)

GraphUtils.required_predecessors(   ::OpenBuildStep)    = Dict(Union{AssemblyStart,CloseBuildStep}=>1)
GraphUtils.required_successors(    n::OpenBuildStep)    = Dict(DepositCargo=>num_components(n))

GraphUtils.required_predecessors(   ::ObjectStart)      = Dict()
GraphUtils.required_successors(     ::ObjectStart)      = Dict(FormTransportUnit=>1)

GraphUtils.required_predecessors(  n::AssemblyComplete) = Dict(Union{ObjectStart,CloseBuildStep}=>num_components(n))
GraphUtils.required_successors(     ::AssemblyComplete) = Dict(Union{FormTransportUnit,ProjectComplete}=>1)

GraphUtils.required_predecessors(   ::AssemblyStart)    = Dict()
GraphUtils.required_successors(     ::AssemblyStart)    = Dict(OpenBuildStep=>1)

GraphUtils.required_predecessors(   ::ProjectComplete)  = Dict(AssemblyComplete=>1)
GraphUtils.required_successors(     ::ProjectComplete)  = Dict()


for T in (
    :ObjectStart,
    :RobotStart,
    :RobotGo,
    :AssemblyStart,
    :AssemblyComplete,
    :FormTransportUnit,
    :TransportUnitGo,
    :DepositCargo,
    :LiftIntoPlace,
)
    @eval begin
        function GraphUtils.node_id(n::$T) 
            TemplatedID{Tuple{$T,typeof(node_id(entity(n)))}}(get_id(node_id(entity(n))))
        end
    end
end

# for T in (
#     :ObjectStart,
#     :RobotStart,
#     :RobotGo,
#     :AssemblyStart,
#     :AssemblyComplete,
# )
#     @eval begin
#         GraphUtils.node_id(n::$T) = TemplatedID{$T}(get_id(node_id(entity(n))))
#     end
# end
GraphUtils.add_node!(g::AbstractCustomNGraph, n::ConstructionPredicate) = add_node!(g,n,node_id(n))
GraphUtils.add_node!(g::AbstractCustomNGraph, n::P) where {P<:BuildPhasePredicate} = add_node!(g,n,get_unique_id(TemplatedID{P}))
GraphUtils.get_vtx(g::AbstractCustomNGraph,n::ConstructionPredicate) = get_vtx(g,node_id(n))

"""
    get_previous_build_step(model_spec,v)

Return the node representing the previous build step in the model spec.
"""
function get_previous_build_step(model_spec,v;
        skip_first=matches_template(SubModelPlan,get_node(model_spec,v)),
    )
    vp = depth_first_search(model_spec,get_vtx(model_spec,v),
        u->matches_template(BuildingStep,get_node(model_spec,u)),
        u->(get_vtx(model_spec,v) == u || !matches_template(SubModelPlan,get_node(model_spec,u))),
        inneighbors;
        skip_first=skip_first,
    )
    if has_vertex(model_spec,vp)
        return get_node(model_spec,vp)
    end
    return nothing
end

export construct_partial_construction_schedule

"""
    populate_schedule_build_step!(sched,cb,step_node,model_spec,scene_tree,id_map)

Creates an OpenBuildStep ... CloseBuildStep structure and adds an edge 
CloseBuildStep => parent.
Args:
- sched - the schedule
- cb::CustomNode{CloseBuildStep,AbstractID}
- model_spec
- scene_tree
- id_map
"""
function populate_schedule_build_step!(sched,parent::AssemblyComplete,cb,step_node,model_spec,scene_tree,id_map;
        connect_to_sub_assemblies=true,
    )
    # OpenBuildStep
    ob = add_node!(sched,OpenBuildStep(node_val(cb)))
    for child_id in get_build_step_components(model_spec,id_map,step_node)
        cargo = get_node(scene_tree,child_id)
        @assert isa(cargo,Union{AssemblyNode,ObjectNode})
        # LiftIntoPlace
        # l = add_node!(sched,    LiftIntoPlace(cargo)) 
        l = add_node!(sched,    LiftIntoPlace(cargo,TransformNode(),TransformNode())) #######
        set_parent!(goal_config(l),start_config(parent))
        # set_parent!(start_config(l),goal_config(l))
        set_parent!(start_config(l),start_config(parent)) # point to parent
        add_edge!(sched,l,cb) # LiftIntoPlace => CloseBuildStep
        # DepositCargo
        # d = add_node!(sched,    DepositCargo(TransportUnitNode(cargo)))
        d = add_node!(sched,    DepositCargo(TransportUnitNode(cargo),TransformNode(),TransformNode()))
        set_parent!(goal_config(d),start_config(l))
        set_parent!(start_config(d),goal_config(d))
        add_edge!(sched,d,l) # DepositCargo => LiftIntoPlace
        add_edge!(sched,ob,d) # OpenBuildStep => DepositCargo
        # TransportUnitGo
        # tgo = add_node!(sched,  TransportUnitGo(entity(node_val(d))))
        tgo = add_node!(sched,  TransportUnitGo(entity(node_val(d)),TransformNode(),TransformNode()))
        set_parent!(goal_config(tgo),start_config(d))
        add_edge!(sched,tgo,d) # TransportUnitGo => DepositCargo
        # FormTransportUnit
        # f = add_node!(sched,    FormTransportUnit(entity(node_val(d))))
        f = add_node!(sched,    FormTransportUnit(entity(node_val(d)),TransformNode(),TransformNode()))
        set_parent!(start_config(tgo),goal_config(f))
        add_edge!(sched,f,tgo) # FormTransportUnit => TransportUnitGo
        if isa(cargo,AssemblyNode) && connect_to_sub_assemblies
            # add_edge!(sched,AssemblyComplete(cargo),f) # AssemblyComplete => FormTransportUnit 
            cargo_node = get_node(sched,AssemblyComplete(cargo))
            add_edge!(sched,cargo_node,f) # AssemblyComplete => FormTransportUnit 
            set_parent!(goal_config(f),start_config(cargo_node))
        end
    end
    return ob
end

"""
    populate_schedule_sub_graph!(
        sched,
        parent::AssemblyComplete,
        model_spec,
        scene_tree,
        id_map)

Add all building steps to parent, working backward from parents
"""
function populate_schedule_sub_graph!(sched,parent::AssemblyComplete,model_spec,scene_tree,id_map)
    parent_assembly = entity(parent)
    # sa = add_node!(sched,AssemblyStart(parent_assembly))
    sa = add_node!(sched,AssemblyStart(parent)) ################
    spec_node = get_node(model_spec, id_map[node_id(parent_assembly)])
    step_node = get_previous_build_step(model_spec,spec_node;skip_first=true)
    immediate_parent = parent
    while !(step_node === nothing)
        # CloseBuildStep
        cb = add_node!(sched,CloseBuildStep(step_node,scene_tree,model_spec,id_map))
        add_edge!(sched,cb,immediate_parent) # CloseBuildStep => AssemblyComplete / OpenBuildStep
        ob = populate_schedule_build_step!(sched,parent,cb,step_node,model_spec,scene_tree,id_map)
        immediate_parent = ob
        step_node = get_previous_build_step(model_spec,step_node;skip_first=true)
    end
    add_edge!(sched,sa,immediate_parent) # AssemblyStart => OpenBuildStep
    sched
end

"""
    construct_partial_construction_schedule(sched,args...)

Constructs a schedule that does not yet reflect the "staging plan" and does not
yet contain any RobotStart or RobotGo nodes.
"""
function construct_partial_construction_schedule(
        mpd_model,
        model_spec,
        scene_tree,
        id_map=build_id_map(mpd_model,model_spec)
    )
    sched = NGraph{DiGraph,ConstructionPredicate,AbstractID}()
    parent_map = backup_descendants(model_spec,n->matches_template(SubModelPlan,n))
    # Add assemblies first
    for v in topological_sort_by_dfs(model_spec)
        node = get_node(model_spec,v)
        if matches_template(SubModelPlan,node)
            assembly = get_node(scene_tree,id_map[node_id(node)])
            # AssemblyComplete
            a = add_node!(sched,AssemblyComplete(assembly,TransformNode())) ###############
            if is_root_node(scene_tree,assembly)
                # ProjectComplete
                p = add_node!(sched,ProjectComplete())
                add_edge!(sched,a,p)
            end
            # add build steps
            populate_schedule_sub_graph!(sched,node_val(a),model_spec,scene_tree,id_map)
        end
    end
    sched
end

export validate_schedule_transform_tree

function assert_transform_tree_ancestor(a,b)
    @assert GraphUtils.has_ancestor(a,b) "a should have ancestor b, for a = $(a), b = $(b)"
end

function transformations_approx_equiv(t1,t2)
    a = all(isapprox.(t1.translation,t2.translation))
    b = all(isapprox.(t1.linear,t2.linear))
    a && b
end

"""
    validate_schedule_transform_tree(sched)

Checks if sched and its embedded transform tree are valid.
- The graph itself should be valid
- All chains AssemblyComplete ... LiftIntoPlace -> DepositCargo should be 
    connected in the embedded transform tree

"""
function validate_schedule_transform_tree(sched;post_staging=false)
    try
        @assert GraphUtils.validate_graph(sched)
        for n in get_nodes(sched)
            if matches_template(AssemblyComplete,n)
                @assert validate_tree(goal_config(n)) "Subtree invalid for $(n)"
            end
        end
        for n in get_nodes(sched)
            if matches_template(OpenBuildStep,n)
                open_build_step = node_val(n)
                assembly = open_build_step.assembly
                assembly_complete = get_node(sched,AssemblyComplete(assembly))
                for v in outneighbors(sched,n)
                    child = get_node(sched,v)
                    @assert matches_template(DepositCargo,child)
                    assert_transform_tree_ancestor(goal_config(child),start_config(assembly_complete))
                    for vp in outneighbors(sched,v)
                        lift_node = get_node(sched,vp)
                        @assert matches_template(LiftIntoPlace,lift_node)
                        @assert GraphUtils.has_child(
                            goal_config(assembly_complete),start_config(lift_node))
                        @assert GraphUtils.has_child(
                            goal_config(assembly_complete),goal_config(lift_node))
                        if post_staging
                            # Show that goal_config(LiftIntoPlace) matches the 
                            # goal config of cargo relative to assembly
                            @assert transformations_approx_equiv(
                                local_transform(goal_config(lift_node)),
                                child_transform(assembly,node_id(entity(lift_node)))
                            )
                            # Show that start_config(LiftIntoPlace) matches 
                            # goal_config(DepositCargo)
                            @assert transformations_approx_equiv(
                                global_transform(start_config(lift_node)),
                                global_transform(goal_config(child)),
                            )
                        end
                    end
                end
            elseif matches_template(LiftIntoPlace,n)
                # @assert GraphUtils.has_child(goal_config(n),start_config(n))
            end
        end
    catch e
        if isa(e,AssertionError)
            bt = catch_backtrace()
            showerror(stderr,e,bt)
            return false
        else
            rethrow(e)
        end
    end
    return true
end


"""
    generate_staging_plan(scene_tree,params)

Given a `SceneTree` representing the final configuration of an assembly, 
construct a plan for the start config, staging config, and final config of
each subassembly and individual object. The idea is to ensure that no node of 
the "plan" graph overlaps with any other. Requires circular bounding geometry
for each component of the assembly. 
    Start at terminal assembly
    work downward through building steps
    For each building step, arrange the incoming parts to balance these 
    objectives: 
    - minimize "LiftIntoPlace" distance
    - maximize distance between components to be placed. 
    It may be necessary to not completely isolate build steps (i.e., two 
    consecutive build steps may overlap in the arrival times of subcomponents).
"""
function generate_staging_plan!(scene_tree,sched;
        robot_radius=0.0,
    )
    if !all(map(n->has_vertex(n.geom_hierarchy, HypersphereKey()), get_nodes(scene_tree)))
        HierarchicalGeometry.compute_approximate_geometries!(scene_tree,
            HypersphereKey();ϵ=0.0)
    end
    # To store the transform tree of start, staging, and final configs 
    start_configs   = TransformDict{AbstractID}()

    # store growing bounding circle of each assembly
    bounding_radii = Dict{AssemblyID,Float64}() 
    staging_radii = Dict{AssemblyID,Float64}() # store radius of staging area for each assembly
    for v in topological_sort_by_dfs(sched)
        node = get_node(sched,v)
        if matches_template(AssemblyStart,node)
            bounding_radii[node_id(entity(node_val(node)))] = 0.0
            staging_radii[node_id(entity(node_val(node)))] = 0.0
        elseif matches_template(OpenBuildStep,node)
            # work updward through build steps
            # Set staging config of each part as the start_config of its
            # LiftIntoPlace node.
            process_schedule_build_step!(
                node,
                sched,
                scene_tree,
                bounding_radii,
                staging_radii,
                ;
                robot_radius=robot_radius,
            )
        end
    end
    # Update assembly start points so that none of the staging regions overlap
    for v in reverse(topological_sort_by_dfs(scene_tree))
        node = get_node(scene_tree,v)
        if matches_template(AssemblyNode,node)
            # Apply ring solver with child assemblies (not objects), using
            assembly = node 
            part_ids = sort(filter(k->isa(k,AssemblyID),
                collect(keys(assembly_components(node)))))
            if isempty(part_ids) 
                continue
            end
            # staging_radii
            parts = map(part_id->get_node(scene_tree,part_id), part_ids)
            staging_radius = staging_radii[node_id(node)]
            θ_des = Vector{Float64}()
            radii = Vector{Float64}()
            for (part_id,part) in zip(part_ids,parts)
                # retrieve staging config from LiftIntoPlace node
                lift_node = get_node(sched,LiftIntoPlace(part))
                tform = local_transform(start_config(lift_node))
                d = HG.project_to_2d(tform.translation)
                push!(θ_des, atan(d[2],d[1]))
                r = staging_radii[part_id]
                push!(radii, r)
            end
            # optimize placement and increase staging_radius if necessary
            θ_star, staging_radius = solve_ring_placement_problem(
                θ_des,
                radii,
                staging_radius,
                robot_radius,
                )
            # Compute staging config transforms (relative to parent assembly)
            for (i,(θ,r,part_id,part)) in enumerate(zip(θ_star,radii,part_ids,parts))
                R = staging_radius + r
                t = CoordinateTransformations.Translation(
                    R*cos(θ),
                    R*sin(θ),
                    0.0)
                tform = t ∘ identity_linear_map() # AffineMap transform
                # set transform of start node
                start_node = get_node(sched,AssemblyComplete(part))
                @info "Starting config: setting START config of $(node_id(start_node)) to $(tform)"
                set_local_transform!(start_config(start_node),tform)
                staging_radii[node_id(assembly)] = max(
                    staging_radii[node_id(assembly)], R+r)
            end
        end
    end
    # TODO store a TransformNode in ProjectComplete() (or in the schedule itself,
    # once there is a dedicated ConstructionSchedule type) so that an entire 
    # schedule can be moved anywhere. All would-be root TransormNodes will have
    # this root node as their parent, regardless of the edge structure of the 
    # schedule graph
    return sched
end

"""
    process_schedule_build_step!(sched,staging_configs,node,bounding_radii;

Select the staging configuration for all subcomponents to be added to `assembly`
during `build_step`, where `build_step::OpenBuildStep = node_val(node)`, and 
`assembly = build_step.assembly.`
Also updates `bounding_radii[node_id(assembly)]` to reflect the increasing size 
of assembly as more parts are added to it.
Updates:
- `staging_configs`
- `bounding_radii`
- the relevant `LiftIntoPlace` nodes (start_config and goal_config transforms)
"""
function process_schedule_build_step!(node,sched,scene_tree,
        bounding_radii,
        staging_radii,
        ;
        robot_radius=0.0,
    )
    open_build_step = node_val(node)
    assembly = open_build_step.assembly
    # radius of assembly at current stage
    assembly_radius = bounding_radii[node_id(assembly)]
    # optimize staging locations
    θ_des = Vector{Float64}()
    radii = Vector{Float64}()
    part_ids = sort(collect(keys(assembly_components(open_build_step))))
    tforms = map(id->assembly_components(open_build_step)[id],part_ids)
    for (part_id,tform) in zip(part_ids,tforms)
        part = get_node(scene_tree,part_id)
        d = HG.project_to_2d(tform.translation)
        push!(θ_des, atan(d[2],d[1]))
        r = get_base_geom(part,HypersphereKey()).radius
        push!(radii, r)
        # update bounding radius for next stage
        bounding_radii[node_id(assembly)] = max(
            bounding_radii[node_id(assembly)],
            norm(d) + r)
        # Set goal config of LiftIntoPlace node
        lift_node = get_node(sched,LiftIntoPlace(get_node(scene_tree,part_id)))
        @info "Staging config: setting GOAL config of $(node_id(lift_node)) to $(tform)"
        set_local_transform!(goal_config(lift_node),tform)
    end
    # optimize placement and increase assembly_radius if necessary
    θ_star, assembly_radius = solve_ring_placement_problem(
        θ_des,
        radii,
        assembly_radius,
        robot_radius,
        )
    # Compute staging config transforms (relative to parent assembly)
    for (i,(θ,r,part_id)) in enumerate(zip(θ_star,radii,part_ids))
        R = assembly_radius + r
        t = CoordinateTransformations.Translation(
            R*cos(θ),
            R*sin(θ),
            0.0)
        tform = t ∘ identity_linear_map() # AffineMap transform
        # set transform of lift node
        lift_node = get_node(sched,LiftIntoPlace(get_node(scene_tree,part_id)))
        @info "Staging config: setting START config of $(node_id(lift_node)) to $(tform)"
        set_local_transform!(start_config(lift_node),tform)
        staging_radii[node_id(assembly)] = max(
            staging_radii[node_id(assembly)], R+r)
    end
end

"""
    solve_ring_placement_problem(θ_des,R,r,rmin)

Formulate and solve a JuMP Model that encodes a ring optimization problem:
    Min_θ sum((θ .- θ_des).^2)
    s.t.  no overlapping of circles placed along ring
If there are too many circles, the problem will be infeasible. Either allow R 
to increase as a variable, or use a search to optimize R while repeating the 
optimization.
return θ_star
"""
function solve_ring_placement_problem(θ_des,r,R,rmin=0.0;
        ϵ = 1e-1, # buffer for increasing R when necessary
        weights = ones(length(θ_des)),
    )
    model = Model(HG.default_optimizer())
    set_optimizer_attributes(model,HG.default_optimizer_attributes()...)

    n = length(θ_des)
    @assert length(r) == n
    # sort θ (the unsorted vector will be returned at the end)
    idxs = sortperm(θ_des)
    reverse_idxs = collect(1:n)[idxs]
    θ_des = θ_des[idxs]
    r = r[idxs]
    # compute half widths in radians (required radial spacing between parts)
    half_widths = asin.(r ./ (r .+ R))
    # increase R and recompute half widths if ring is too small
    while sum(half_widths)*2 >= 2π
        @info "R = $R is too small----sum(half_widths)*2 = $(sum(half_widths)*2)"
        R = R*sum(half_widths)/(π) + ϵ
        half_widths = asin.(r ./ (r .+ R))
        @info "Increased R to $R. sum(half_widths)*2 = $(sum(half_widths)*2)"
    end
    @variable(model, θ[1:n])
    # Constrain order and spacing of elements of θ
    for i in 1:n-1
        @constraint(model, θ[i] + half_widths[i] <= θ[i+1] - half_widths[i+1])
    end
    # wrap-around constraint
    @constraint(model, θ[n] + half_widths[n] <= θ[1] - half_widths[1] + 2π)
    @objective(model, Min, sum(weights .* (θ .- θ_des).^2))

    optimize!(model)
    if !(primal_status(model) == MOI.FEASIBLE_POINT)
        @warn "Ring optimization failed!"
    end

    return value.(θ)[reverse_idxs], R
end


"""
    add_construction_delivery_task!(sched,args...)
    
Args:
- assembly
- goal_config
- start_config
- assigned transport unit

"""
function add_construction_delivery_task!(sched,)
end