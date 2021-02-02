export  
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

abstract type EntityConfigPredicate{E,T} <: ConstructionPredicate end
start_config(n::EntityConfigPredicate)   = n.config
goal_config(n::EntityConfigPredicate)    = n.config

struct RobotStart{T} <: EntityConfigPredicate{RobotNode,T}
    entity::RobotNode
    config::T
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

for T in (:RobotStart,:ObjectStart,:AssemblyComplete,:AssemblyStart)
    @eval $T(n::SceneNode) = $T(n,global_transform(n))
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
# for T in (
#     :TransportUnitGo,
#     :FormTransportUnit,
#     :DepositCargo
# )
#     @eval begin
#         function GraphUtils.node_id(n::$T)
#             TemplatedID{Tuple{$T,cargo_type(entity(n))}}(get_id(node_id(entity(n))))
#         end
#     end
# end

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

GraphUtils.required_predecessors(   ::ProjectComplete)  = Dict(AssemblyComplete=>1)
GraphUtils.required_successors(     ::ProjectComplete)  = Dict()

function GraphUtils.validate_edge(a::ConstructionPredicate,b::ConstructionPredicate)
    valid = false
    for (key,val) in required_successors(a)
        if matches_template(key,b)
            valid = true
        end
    end
    for (key,val) in required_predecessors(b)
        if matches_template(key,a) && val >= 1
            valid = valid && true
            return valid
        end
    end
    return false
end

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
function populate_schedule_build_step!(sched,cb,step_node,model_spec,scene_tree,id_map;
        connect_to_sub_assemblies=true,
    )
    # OpenBuildStep
    ob = add_node!(sched,OpenBuildStep(node_val(cb)))
    for child_id in get_build_step_components(model_spec,id_map,step_node)
        cargo = get_node(scene_tree,child_id)
        @assert isa(cargo,Union{AssemblyNode,ObjectNode})
        # LiftIntoPlace
        l = add_node!(sched,    LiftIntoPlace(cargo)) 
        add_edge!(sched,l,cb) # LiftIntoPlace => CloseBuildStep
        # DepositCargo
        d = add_node!(sched,    DepositCargo(TransportUnitNode(cargo)))
        add_edge!(sched,d,l) # DepositCargo => LiftIntoPlace
        add_edge!(sched,ob,d) # OpenBuildStep => DepositCargo
        # TransportUnitGo
        tgo = add_node!(sched,  TransportUnitGo(entity(node_val(d))))
        add_edge!(sched,tgo,d) # TransportUnitGo => DepositCargo
        # FormTransportUnit
        f = add_node!(sched,    FormTransportUnit(entity(node_val(d))))
        add_edge!(sched,f,tgo) # FormTransportUnit => TransportUnitGo
        if isa(cargo,AssemblyNode) connect_to_sub_assemblies
            add_edge!(sched,AssemblyComplete(cargo),f) # AssemblyComplete => FormTransportUnit 
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
    sa = AssemblyStart(parent_assembly)
    spec_node = get_node(model_spec, id_map[node_id(parent_assembly)])
    step_node = get_previous_build_step(model_spec,spec_node;skip_first=true)
    while !(step_node === nothing)
        # CloseBuildStep
        cb = add_node!(sched,CloseBuildStep(step_node,scene_tree,model_spec,id_map))
        add_edge!(sched,cb,parent) # CloseBuildStep => AssemblyComplete / OpenBuildStep
        ob = populate_schedule_build_step!(sched,cb,step_node,model_spec,scene_tree,id_map)
        parent = ob
        step_node = get_previous_build_step(model_spec,step_node;skip_first=true)
    end
    add_edge!(sched,sa,parent) # AssemblyStart => OpenBuildStep
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
            a = add_node!(sched,AssemblyComplete(assembly))
            if is_terminal_node(scene_tree,assembly)
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