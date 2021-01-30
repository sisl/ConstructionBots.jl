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
    entity::C
    start_config::S
    goal_config::G
end

"""
    abstract type BuildPhasePredicate <: ConstructionPredicate

References a building phase. Interface: 
- `get_assembly(::BuildPhasePredicate)` => ::AssemblyNode - the assembly to be 
modified by this build phase
- `components(::BuildPhasePredicate)` => TransformDict{Union{ObjectID,AssemblyID}} 
- the components to be added to the assembly during this phase
"""
abstract type BuildPhasePredicate <: ConstructionPredicate end

get_assembly(n::BuildPhasePredicate) = n.assembly
HierarchicalGeometry.components(n::BuildPhasePredicate) = n.components

"""
    extract_building_phase(n::BuildingStep,tree::SceneTree,id_map)

extract the assembly and set of subcomponents from a BuildingStep.
"""
function extract_building_phase(n::BuildingStep,tree::SceneTree,id_map)
    assembly_id = id_map[n.parent]
    @assert isa(assembly_id, AssemblyID)
    assembly = get_node(tree,assembly_id)
    components = typeof(components(assembly))
    for line in n.lines
        child_id = id_map[line.file]
        @assert has_component(assembly, child_id)
        components[child_id] = child_transform(assembly,child_id)
    end
    return assembly_id, components
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
    components::TransformDict{Union{AssemblyNode,ObjectNode}}
end
struct CloseBuildStep <: BuildPhasePredicate
    assembly::AssemblyNode 
    components::TransformDict{Union{AssemblyNode,ObjectNode}}
end
for T in (:OpenBuildStep,:CloseBuildStep)
    @eval begin 
        $T(n::BuildingStep,args...) = $T(extract_building_phase(n,args...)...)
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
HierarchicalGeometry.robot_team(n::ConstructionPredicate) = robot_team(entity(n))
cargo_node_type(n::TransportUnitNode) = cargo_type(n) == AsemblyNode ? AssemblyComplete : ObjectStart
cargo_node_type(n::ConstructionPredicate) = cargo_node_type(entity(n))

struct ProjectComplete <: ConstructionPredicate end

GraphUtils.required_predecessors(::ConstructionPredicate)   = Dict()
GraphUtils.required_successors(::ConstructionPredicate)     = Dict()
GraphUtils.eligible_predecessors(n::ConstructionPredicate)  = required_predecessors(n)
GraphUtils.eligible_successors(n::ConstructionPredicate)    = required_successors(n)

GraphUtils.required_predecessors(   ::RobotStart)       = Dict()
GraphUtils.required_successors(     ::RobotStart)       = Dict(RobotGo=>1)

GraphUtils.required_predecessors(   ::RobotGo)          = Dict(Union{RobotStart,DepositCargo,RobotGo}=>1)
GraphUtils.required_successors(     ::RobotGo)          = Dict()
GraphUtils.eligible_successors(     ::RobotGo)          = Dict(Union{RobotGo,FormTransportUnit}=>1)

GraphUtils.required_predecessors(  n::FormTransportUnit) = Dict(RobotGo=>length(robot_team(n)),cargo_node_type(n)=>1)
GraphUtils.required_successors(     ::FormTransportUnit) = Dict(TransportUnitGo=>1)

GraphUtils.required_predecessors(   ::TransportUnitGo)      = Dict(FormTransportUnit=>1)
GraphUtils.required_successors(     ::TransportUnitGo)      = Dict(DepositCargo=>1)

GraphUtils.required_predecessors(   ::DepositCargo)     = Dict(TransportUnitGo=>1,OpenBuildStep=>1)
GraphUtils.required_successors(    n::DepositCargo)     = Dict(LiftIntoPlace=>1,RobotGo=>length(robot_team(n)))

GraphUtils.required_predecessors(   ::LiftIntoPlace)    = Dict(DepositCargo=>1)
GraphUtils.required_successors(     ::LiftIntoPlace)    = Dict(CloseBuildStep=>1)

GraphUtils.required_predecessors(  n::CloseBuildStep)   = Dict(LiftIntoPlace=>num_components(n))
GraphUtils.required_successors(     ::CloseBuildStep)   = Dict(Union{OpenBuildStep,AssemblyComplete}=>1)

GraphUtils.required_predecessors(   ::OpenBuildStep)    = Dict(Union{AssemblyStart,CloseBuildStep}=>1)
GraphUtils.required_successors(     ::OpenBuildStep)    = Dict(DepositCargo=>num_components(n))

GraphUtils.required_predecessors(   ::ObjectStart)      = Dict()
GraphUtils.required_successors(     ::ObjectStart)      = Dict(FormTransportUnit=>1)

GraphUtils.required_predecessors(  n::AssemblyComplete) = Dict(Union{ObjectStart,LiftIntoPlace}=>num_components(n))
GraphUtils.required_successors(     ::AssemblyComplete) = Dict(Union{FormTransportUnit,ProjectComplete}=>1)

"""
    struct TemplatedID{T} <: AbstractID 

A simple way to dispatch by Node type.
"""
struct TemplatedID{T} <: AbstractID 
    id::Int
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
    # :OpenBuildStep,
    # :CloseBuildStep,
    :LiftIntoPlace,
)
    @eval begin
        GraphUtils.node_id(n::$T) = TemplatedID{$T}(get_id(node_id(entity(n))))
        # GraphUtils.add_node!(g::AbstractCustomNGraph, n::$T) = add_node!(g,n,node_id(n))
    end
end
GraphUtils.add_node!(g::AbstractCustomNGraph, n::ConstructionPredicate) = add_node!(g,n,node_id(n))
# ProjectComplete must be a singleton
GraphUtils.node_id(::ProjectComplete) = TemplatedID{ProjectComplete}(1)

"""
    construct_partial_construction_schedule(sched,args...)

Constructs a schedule that does not yet reflect the "staging plan".
"""
function construct_partial_construction_schedule(
        mpd_model,
        model_spec,
        id_map=build_id_map(mpd_model,model_spec)
    )
    sched = CustomNDiGraph{ConstructionPredicate,AbstractID}()
    parent_map = backup_descendants(spec,n->matches_template(SubModelPlan,n))
    for v in reverse(topological_sort_by_dfs(model_spec))
        node = get_node(model_spec,v)
        if matches_template(SubModelPlan,node)
            if is_terminal_node(model_spec,v)
                # ProjectComplete
                add_node!(sched,ProjectComplete(),node_id(ProjectComplete()))
            end
            # AssemblyComplete
            # add_node!(sched,AssemblyComplete())
            for step in node_val(node).steps
                # CloseBuildStep
                for line in step.lines
                    # LiftIntoPlace
                    # DepositCargo
                    # TransportUnitGo
                    # FormTransportUnit
                end
                # OpenBuildStep
            end
        end
    end
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