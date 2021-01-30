"""
    abstract type ConstructionPredicate

Abstract type for ConstructionSchedule nodes.
"""
abstract type ConstructionPredicate end
# abstract type ConstructionInitialCondition <: ConstructionPredicate end
# abstract type ConstructionAction <: ConstructionPredicate end
# abstract type ConstructionRobotAction <: ConstructionAction end
# abstract type ConstructionTeamAction <: ConstructionAction end

struct EntityStart{T,S} <: ConstructionPredicate
    entity::T
    config::S
end
const RobotStart{T}         = EntityStart{RobotNode,T}
const ObjectStart{T}        = EntityStart{ObjectNode,T}
const AssemblyComplete{T}   = EntityStart{AssemblyNode,T}
for N in (:ObjectStart,:RobotStart,:AssemblyComplete)
    @eval $N(n,t::T) where {T} = $N{T}(n,t)
end

struct EntityGo{T,S}
    entity::T
    start_config::S
    goal_config::S
end
const RobotGo{T}            = EntityGo{RobotNode,T} 
const TransportGo{T}        = EntityGo{TransportUnitNode,T} 
const LiftIntoPlace{T}      = EntityGo{AssemblyNode,T} 

# struct OpenPhase{T} <: ConstructionPredicate
#     phase::T
# end
# struct ClosePhase{T} <: ConstructionPredicate
#     phase::T
# end

struct OpenBuildStep <: ConstructionPredicate 
    step::BuildingStep
end
struct CloseBuildStep <: ConstructionPredicate 
    step::BuildingStep
end
struct AssemblyBegin <: ConstructionPredicate 
    model::SubModelPlan
end

# struct PartMounted{T} <: ConstructionPredicate 
#     entity::AssemblyNode
#     goal_config::T
# end
struct DepositAssembly{T} <: ConstructionPredicate 
    entity::TransportUnitNode
    start_config::T # of assembly or transport unit?
    goal_config::T
end
struct FormTransportUnit{T} <: ConstructionPredicate 
    entity::TransportUnitNode
    start_config::T
    goal_config::T
end
struct ProjectComplete <: ConstructionPredicate end

GraphUtils.required_predecessors(::ConstructionPredicate)   = Dict()
GraphUtils.required_successors(::ConstructionPredicate)     = Dict()
GraphUtils.eligible_predecessors(n::ConstructionPredicate)  = required_predecessors(n)
GraphUtils.eligible_successors(n::ConstructionPredicate)    = required_successors(n)

GraphUtils.required_predecessors(   ::RobotStart)       = Dict()
GraphUtils.required_successors(     ::RobotStart)       = Dict(RobotGo=>1)

GraphUtils.required_predecessors(   ::RobotGo)          = Dict(Union{RobotStart,DepositAssembly,RobotGo}=>1)
GraphUtils.required_successors(     ::RobotGo)          = Dict()
GraphUtils.eligible_successors(     ::RobotGo)          = Dict(Union{RobotGo,FormTransportUnit}=>1)

GraphUtils.required_predecessors(   ::TransportGo)      = Dict(FormTransportUnit=>1)
GraphUtils.required_successors(     ::TransportGo)      = Dict(DepositAssembly=>1)

GraphUtils.required_predecessors(   ::DepositAssembly)  = Dict(TransportGo=>1,OpenBuildStep=>1)
GraphUtils.required_successors(    n::DepositAssembly)  = Dict(LiftIntoPlace=>1,RobotGo=>length(robot_team(n)))

GraphUtils.required_predecessors(   ::LiftIntoPlace)    = Dict(DepositAssembly=>1)
GraphUtils.required_successors(     ::LiftIntoPlace)    = Dict(CloseBuildStep=>1)

# GraphUtils.required_predecessors(   ::PartMounted)      = Dict(LiftIntoPlace=>1)
# GraphUtils.required_successors(     ::PartMounted)      = Dict(CloseBuildStep=>1)

GraphUtils.required_predecessors(  n::CloseBuildStep)   = Dict(LiftIntoPlace=>n_lines(n.step))
GraphUtils.required_successors(     ::CloseBuildStep)   = Dict(Union{OpenBuildStep,AssemblyComplete}=>1)

GraphUtils.required_predecessors(   ::OpenBuildStep)    = Dict(Union{AssemblyBegin,CloseBuildStep}=>1)
GraphUtils.required_successors(     ::OpenBuildStep)    = Dict(DepositAssembly=>n_lines(n.step))

GraphUtils.required_predecessors(   ::ObjectStart)      = Dict()
GraphUtils.required_successors(     ::ObjectStart)      = Dict(AssemblyComplete=>1)

GraphUtils.required_predecessors(  n::AssemblyComplete) = Dict(Union{ObjectStart,LiftIntoPlace}=>length(components(n)))
GraphUtils.required_successors(     ::AssemblyComplete) = Dict(Union{FormTransportUnit,ProjectComplete}=>1)

"""
    construct_partial_construction_schedule(sched,args...)

Constructs a schedule that does not yet reflect the "staging plan".
"""
function construct_partial_construction_schedule(
        mpd_model,
        model_spec,
        id_map=build_id_map(mpd_model,model_spec)
    )
    sched = NGraph{DiGraph,ConstructionPredicate,AbstractID}()
    for v in reverse(topological_sort_by_dfs(model_spec))
        node = get_node(model_spec,v)
        
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