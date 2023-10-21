# # module PlanningPredicates
# #
# # using Parameters

export
    AbstractPlanningPredicate,
    AbstractRobotAction,
	AbstractSingleRobotAction,
    get_object_id,
    get_location_id,
    get_location_ids,
	get_robot_id,
	get_initial_location_id,
	get_destination_location_id,
	robot_type,
	graph_key,
	has_object_id,
	check_object_id,
	has_robot_id,
	get_default_robot_id,
	robot_ids_match,
	get_default_initial_location_id,
	get_default_final_location_id,
	replace_robot_id,
	replace_destination,
	replace_initial_location,
	resources_reserved,
	align_with_predecessor,
	align_with_successor,
	split_node,
	matches_node_type,
	id_type,
	title_string


abstract type AbstractPlanningPredicate end
abstract type AbstractRobotAction{R<:AbstractRobotType} <: AbstractPlanningPredicate end
abstract type AbstractSingleRobotAction{R<:AbstractRobotType} <: AbstractRobotAction{R} end
abstract type AbstractTeamRobotAction{R<:AbstractRobotType} <: AbstractRobotAction{R} end

Base.copy(p::AbstractPlanningPredicate) = deepcopy(p)

function get_object_id end
function get_robot_id end
function get_initial_location_id end
function get_destination_location_id end
function get_location_id end
function get_location_ids end

robot_type(a::AbstractRobotAction{R}) where {R} = R
robot_type(a) = Nothing
graph_key() = Symbol(DefaultRobotType)
function graph_key(a)
	if robot_type(a) == Nothing
		return graph_key()
	else
		return Symbol(robot_type(a))
	end
end

has_object_id(a) = false

"""
    Check if a node is associated with objectid
"""
function check_object_id(node,o)
    if has_object_id(node)
        if get_object_id(node) == o
            return true
        end
    end
    return false
end

has_robot_id(a) = robot_type(a) == Nothing ? false : true
get_default_robot_id(a) = has_robot_id(a) ? get_robot_id(a) : RobotID()
get_default_initial_location_id(a) = has_robot_id(a) ? get_initial_location_id(a) : LocationID()
get_default_destination_location_id(a) = has_robot_id(a) ? get_destination_location_id(a) : LocationID()

"""
	robot_ids_match(node,node2)

Checks if robot_ids match between the nodes
"""
function robot_ids_match(node,node2)
	if has_robot_id(node) && has_robot_id(node2)
		if get_id(get_robot_id(node)) != -1 && get_id(get_robot_id(node2)) != -1
			status = (get_robot_id(node) == get_robot_id(node2)) ? true : false
			return status
		end
	end
	return true
end

function replace_robot_id end
function replace_destination end
function replace_initial_location end

"""
	resources_reserved(node)

Identifies the resources reserved by a particular `node` for its duration.
For example, `resources_reserved(node::COLLECT) = AbstractID[get_location_id(node)]`
"""
resources_reserved(node) = AbstractID[]

"""
	align_with_predecessor(node,succ)

Modifies a node to match the information encoded by its predecessor. This is
how e.g., robot ids are propagated through an existing operating schedule
after assignments (or re-assignments) have been made.
"""
align_with_predecessor(node,pred) = node

CRCBS.is_valid(id::A) where {A<:AbstractID} = valid_id(id) #get_id(id) != -1
first_valid(a,b) = CRCBS.is_valid(a) ? a : b

"""
	align_with_successor(node,succ)

Modifies a node to match the information encoded by its successor. This is
how e.g., robot ids are propagated through an existing operating schedule
after assignments (or re-assignments) have been made.
"""
align_with_successor(node,succ) = node

"""
	split_node(node::N,x::LocationID)

Creates two new nodes of type `N`, where the destination of the first node
and the starting location of the second node are both set to `x`.
"""
function split_node end

"""
    matches_node_type(::A,::Type{B}) where {A<:AbstractPlanningPredicate,B}

Returns true if {A <: B}
"""
matches_node_type(::A,::Type{B}) where {A,B} = A <: B

title_string(a) = string(a)

const predicate_accessor_interface = [
	:get_initial_location_id,
	:get_destination_location_id,
	:get_robot_id,
	:get_default_robot_id,
	:get_default_initial_location_id,
	:get_default_final_location_id,
	:get_object_id,
	:has_object_id,
	:has_robot_id,
	:sub_nodes,
	:(GraphUtils.required_successors),
	:(GraphUtils.required_predecessors),
	:(GraphUtils.eligible_successors),
	:(GraphUtils.eligible_predecessors),
	:(GraphUtils.num_required_successors),
	:(GraphUtils.num_required_predecessors),
	:(GraphUtils.num_eligible_successors),
	:(GraphUtils.num_eligible_predecessors),
	:resources_reserved,
	:id_type,
	:title_string
]
const predicate_comparison_interface = [
	:(GraphUtils.matches_template),
	:(GraphUtils.validate_edge),
]

abstract type PredicateTrait end
struct HasObject <: PredicateTrait end
struct HasRobot <: PredicateTrait end

# single robot actions: BOT_GO, BOT_COLLECT, BOT_CARRY, BOT_DEPOSIT
# initial condition preds: OBJECT_AT, ROBOT_AT
# event preds: Operation

@with_kw struct LargeObjectDef
	footprint::Matrix{Bool} = ones(Int,1,1)
	origin::Tuple{Int,Int} 	= (0,0)
	carrying_positions::Vector{Tuple{Int,Int}} = [(0,0)]
end
struct LARGE_OBJECT_AT <: AbstractPlanningPredicate
	o::ObjectID
	x::LocationID
	def::LargeObjectDef
end
get_object_id(pred::LARGE_OBJECT_AT) = pred.o
GraphUtils.node_id(n::LARGE_OBJECT_AT) = get_object_id(n)
get_location_id(pred::LARGE_OBJECT_AT) = pred.x
get_initial_location_id(pred::LARGE_OBJECT_AT) = pred.x
get_destination_location_id(pred::LARGE_OBJECT_AT) = pred.x
LARGE_OBJECT_AT(o::ObjectID,x::LocationID) = LARGE_OBJECT_AT(o,x,LargeObjectDef())

export
	OBJECT_AT,
	BOT_AT,
    ROBOT_AT

"""
	OBJECT_AT <: AbstractPlanningPredicate

Encodes the location and shape of the object with id `o`
"""
struct OBJECT_AT <: AbstractPlanningPredicate
    o::ObjectID
    x::Vector{LocationID} # can occupy multiple locations
	# n::Int # number of robots required for transport
	shape::Tuple{Int,Int}
end
OBJECT_AT(o,x) = OBJECT_AT(o,x,(1,1))
OBJECT_AT(o,x::Union{Int,LocationID},args...) = OBJECT_AT(o,[x],args...)
get_object_id(pred::OBJECT_AT) = pred.o
get_location_id(pred::OBJECT_AT) = pred.x[1]
get_location_ids(pred::OBJECT_AT) = pred.x


"""
	BOT_AT <: AbstractPlanningPredicate

Encodes the location of robot with id `r`
"""
struct BOT_AT{R<:AbstractRobotType} <: AbstractPlanningPredicate
    r::BotID{R}
    x::LocationID
end
robot_type(id::BotID{R}) where {R} = R
robot_type(a::BOT_AT{R}) where {R} = R
get_location_id(pred::BOT_AT) 	= pred.x
get_robot_id(pred::BOT_AT) 		= pred.r

const ROBOT_AT = BOT_AT{DeliveryBot}

export
    Operation,
	get_input_ids,
	get_output_ids,
	get_operation_id,
    preconditions,
	postconditions,
	set_precondition!,
	set_postcondition!,
	get_dropoff,
	get_dropoffs,
    duration

"""
	Operation <: AbstractPlanningPredicate

A manufacturing operation.
"""
@with_kw struct Operation <: AbstractPlanningPredicate
    pre::Dict{ObjectID,OBJECT_AT}     = Dict{ObjectID,OBJECT_AT}()
    post::Dict{ObjectID,OBJECT_AT}    = Dict{ObjectID,OBJECT_AT}()
    Δt::Int 					= 0
    station_id::LocationID   	= LocationID(-1)
    id::OperationID         	= get_unique_id(OperationID) #OperationID(-1)
end
get_location_id(op::Operation) = op.station_id
get_operation_id(op::Operation) = op.id
duration(op::Operation) = op.Δt
preconditions(op::Operation) = op.pre
postconditions(op::Operation) = op.post
get_condition(op::Operation,o::ObjectID,dict) = get(dict,o,OBJECT_AT(-1,-1))
function get_precondition(op::Operation,o::ObjectID)
	@assert haskey(preconditions(op),o) "Object $o is not an input to operation $(get_operation_id(op))"
	return preconditions(op)[o]
end
function get_postcondition(op::Operation,o::ObjectID)
	@assert haskey(posconditions(op),o) "Object $o is not an output of operation $(get_operation_id(op))"
	return postconditions(op)[o]
end
function set_precondition!(op::Operation,o::ObjectID,pred::OBJECT_AT)
	preconditions(op)[o] = pred
end
function set_postcondition!(op::Operation,o::ObjectID,pred::OBJECT_AT)
	postconditions(op)[o] = pred
end
for op in (:set_precondition!,:set_postcondition!)
	@eval $op(o::Operation,pred::OBJECT_AT) = $op(o,get_object_id(pred),pred)
	@eval begin
		function $op(o::Operation,preds::Vector{OBJECT_AT})
			for pred in preds
				$op(o,pred)
			end
			return o
		end
	end
end
get_input_ids(op::Operation) = sort(collect(keys(preconditions(op))))
get_output_ids(op::Operation) = sort(collect(keys(postconditions(op))))
GraphUtils.get_id(op::Operation) = get_id(op.id)

get_dropoff(op::Operation,o::ObjectID) = get_location_id(get_precondition(op,o))
get_dropoffs(op::Operation,o::ObjectID) = get_location_ids(get_precondition(op,o))

id_type(::BOT_AT{R}) where {R} = BotID{R}
id_type(::AbstractRobotAction{R}) where {R} = ActionID
id_type(::OBJECT_AT) = ObjectID
id_type(::Operation) = OperationID

export
	BOT_GO,BOT_CARRY,BOT_COLLECT,BOT_DEPOSIT,
    GO,COLLECT,CARRY,DEPOSIT

"""
	BOT_GO <: AbstractRobotAction

Encodes the event "robot `r` goes from `x1` to `x2`"
"""
@with_kw struct BOT_GO{R} <: AbstractSingleRobotAction{R} # go to position x
    r::BotID{R} 		= BotID{R}()
    x1::LocationID	= LocationID()
    x2::LocationID	= LocationID()
end
const GO = BOT_GO{DeliveryBot}

"""
	BOT_CARRY <: AbstractRobotAction

Encodes the event "robot `r` carries object `o` from `x1` to `x2`"
"""
@with_kw struct BOT_CARRY{R} <: AbstractSingleRobotAction{R} # carry object o to position x
    r::BotID{R}		= BotID{R}()
    o::ObjectID		= ObjectID()
    x1::LocationID	= LocationID()
    x2::LocationID	= LocationID()
end
const CARRY = BOT_CARRY{DeliveryBot}

"""
	BOT_COLLECT <: AbstractRobotAction

Encodes the event "robot `r` collects object `o` from `x`
"""
@with_kw struct BOT_COLLECT{R} <: AbstractSingleRobotAction{R}
    r::BotID{R}     = BotID{R}()
    o::ObjectID 	= ObjectID()
    x::LocationID 	= LocationID()
end
const COLLECT = BOT_COLLECT{DeliveryBot}

"""
	BOT_DEPOSIT <: AbstractRobotAction

Encodes the event "robot `r` collects object `o` from `x`
"""
@with_kw struct BOT_DEPOSIT{R} <: AbstractSingleRobotAction{R}
    r::BotID{R}     = BotID{R}()
    o::ObjectID 	= ObjectID()
    x::LocationID 	= LocationID()
end
const DEPOSIT = BOT_DEPOSIT{DeliveryBot}

@with_kw struct COLLABORATIVE_TRANSPORT_ACTION{R,A<:AbstractRobotAction{R}} <: AbstractTeamRobotAction{R}
	action::A 				   = A()
	object_def::LargeObjectDef = LargeObjectDef() # defines object shape, carrying config
	start_positions::Vector{Pair{BotID{R},LocationID}} = Vector{Pair{BotID{R},LocationID}}()
	goal_positions::Vector{Pair{BotID{R},LocationID}}	= Vector{Pair{BotID{R},LocationID}}()
end
function init_collaborative_action(env::GridFactoryEnvironment, action::A, def::LargeObjectDef) where {R<:AbstractRobotType,A<:AbstractRobotAction{R}}
	x0 = get_initial_location_id(action)
	xF = get_destination_location_id(action)
	starts = Vector{Pair{BotID{R},LocationID}}()
	goals = Vector{Pair{BotID{R},LocationID}}()
	for p in def.carrying_positions
		id = get_unique_invalid_id(RobotID)
		offset = p .- def.origin
		push!(starts,id=>GraphUtils.idx_from_offset(env, get_id(x0), offset))
		push!(goals,id=>GraphUtils.idx_from_offset(env, get_id(xF), offset))
	end
	COLLABORATIVE_TRANSPORT_ACTION{R,A}(action,def,starts,goals)
end
# sub_nodes(n) = [n]
# sub_nodes(n::COLLABORATIVE_TRANSPORT_ACTION) = n.instructions
# team_configuration(n) = (1,1)
# team_configuration(n::COLLABORATIVE_TRANSPORT_ACTION) = n.shape
# team_action_type(n::COLLABORATIVE_TRANSPORT_ACTION{R,A}) where {R,A} = A

export
	CleanUpBot,
	CleanUpBotID,
	CUB_AT,
	CUB_GO,
	CLEAN_UP

"""
	CleanUpBot <: AbstractRobotType

A robot type for picking up dropped objects, cleaning up spills, and taking
care of dead robots
"""
struct CleanUpBot <: AbstractRobotType end
const CleanUpBotID = BotID{CleanUpBot}

export
	CUB_AT,
	CUB_GO,
	CUB_COLLECT,
	CUB_CARRY,
	CUB_DEPOSIT

const CUB_AT = BOT_AT{CleanUpBot}
const CUB_GO = BOT_GO{CleanUpBot}
const CUB_COLLECT = BOT_COLLECT{CleanUpBot}
const CUB_CARRY = BOT_CARRY{CleanUpBot}
const CUB_DEPOSIT = BOT_DEPOSIT{CleanUpBot}

"""
	CLEAN_UP <: AbstractRobotAction

Encodes the event "robot `r` cleans up locations vtxs`
"""
@with_kw struct CLEAN_UP <: AbstractSingleRobotAction{CleanUpBot}
	r::BotID{CleanUpBot} = BotID{CleanUpBot}()
	vtxs::Vector{LocationID} = Vector{LocationID}()
end

"""
	UNDERTAKE <: AbstractRobotAction{CleanUpBot}

Encodes the task of collecting, carrying, and depositing a dead robot
"""
@with_kw struct UNDERTAKE <: AbstractSingleRobotAction{CleanUpBot}
	r::BotID{CleanUpBot}	= BotID{CleanUpBot}()
	dr::BotID 				= RobotID()
	x1::LocationID 			= LocationID()
	x2::LocationID			= LocationID()
end

export
	TEAM_ACTION,
	sub_nodes,
	team_configuration,
	robot_type,
	team_action_type

"""
	TEAM_ACTION{R<:AbstractRobotType,A<:AbstractRobotAction{R}}

For collaborative tasks.

[GO, ...] -> TEAM_COLLECT -> TEAM_CARRY -> TEAM_DEPOSIT -> [GO, ...]
"""
@with_kw struct TEAM_ACTION{R,A<:AbstractRobotAction{R}} <: AbstractTeamRobotAction{R}
    instructions::Vector{A} = Vector{A}()
	shape::Tuple{Int,Int} 	= (1,1)
    n::Int 					= length(instructions) # number of robots
    # config::Matrix{Int} = ones(n) # defines configuration of agents relative to each other during collaborative task
end
sub_nodes(n) = [n]
sub_nodes(n::TEAM_ACTION) = n.instructions
team_configuration(n) = (1,1)
team_configuration(n::TEAM_ACTION) = n.shape
team_action_type(n::TEAM_ACTION{R,A}) where {R,A} = A

export
	TEAM_GO,
	TEAM_COLLECT,
	TEAM_CARRY,
	TEAM_DEPOSIT

const TEAM_GO= TEAM_ACTION{DeliveryBot,GO}
const TEAM_COLLECT= TEAM_ACTION{DeliveryBot,COLLECT}
const TEAM_CARRY= TEAM_ACTION{DeliveryBot,CARRY}
const TEAM_DEPOSIT= TEAM_ACTION{DeliveryBot,DEPOSIT}

get_location_id(a::A) where {A<:Union{BOT_COLLECT,BOT_DEPOSIT}}             				= a.x
get_initial_location_id(a::A) where {A<:Union{BOT_GO,BOT_CARRY}}        					= a.x1
get_destination_location_id(a::A) where {A<:Union{BOT_GO,BOT_CARRY}}    					= a.x2
get_initial_location_id(a::A) where {A<:Union{BOT_COLLECT,BOT_DEPOSIT,BOT_AT,OBJECT_AT}}    = get_location_id(a)
get_destination_location_id(a::A) where {A<:Union{BOT_COLLECT,BOT_DEPOSIT,BOT_AT,OBJECT_AT}}= get_location_id(a)
get_object_id(a::A) where {A<:Union{BOT_CARRY,BOT_COLLECT,BOT_DEPOSIT}}         			= a.o
get_robot_id(a::A) where {A<:AbstractRobotAction} 											= a.r

has_object_id(a::Union{OBJECT_AT,BOT_COLLECT,BOT_CARRY,BOT_DEPOSIT}) = true
has_object_id(a::TEAM_ACTION{R,A} where {R,A<:Union{BOT_COLLECT,BOT_CARRY,BOT_DEPOSIT}}) = true

for T in [:BOT_AT,:BOT_GO,:BOT_COLLECT,:BOT_CARRY,:BOT_DEPOSIT]
	@eval replace_robot_id(node::$T,id) = $T(node,r=id)
end
function replace_robot_id(node::A,id) where {A<:TEAM_ACTION}
	return A(node,instructions=map(n->replace_robot_id(n,id), sub_nodes(node)))
end
for T in [:BOT_GO,:BOT_CARRY]
	@eval replace_destination(node::$T,id) = $T(node,x2=id)
	@eval replace_initial_location(node::$T,id) = $T(node,x1=id)
end
for T in [:BOT_AT,:BOT_COLLECT,:BOT_DEPOSIT]
	@eval replace_destination(node::$T,id) = $T(node,x=id)
	@eval replace_initial_location(node::$T,id) = $T(node,x=id)
end

split_node(node::BOT_GO,x::LocationID) = BOT_GO(node, x2=x), BOT_GO(node, x1=x)
split_node(node::BOT_CARRY,x::LocationID) = BOT_CARRY(node, x2=x), BOT_CARRY(node, x1=x)
split_node(node::BOT_COLLECT,x::LocationID) = BOT_COLLECT(node, x=x), BOT_COLLECT(node, x=x)
split_node(node::BOT_DEPOSIT,x::LocationID) = BOT_DEPOSIT(node, x=x), BOT_DEPOSIT(node, x=x)

get_object_id(a::TEAM_ACTION) = get_object_id(get(sub_nodes(a),1,OBJECT_AT(-1,-1)))

GraphUtils.required_predecessors(node::BOT_GO{R}) where {R} 		= Dict((BOT_GO{R},BOT_AT{R},BOT_DEPOSIT{R},TEAM_ACTION{R,BOT_DEPOSIT{R}})=>1)
GraphUtils.required_successors(node::BOT_GO{R}) where {R}       	= Dict()
GraphUtils.required_predecessors(node::BOT_COLLECT{R}) where {R} 	= Dict(OBJECT_AT=>1,BOT_GO{R}=>1)
GraphUtils.required_successors(node::BOT_COLLECT{R}) where {R} 	= Dict(BOT_CARRY{R}=>1)
GraphUtils.required_predecessors(node::BOT_CARRY{R}) where {R} 	= Dict(BOT_COLLECT{R}=>1)
GraphUtils.required_successors(node::BOT_CARRY{R}) where {R} 		= Dict(BOT_DEPOSIT{R}=>1)
GraphUtils.required_predecessors(node::BOT_DEPOSIT{R}) where {R} 	= Dict(BOT_CARRY{R}=>1)
GraphUtils.required_successors(node::BOT_DEPOSIT{R}) where {R} 	= Dict(Operation=>1,BOT_GO{R}=>1)
GraphUtils.required_predecessors(node::Operation)  				= Dict((BOT_DEPOSIT{DeliveryBot},OBJECT_AT)=>length(preconditions(node)))
GraphUtils.required_successors(node::Operation)    				= Dict(OBJECT_AT=>length(postconditions(node)))
GraphUtils.required_predecessors(node::OBJECT_AT)  				= Dict()
GraphUtils.required_successors(node::OBJECT_AT)    				= Dict(BOT_COLLECT{DeliveryBot}=>1)
GraphUtils.required_predecessors(node::BOT_AT{R}) where {R}   		= Dict()
GraphUtils.required_successors(node::BOT_AT{R}) where {R}     		= Dict(BOT_GO{R}=>1)

GraphUtils.required_predecessors(node::TEAM_ACTION{R,GO}) where {R} 		= Dict(GO=>length(node.instructions))
GraphUtils.required_predecessors(node::TEAM_ACTION{R,COLLECT}) where {R} 	= Dict(GO=>length(node.instructions),OBJECT_AT=>1)
GraphUtils.required_predecessors(node::TEAM_ACTION{R,CARRY}) where {R} 	= Dict(TEAM_ACTION{R,COLLECT}=>1)
GraphUtils.required_predecessors(node::TEAM_ACTION{R,DEPOSIT}) where {R} 	= Dict(TEAM_ACTION{R,CARRY}=>1)

GraphUtils.required_successors(node::TEAM_ACTION{R,GO}) where {R}        	= Dict(TEAM_ACTION{R,COLLECT}=>1)
GraphUtils.required_successors(node::TEAM_ACTION{R,COLLECT}) where {R}   	= Dict(TEAM_ACTION{R,CARRY}=>1)
GraphUtils.required_successors(node::TEAM_ACTION{R,CARRY}) where {R}     	= Dict(TEAM_ACTION{R,DEPOSIT}=>1)
GraphUtils.required_successors(node::TEAM_ACTION{R,DEPOSIT}) where {R}   	= Dict(GO=>length(node.instructions),Operation=>1)

GraphUtils.eligible_predecessors(node) 			= required_predecessors(node)
GraphUtils.eligible_successors(node) 				= required_successors(node)
GraphUtils.eligible_successors(node::BOT_GO{R}) where {R} = Dict((BOT_GO{R},TEAM_ACTION{R,BOT_COLLECT{R}},TEAM_ACTION{R,BOT_GO{R}},BOT_COLLECT{R})=>1)
GraphUtils.eligible_predecessors(node::OBJECT_AT)  = Dict(Operation=>1)


resources_reserved(node::BOT_COLLECT)	= AbstractID[get_location_id(node)]
resources_reserved(node::BOT_DEPOSIT)	= AbstractID[get_location_id(node)]
resources_reserved(node::TEAM_ACTION)	= union(map(pred->resources_reserved(pred), node.instructions)...)

align_with_predecessor(graph,node,pred) = align_with_predecessor(node,pred)
align_with_successor(graph,node,pred) 	= align_with_successor(node,pred)

align_with_predecessor(node::BOT_GO,pred::BOT_AT)			= BOT_GO(first_valid(node.r,pred.r), first_valid(node.x1,pred.x), node.x2)
align_with_predecessor(node::BOT_GO,pred::BOT_GO)			= BOT_GO(first_valid(node.r,pred.r), first_valid(node.x1,pred.x2), node.x2)
align_with_predecessor(node::BOT_GO,pred::BOT_DEPOSIT)		= BOT_GO(first_valid(node.r,pred.r), first_valid(node.x1,pred.x), node.x2)
align_with_predecessor(node::BOT_COLLECT,pred::OBJECT_AT)	= BOT_COLLECT(node.r, first_valid(node.o,pred.o), node.x) # NOTE: Because the object could occupy multiple vertices, we do not want to dispatch first_valid between BOT_COLLECT.x and OBJECT_AT.x
align_with_predecessor(node::BOT_COLLECT,pred::BOT_GO)		= BOT_COLLECT(first_valid(node.r,pred.r), node.o, first_valid(node.x,pred.x2))
align_with_predecessor(node::BOT_CARRY,pred::BOT_COLLECT)	= BOT_CARRY(first_valid(node.r,pred.r), first_valid(node.o,pred.o), first_valid(node.x1,pred.x), node.x2)
align_with_predecessor(node::BOT_CARRY,pred::BOT_CARRY)		= BOT_CARRY(first_valid(node.r,pred.r), first_valid(node.o,pred.o), first_valid(node.x1,pred.x2), node.x2)
align_with_predecessor(node::BOT_DEPOSIT,pred::BOT_CARRY)	= BOT_DEPOSIT(first_valid(node.r,pred.r), first_valid(node.o,pred.o), first_valid(node.x,pred.x2))

# NOTE job shop constraints were wreaking havoc with id propagation between BOT_COLLECT nodes and BOT_DEPOSIT nodes! Hence the alignment functions have been removed
# align_with_predecessor(node::BOT_COLLECT,pred::BOT_COLLECT) 	= BOT_COLLECT(node.r, node.o, first_valid(node.x,pred.x))
# align_with_predecessor(node::BOT_DEPOSIT,pred::BOT_DEPOSIT) 	= BOT_DEPOSIT(node.r, node.o, first_valid(node.x,pred.x))

function align_with_predecessor(node::TEAM_ACTION,pred)
	for i in 1:length(node.instructions)
		p = node.instructions[i]
		if get_destination_location_id(pred) == get_initial_location_id(p)
			node.instructions[i] = align_with_predecessor(p,pred)
		end
	end
	node
end
function align_with_predecessor(node::AbstractRobotAction,pred::TEAM_ACTION)
	for i in 1:length(pred.instructions)
		p = pred.instructions[i]
		if get_destination_location_id(p) == get_initial_location_id(node)
			return align_with_predecessor(node,p)
		end
	end
	return node
end
function align_with_predecessor(node::TEAM_ACTION,pred::TEAM_ACTION)
	for i in 1:length(node.instructions)
		p = node.instructions[i]
		for (j,pj) in enumerate(pred.instructions)
			if get_destination_location_id(pj) == get_initial_location_id(p)
				@assert(i == j, "TEAM_ACTION indices $i and $j do not match")
				node.instructions[i] = align_with_predecessor(p,pj)
			end
		end
	end
	node
end

GraphUtils.validate_edge(n1::AbstractPlanningPredicate,n2::AbstractPlanningPredicate) = true

align_with_successor(node::BOT_GO,succ::BOT_COLLECT) 			= BOT_GO(first_valid(node.r,succ.r), node.x1, first_valid(node.x2,succ.x))
align_with_successor(node::BOT_GO,succ::BOT_GO) 				= BOT_GO(first_valid(node.r,succ.r), node.x1, first_valid(node.x2,succ.x1))

GraphUtils.validate_edge(n1::N1,n2::N2) where {N1<:Union{BOT_AT,OBJECT_AT},N2<:Union{BOT_AT,OBJECT_AT}} = false
GraphUtils.validate_edge(n1::BOT_AT,		n2::BOT_GO			) = (n1.x 	== n2.x1) && (n1.r == n2.r)
GraphUtils.validate_edge(n1::BOT_GO,			n2::BOT_GO			) = (n1.x2 	== n2.x1) && (n1.r == n2.r)
GraphUtils.validate_edge(n1::BOT_GO,			n2::BOT_COLLECT		) = (n1.x2 	== n2.x ) && (n1.r == n2.r)
GraphUtils.validate_edge(n1::BOT_COLLECT,		n2::BOT_CARRY		) = (n1.x 	== n2.x1) && (n1.o == n2.o)
GraphUtils.validate_edge(n1::BOT_CARRY,		n2::BOT_COLLECT		) = false
GraphUtils.validate_edge(n1::BOT_CARRY,		n2::BOT_DEPOSIT		) = (n1.x2 	== n2.x) && (n1.o == n2.o)
GraphUtils.validate_edge(n1::BOT_DEPOSIT,		n2::BOT_CARRY		) = false
GraphUtils.validate_edge(n1::BOT_DEPOSIT,		n2::BOT_GO			) = (n1.x 	== n2.x1)
GraphUtils.validate_edge(n1::N,n2::N) where {N<:Union{BOT_COLLECT,BOT_DEPOSIT}} = (n1.x == n2.x) # job shop edges are valid
GraphUtils.validate_edge(n1::Operation,n2::OBJECT_AT) = haskey(postconditions(n1),get_object_id(n2))
GraphUtils.validate_edge(n1::OBJECT_AT,n2::Operation) = haskey(preconditions(n2),get_object_id(n1))

Base.string(pred::OBJECT_AT)	=  string("O(",get_id(get_object_id(pred)),",",map(x->get_id(x), get_location_ids(pred)),")")
Base.string(pred::BOT_AT)  		=  string("R(",get_id(get_robot_id(pred)),",",get_id(get_location_id(pred)),")")
Base.string(a::BOT_GO)        	=  string("GO(",get_id(get_robot_id(a)),",",get_id(get_initial_location_id(a)),"=>",get_id(get_destination_location_id(a)),")")
Base.string(a::BOT_COLLECT)   	=  string("COLLECT(",get_id(get_robot_id(a)),",",get_id(get_object_id(a)),",",get_id(get_location_id(a)),")")
Base.string(a::BOT_CARRY)     	=  string("CARRY(",get_id(get_robot_id(a)),",",get_id(get_object_id(a)),",",get_id(get_initial_location_id(a)),"=>",get_id(get_destination_location_id(a)),")")
Base.string(a::BOT_DEPOSIT)   	=  string("DEPOSIT(",get_id(get_robot_id(a)),",",get_id(get_object_id(a)),",",get_id(get_location_id(a)),")")
Base.string(op::Operation)    	=  string("OP(",get_id(get_operation_id(op)),")")
Base.string(a::TEAM_ACTION)   	=  string("TEAM_ACTION( ",map(i->string(string(i), ","), a.instructions)...," )")

title_string(pred::OBJECT_AT,verbose=true) = verbose ? string("O",get_id(get_object_id(pred)),"-",get_id(get_location_id(pred))) : string("O",get_id(get_object_id(pred)));
title_string(pred::BOT_AT,verbose=true)  = verbose ? string("R",get_id(get_robot_id(pred)),"-",get_id(get_location_id(pred))) : string("R",get_id(get_robot_id(pred)));
title_string(a::BOT_GO,verbose=true)        = verbose ? string("go\n",get_id(get_robot_id(a)),",",get_id(get_initial_location_id(a)),",",get_id(get_destination_location_id(a))) : "go";
title_string(a::BOT_COLLECT,verbose=true)   = verbose ? string("collect\n",get_id(get_robot_id(a)),",",get_id(get_object_id(a)),",",get_id(get_location_id(a))) : "collect";
title_string(a::BOT_CARRY,verbose=true)     = verbose ? string("carry\n",get_id(get_robot_id(a)),",",get_id(get_object_id(a)),",",get_id(get_destination_location_id(a))) : "carry";
title_string(a::BOT_DEPOSIT,verbose=true)   = verbose ? string("deposit\n",get_id(get_robot_id(a)),",",get_id(get_object_id(a)),",",get_id(get_location_id(a))) : "deposit";
title_string(op::Operation,verbose=true)= verbose ? string("op",get_id(get_operation_id(op))) : "op";
title_string(a::TEAM_ACTION,verbose=true) = verbose ? string("T-", team_action_type(a), "\n","r: (",map(i->string(get_id(get_robot_id(i)), ","), a.instructions)...,")") : string("TEAM","\n",title_string(team_action_type(a)(),verbose)) #string("TEAM\n", string(team_action_type(a)))
