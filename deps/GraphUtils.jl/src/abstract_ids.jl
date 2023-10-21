export
	AbstractRobotType,
	DeliveryBot,
	DefaultRobotType

abstract type AbstractRobotType end
struct DeliveryBot <: AbstractRobotType end
const DefaultRobotType = DeliveryBot

export
	AbstractID,
	TemplatedID,
    ObjectID,
    BotID,
    RobotID,
    LocationID,
    ActionID,
    OperationID,
	AgentID,
	VtxID

abstract type AbstractID end

"""
    struct TemplatedID{T} <: AbstractID 

A simple way to dispatch by Node type.
"""
struct TemplatedID{T} <: AbstractID 
    id::Int
end

@with_kw struct ObjectID <: AbstractID
	id::Int = -1
end
@with_kw struct BotID{R<:AbstractRobotType} <: AbstractID
	id::Int = -1
end
const RobotID = BotID{DeliveryBot}
@with_kw struct LocationID <: AbstractID
	id::Int = -1
end
@with_kw struct ActionID <: AbstractID
	id::Int = -1
end
@with_kw struct OperationID <: AbstractID
	id::Int = -1
end
"""
	AgentID
Special helper for identifying agents.
"""
@with_kw struct AgentID <: AbstractID
	id::Int = -1
end
"""
	VtxID
Special helper for identifying schedule vertices.
"""
@with_kw struct VtxID <: AbstractID
	id::Int = -1
end

Base.summary(id::A) where {A<:AbstractID} = string(string(A),"(",get_id(id),")")
Base.string(id::A) where {A<:AbstractID} = summary(id)

export
	get_unique_id,
	get_unique_invalid_id,
	reset_id_counter!,
	reset_all_id_counters!,
	reset_invalid_id_counter!,
	reset_all_invalid_id_counters!,
    reset_operation_id_counter!,
    get_unique_operation_id,
    reset_action_id_counter!,
    get_unique_action_id

VALID_ID_COUNTERS = Dict{DataType,Int}()
INVALID_ID_COUNTERS = Dict{DataType,Int}()

function get_unique_id(::Type{T}) where {T}
	global VALID_ID_COUNTERS
	id = get!(VALID_ID_COUNTERS,T,1)
	VALID_ID_COUNTERS[T] += 1
	return T(id)
end
function reset_id_counter!(::Type{T}) where {T}
	global VALID_ID_COUNTERS
	VALID_ID_COUNTERS[T] = 1
end
function reset_all_id_counters!()
	global VALID_ID_COUNTERS
	for k in collect(keys(VALID_ID_COUNTERS))
		reset_id_counter!(k)
	end
end
function get_unique_invalid_id(::Type{T}) where {T}
	global INVALID_ID_COUNTERS
	id = get!(INVALID_ID_COUNTERS,T,-1)
	INVALID_ID_COUNTERS[T] -=1
	return T(id)
end
function reset_invalid_id_counter!(::Type{T}) where {T}
	global INVALID_ID_COUNTERS
	INVALID_ID_COUNTERS[T] = -1
end
function reset_all_invalid_id_counters!()
	global INVALID_ID_COUNTERS
	for k in collect(keys(INVALID_ID_COUNTERS))
		reset_invalid_id_counter!(k)
	end
end
get_unique_operation_id() = get_unique_id(OperationID)
get_unique_action_id() = get_unique_id(ActionID)
reset_action_id_counter!() = reset_id_counter!(ActionID)
reset_operation_id_counter!() = reset_id_counter!(OperationID)

# OPERATION_ID_COUNTER = 0
# get_unique_operation_id() = OperationID(Int(global OPERATION_ID_COUNTER += 1))
# function reset_operation_id_counter!()
#     global OPERATION_ID_COUNTER = 0
# end
# ACTION_ID_COUNTER = 0
# get_unique_action_id() = ActionID(Int(global ACTION_ID_COUNTER += 1))
# function reset_action_id_counter!()
#     global ACTION_ID_COUNTER = 0
# end

export
	get_id,
	valid_id

get_id(id::AbstractID) = id.id
get_id(id::Int) = id
Base.:+(id::A,i::Int) where {A<:AbstractID} = A(get_id(id)+i)
Base.:+(id::A,i::A) where {A<:AbstractID} = A(get_id(id)+get_id(i))
Base.:-(id::A,i::Int) where {A<:AbstractID} = A(get_id(id)-i)
Base.:-(id::A,i::A) where {A<:AbstractID} = A(get_id(id)-get_id(i))
Base.:(<)(id1::AbstractID,id2::AbstractID) = get_id(id1) < get_id(id2)
Base.:(>)(id1::AbstractID,id2::AbstractID) = get_id(id1) > get_id(id2)
Base.isless(id1::AbstractID,id2::AbstractID) = id1 < id2
Base.convert(::Type{ID},i::Int) where {ID<:AbstractID} = ID(i)
Base.copy(id::ID) where {ID<:AbstractID} = ID(get_id(id))

valid_id(id::AbstractID) = get_id(id) > -1
