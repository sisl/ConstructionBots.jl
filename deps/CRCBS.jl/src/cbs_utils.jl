###############################################################################
################################## Conflicts ###################################
################################################################################
export
    Conflict,
    state_conflict,
    action_conflict,
    conflict_type,
    agent1_id,
    agent2_id,
    node1,
    node2,
    is_state_conflict,
    is_action_conflict

@with_kw struct Conflict{P1 <: PathNode,P2 <: PathNode}
    conflict_type   ::Symbol    = :NullConflict
    agent1_id       ::Int       = -1
    agent2_id       ::Int       = -1
    node1           ::P1        = P1()
    node2           ::P2        = P2()
    t               ::Int       = -1
end
state_conflict(args...) = Conflict(:StateConflict,args...)
action_conflict(args...) = Conflict(:ActionConflict,args...)
conflict_type(c::Conflict)   = c.conflict_type
is_state_conflict(c::Conflict) = conflict_type(c) == :StateConflict
is_action_conflict(c::Conflict) = conflict_type(c) == :ActionConflict
agent1_id(c::Conflict)       = c.agent1_id
agent2_id(c::Conflict)       = c.agent2_id
node1(c::Conflict)           = c.node1
node2(c::Conflict)           = c.node2
state1(c::Conflict)          = get_s(node1(c))
state2(c::Conflict)          = get_s(node2(c))
action1(c::Conflict)         = get_a(node1(c))
action2(c::Conflict)         = get_a(node2(c))
next_state1(c::Conflict)     = get_sp(node1(c))
next_state2(c::Conflict)     = get_sp(node2(c))
time_of(c::Conflict)         = c.t
Base.string(c::Conflict) = string(
    "conflict type: ",conflict_type(c),
    ": agent1=",agent1_id(c),
    ", agent2=", agent2_id(c),
    ", v1=(",get_s(node1(c)).vtx,",",get_sp(node1(c)).vtx,")",
    ", v2=(",get_s(node2(c)).vtx,",",get_sp(node2(c)).vtx,")",
    ", t=",get_s(node1(c)).t)

const SymmetricConflict{N} = Conflict{N,N}

Base.isless(c1::C,c2::C) where {C<:Conflict} = (
    [c1.t,c1.agent1_id,c1.agent2_id,is_state_conflict(c1)] < [c2.t,c2.agent1_id,c2.agent2_id,is_state_conflict(c2)]
)

""" Checks if a conflict is valid """
is_valid(c::C) where {C<:Conflict} = ((is_state_conflict(c) || is_action_conflict(c))
                                && (agent1_id(c) != agent2_id(c))
                                && (agent1_id(c) != -1)
                                && (agent2_id(c) != -1))

export
    detect_conflicts!,
    reset_conflict_table!,
    detect_conflicts

""" add detected conflicts to conflict table """
function detect_conflicts!(conflict_table,n1::P1,n2::P2,i::Int,j::Int,t::Int) where {P1<:PathNode,P2<:PathNode}
    error("detect_conflicts!(conflict_table,n1,n2,i,j,t) not yet implemented \n
        for conflict_table::",typeof(conflict_table),". \n
        Aborting Conflict Checking.")
    return conflict_table
end

"""
detect conflicts between paths
"""
function detect_conflicts!(conflict_table,path1::P1,path2::P2,i::Int,j::Int,t0::Int=1) where {P1<:AbstractPath,P2<:AbstractPath}
    # print("detect_conflicts!(conflict_table,path1::Path,path2::Path,i::Int,j::Int)\n")
    if length(path1) > length(path2)
        path2 = extend_path(path2,length(path1))
    elseif length(path1) < length(path2)
        path1 = extend_path(path1,length(path2))
    end
    @assert(length(path1) == length(path2))
    reset_conflict_table!(conflict_table,i,j)
    for t in t0:length(path1)
        path_node1 = path1[t]
        path_node2 = path2[t]
        detect_conflicts!(conflict_table,path_node1,path_node2,i,j,t)
    end
    return conflict_table
end

"""
Populates a `ConflictTable` with all conflicts that occur in a given vector
of paths. Conflict checking is performed in a pairwise fashion between
all paths.

args:
- conflict_table        a `ConflictTable` to store the detected conflicts
- paths:                a list of `Path`s, one for each individual agent
- idxs                  (optional) a list of agent ids for which to check
                        collisions against all other agents
"""
function detect_conflicts!(conflict_table, paths::Vector{P}, idxs=collect(1:length(paths)),args...) where {P<:AbstractPath}
    # print("detect_conflicts!(conflict_table, paths::LowLevelSolution, idxs=collect(1:length(paths)))\n")
    for (i,path1) in enumerate(paths)
        for (j,path2) in enumerate(paths)
            if j <= i
                continue
            end
            if (i in idxs) || (j in idxs) # save time by working only on the upper triangle
                detect_conflicts!(conflict_table,path1,path2,i,j,args...)
            end
        end
    end
    return conflict_table
end

function detect_conflicts!(conflict_table, solution::L, args...) where {L <: LowLevelSolution}
    detect_conflicts!(conflict_table,get_paths(solution),args...)
end

export
    detect_state_conflict

"""
Detect a `StateConflict` between two path nodes. Must be overridden for each specific path class
"""
function detect_state_conflict(n1::P1,n2::P2) where {P1<:PathNode,P2<:PathNode}
    error("detect_state_conflict(n1,n2) Not Implemented for \nn1::",
        typeof(n1),",\nn2::",typeof(n2))
    return false
end

""" Checks for a `StateConflict` between two `Path`s at time t """
function detect_state_conflict(path1::P1, path2::P2, t::Int) where {P1<:AbstractPath,P2<:AbstractPath}
    if detect_state_conflict(get_path_node(path1,t),get_path_node(path2,t))
        return true
    end
    return false
end

export
    detect_action_conflict

"""
Detect an `ActionConflict` between two path nodes. Must be overridden for each specific path class
"""
function detect_action_conflict(n1::P1,n2::P2) where {P1<:PathNode,P2<:PathNode}
    error("detect_action_conflict(n1,n2) Not Implemented for \nn1::",
        typeof(n1),",\nn2::",typeof(n2))
    return false
end

""" Checks for an `ActionConflict` between two `Path`s at time t """
function detect_action_conflict(path1::P1, path2::P2,t::Int) where {P1<:Path,P2<:Path}
    if detect_action_conflict(get_path_node(path1,t),get_path_node(path2,t))
        return true
    end
    return false
end

export
    ConflictTable,
    get_conflicts,
    count_conflicts,
    add_conflict!,
    get_next_conflict

"""
A lookup table to store all conflicts that have been detected
"""
@with_kw struct ConflictTable{C<:Conflict}
    state_conflicts::Dict{Tuple{Int,Int},Vector{C}} = Dict{Tuple{Int,Int},Vector{C}}()
    action_conflicts::Dict{Tuple{Int,Int},Vector{C}} = Dict{Tuple{Int,Int},Vector{C}}()
    # TODO sorted_conflict_list
end
default_conflict(table::ConflictTable{C}) where {C} = C()

""" helper for retrieving conflicts associated with agents i and j """
function get_conflicts(conflict_table::ConflictTable,i::Int,j::Int)
    if i <= j
        state_conflicts = get(conflict_table.state_conflicts, (i,j), Vector{Conflict}())
        action_conflicts = get(conflict_table.action_conflicts, (i,j), Vector{Conflict}())
    else
        @assert !haskey(conflict_table.state_conflicts, (i,j))
        @assert !haskey(conflict_table.action_conflicts, (i,j))
        return get_conflicts(conflict_table,j,i)
    end
    return state_conflicts, action_conflicts
end

export
    get_all_state_conflicts,
    get_all_action_conflicts,
    get_all_conflicts

get_all_state_conflicts(ct) = Base.Iterators.flatten((v for (k,v) in ct.state_conflicts if !isempty(v)))
get_all_action_conflicts(ct) = Base.Iterators.flatten((v for (k,v) in ct.action_conflicts if !isempty(v)))
get_all_conflicts(ct) = Base.Iterators.flatten((get_all_state_conflicts(ct),get_all_action_conflicts(ct)))

"""
    count_conflicts(conflict_table::ConflictTable,i::Int,j::Int)

helper for counting the number of conflicts between agent i and agent j
"""
function count_conflicts(conflict_table::ConflictTable,i::Int,j::Int)
    state_conflicts, action_conflicts = get_conflicts(conflict_table,i,j)
    return length(state_conflicts) + length(action_conflicts)
end
function count_conflicts(conflict_table::ConflictTable,idxs1::Vector{Int},idxs2::Vector{Int})
    N = 0
    n = max(maximum(idxs1),maximum(idxs2))
    mat = sparse(zeros(Int,n,n))
    for i in idxs1
        for j in idxs2
            if i != j
                n = count_conflicts(conflict_table,i,j)
                if i < j
                    mat[i,j] = n
                else
                    mat[j,i] = n
                end
            end
        end
    end
    return sum(mat)
end
function count_conflicts(conflict_table::ConflictTable)
    N = 0
    for (k,v) in conflict_table.state_conflicts
        N += length(v)
    end
    for (k,v) in conflict_table.action_conflicts
        N += length(v)
    end
    return N
end

"""
    helper to insert conflicts into ConflictTable
"""
function add_conflict!(conflict_table::ConflictTable,c)
    @assert is_valid(c)
    if !is_valid(c)
        return
    end
    i = agent1_id(c)
    j = agent2_id(c)
    @assert i < j
    if is_state_conflict(c)
        tab = conflict_table.state_conflicts
    else
        tab = conflict_table.action_conflicts
    end
    vec = get!(tab, (i,j), valtype(conflict_table.state_conflicts)())
    insert!(vec, searchsortedfirst(vec, c), c)
    @assert issorted(vec)
    # push!(vec, c)
    conflict_table
end

"""
    get_next_conflict(conflict_table::ConflictTable)

Returns the next conflict (temporally) that occurs in a conflict table
"""
function get_next_conflict(table::ConflictTable)
    conflict = default_conflict(table)
    for (k,v) in table.state_conflicts
        if ~isempty(v)
            # sort!(v)
            candidate = v[1]
            if time_of(candidate) < time_of(conflict) || !is_valid(conflict)
                conflict = candidate
            end
        end
    end
    for (k,v) in table.action_conflicts
        if ~isempty(v)
            # sort!(v)
            candidate = v[1]
            if time_of(candidate) < time_of(conflict) || !is_valid(conflict)
                conflict = candidate
            end
        end
    end
    return conflict
end

function Base.copy(c::ConflictTable)
   c_new = ConflictTable()
   for (k,v) in c.state_conflicts
       c_new.state_conflicts[k] = copy(v)
   end
   for (k,v) in c.action_conflicts
       c_new.action_conflicts[k] = copy(v)
   end
   return c_new
end

function reset_conflict_table!(conflict_table::ConflictTable,i::Int,j::Int)
    if i <= j
        conflict_table.state_conflicts[(i,j)]   =  Vector{Conflict}()
        conflict_table.action_conflicts[(i,j)]  =  Vector{Conflict}()
    else
        return reset_conflict_table!(conflict_table,j,i)
    end
    return conflict_table
end

# """
#     Returns a `ConflictTable` of all conflicts that occur in a given solution
#
#     args:
#     - conflict_table        a `ConflictTable` to store the detected conflicts
#     - paths:                a list of `Path`s, one for each individual agent
#     - idxs                  (optional) a list of agent ids for which to check
#                             collisions against all other agents
# """
function detect_conflicts(paths::Vector{P}, idxs=collect(1:length(paths)),args...) where {P <: Path}
    conflict_table = ConflictTable{SymmetricConflict{node_type(P())}}()
    detect_conflicts!(conflict_table,paths,idxs,args...)
    conflict_table
end
function detect_conflicts(solution::L, idxs=collect(1:length(get_paths(solution))),args...) where {L<:LowLevelSolution}
    conflict_table = ConflictTable{SymmetricConflict{node_type(solution)}}()
    detect_conflicts!(conflict_table,solution,idxs,args...)
    conflict_table
end

""" add detected conflicts to conflict table """
function detect_conflicts!(conflicts::ConflictTable,n1::PathNode,n2::PathNode,i::Int,j::Int,t::Int)
    if detect_state_conflict(n1,n2)
        add_conflict!(conflicts,state_conflict(i,j,n1,n2,t))
    end
    if detect_action_conflict(n1,n2)
        add_conflict!(conflicts,action_conflict(i,j,n1,n2,t))
    end
end

################################################################################
################################# Constraints ##################################
################################################################################

export
    CBSConstraint,
    get_agent_id,
    get_time_of,
    state_constraint,
    action_constraint,
    is_state_constraint,
    is_action_constraint

struct CBSConstraint{N}
    type::Symbol
    a::Int # agent ID
    v::N   # PathNode
    interval::Tuple{Int,Int} # time ID
end
CBSConstraint(T::Symbol,a::Int,v::N,t1::Int,t2=t1) where {N} = CBSConstraint(T,a,v,(t1,t2))
get_agent_id(c::CBSConstraint) = c.a
get_time_of(c::CBSConstraint) = c.interval[1]
get_constraint_interval(c::CBSConstraint) = c.interval
get_path_node(c::CBSConstraint) = c.v
get_constraint_type(c::CBSConstraint) = c.type
for op in [:get_s,:get_a,:get_sp]
    @eval $op(c::CBSConstraint) = $op(get_path_node(c))
end
Base.isless(c1::CBSConstraint,c2::CBSConstraint) = get_time_of(c1) < get_time_of(c2)
function Base.string(c::CBSConstraint)
    p = get_path_node(c)
    string(
        "$(get_constraint_type(c)):",
        " agent_id=$(get_agent_id(c)), interval=$(get_constraint_interval(c)), ",
        "$(string(get_s(p))) -- $(string(get_a(p))) -- X$(string(get_sp(p)))X"
        )
end
state_constraint(args...) = CBSConstraint(:StateConstraint,args...)
action_constraint(args...) = CBSConstraint(:ActionConstraint,args...)
is_state_constraint(c::CBSConstraint) = get_constraint_type(c) == :StateConstraint
is_action_constraint(c::CBSConstraint) = get_constraint_type(c) == :ActionConstraint

################################################################################
############################## Constraint Tables ###############################
################################################################################
export
    serialize,
    serialize_jointly,
    deserialize,
    num_states,
    num_actions

"""
    serialize(env,state,t=-1)

Encodes a state or action as an integer
"""
function serialize end
serialize(mapf::MAPF,args...) = serialize(mapf.env,args...)

function serialize_jointly end
serialize_jointly(mapf::MAPF,args...) = serialize_jointly(mapf.env,args...)

"""
    deserialize(env,idx,t=-1)

Decodes an integer encoding of a state or action of type `state_type(env)` or
`action_type(env)`
"""
function deserialize end
deserialize(mapf::MAPF,args...) = deserialize(mapf.env,args...)

"""
    num_states(env)

Returns the cardinality of the single agent state space for an environment.
If the state and action spaces are finite and discrete, a discrete
constraint table may be used for fast lookup.
"""
function num_states end

"""
    num_actions(env)

Returns the cardinality of the single agent state space for an environment.
If the state and action spaces are finite and discrete, a discrete
constraint table may be used for fast lookup.
"""
function num_actions end

export
    SpaceTrait,
    DiscreteSpace,
    ContinuousSpace,
    state_space_trait,
    action_space_trait

abstract type SpaceTrait end
struct DiscreteSpace <: SpaceTrait end
struct ContinuousSpace <: SpaceTrait end

"""
    state_space_trait(env)

Defaults to `DiscreteSpace`
"""
state_space_trait(env) = DiscreteSpace()

"""
    action_space_trait(env)

Defaults to `DiscreteSpace`
"""
action_space_trait(env) = DiscreteSpace()

state_space_trait(mapf::MAPF) = state_space_trait(mapf.env)
action_space_trait(mapf::MAPF) = action_space_trait(mapf.env)
num_states(mapf::MAPF) = num_states(mapf.env)
num_actions(mapf::MAPF) = num_actions(mapf.env)

export
    DiscreteConstraintTable,
    discrete_constraint_table


# export OffsetSparseMatrix
#
# """
#     OffsetSparseMatrix{T}
#
# Represents a sparse matrix whose dimensions may begin at non-zero integer.
# example:
# ```
# table = OffsetSparseMatrix{Bool}(10,10,5,5)
#
# ```
# """
# struct OffsetSparseMatrix{T}
#     table::SparseMatrixCSC{T,Int}
#     offset::SVector{2,Int}
#     function OffsetSparseMatrix{T}(i::Int,j::Int,start_i::Int=0,start_j::Int=0) where {T}
#         new(spzeros(T,i,j),SVector{2,Int}(start_i,start_j))
#     end
# end
# Base.getindex(m::OffsetSparseMatrix,i,j) = m.table[i .- m.offset[1],j .- m.offset[2]]
# Base.setindex!(m::OffsetSparseMatrix,val,i,j) = setindex!(m.table,val,i - m.offset[1], j - m.offset[2])
# get_offset(m::OffsetSparseMatrix) = m.offset
# get_offset(m::OffsetSparseMatrix,d::Int) = m.offset[d]
# Base.size(m::OffsetSparseMatrix) = size(m.table) .+ get_offset(m)
# function SparseArrays.findnz(m::OffsetSparseMatrix)
#     row_idxs, col_idxs, vals = findnz(m.table)
#     di = get_offset(m,1)
#     dj = get_offset(m,2)
#     map(i->i+di,row_idxs), map(j->j+dj,col_idxs), vals
# end
# Base.checkbounds(m::OffsetSparseMatrix,i,j) = all(get_offset(m) .<= (i,j) .<= size(m))

"""
    DiscreteStateTable

Stores constraints for a discrete state space
"""
struct DiscreteConstraintTable
    state_constraints::SparseMatrixCSC{Bool,Int}
    action_constraints::SparseMatrixCSC{Bool,Int}
    agent_id::Int
end
function discrete_constraint_table(n_states::Int,n_actions::Int,agent_id::Int=-1,tf::Int=n_states*4)
    DiscreteConstraintTable(
        spzeros(Bool,n_states,tf),
        spzeros(Bool,n_actions,tf),
        agent_id
        )
end
function discrete_constraint_table(n_states::Int,n_actions::Int,agent_id::Int,tf::Float64) 
    @assert abs(tf - Int(round(tf))) < 0.01
    discrete_constraint_table(n_states,n_actions,agent_id,Int(round(tf)))
end
function discrete_constraint_table(env,agent_id=-1,tf=num_states(env)*4)
    @assert isa(state_space_trait(env),DiscreteSpace)
    discrete_constraint_table(num_states(env),num_actions(env),agent_id,tf)
end

export
    get_agent_id,
    state_constraints,
    action_constraints,
    sorted_state_constraints,
    sorted_action_constraints,
    add_constraint!,
    remove_constraint!,
    has_constraint,
    search_constraints

get_agent_id(c::DiscreteConstraintTable)                = c.agent_id
state_constraints(c::DiscreteConstraintTable)           = c.state_constraints
action_constraints(c::DiscreteConstraintTable)          = c.action_constraints
function state_constraints(env,table::DiscreteConstraintTable)
    row_idxs, col_idxs, _ = findnz(table.state_constraints)
    s_constraints = Vector{CBSConstraint{node_type(env)}}()
    for (idx,t) in zip(row_idxs,col_idxs)
        sp,_ = deserialize(env,state_type(env)(),idx,t)
        n = node_type(env)(sp=sp)
        push!(s_constraints,state_constraint(get_agent_id(table),n,t))
    end
    return s_constraints
end
function action_constraints(env,table::DiscreteConstraintTable)
    row_idxs, col_idxs, _ = findnz(table.action_constraints)
    constraints = Vector{CBSConstraint{node_type(env)}}()
    for (idx,t) in zip(row_idxs,col_idxs)
        a,_ = deserialize(env,action_type(env)(),idx,t)
        n = node_type(env)(a=a)
        push!(constraints,action_constraint(get_agent_id(table),n,t))
    end
    return constraints
end
sorted_state_constraints(env,table::DiscreteConstraintTable) = state_constraints(env,table)
sorted_action_constraints(env,table::DiscreteConstraintTable) = action_constraints(env,table)
# function sorted_state_constraints(env,table::DiscreteConstraintTable)
#     row_idxs, col_idxs, _ = findnz(table.state_constraints)
#     s_constraints = Vector{CBSConstraint{node_type(env)}}()
#     for (idx,t) in zip(row_idxs,col_idxs)
#         sp,_ = deserialize(env,state_type(env)(),idx,t)
#         n = node_type(env)(sp=sp)
#         push!(s_constraints,state_constraint(get_agent_id(table),n,t))
#     end
#     return s_constraints
# end
# function sorted_action_constraints(env,table::DiscreteConstraintTable)
#     row_idxs, col_idxs, _ = findnz(table.action_constraints)
#     constraints = Vector{CBSConstraint{node_type(env)}}()
#     for (idx,t) in zip(row_idxs,col_idxs)
#         a,_ = deserialize(env,action_type(env)(),idx,t)
#         n = node_type(env)(a=a)
#         push!(constraints,action_constraint(get_agent_id(table),n,t))
#     end
#     return constraints
# end
"""
    search_constraints(env,table,n::PathNode)

Returns all `CBSConstraint`s and `CBSConstraint`s that match `n`,
regardless of time.
"""
function search_constraints(env,table::DiscreteConstraintTable,n::N) where {N<:PathNode}
    idx,_ = serialize(env,get_sp(n))
    row_idxs, col_idxs, _ = findnz(table.state_constraints)
    s_constraints = Vector{CBSConstraint{node_type(env)}}()
    for (i,(r,t)) in enumerate(zip(row_idxs,col_idxs))
        if r == idx
            push!(s_constraints,state_constraint(get_agent_id(table),n,t))
        end
    end
    idx,_ = serialize(env,get_a(n))
    row_idxs, col_idxs, _ = findnz(table.action_constraints)
    a_constraints = Vector{CBSConstraint{node_type(env)}}()
    for (i,(r,t)) in enumerate(zip(row_idxs,col_idxs))
        if r == idx
            push!(a_constraints,action_constraint(get_agent_id(table),n,t))
        end
    end
    return s_constraints, a_constraints
end

"""
Sets a `CBSConstraint` value in a DiscreteConstraintTable
"""
function set_constraint!(env,table::DiscreteConstraintTable,c::CBSConstraint,val,check=false)
    @assert get_agent_id(table) == get_agent_id(c)
    t1,t2 = get_constraint_interval(c)
    if is_state_constraint(c)
        idx,t1 = serialize(env,get_sp(get_path_node(c)),t1)
        _,t2 = serialize(env,get_sp(get_path_node(c)),t2)
        if check
            @assert all(table.state_constraints[idx,t1:t2] .!= val) "constraint $(string(c)) is $val in table, but should be $(!val)"
        end
        table.state_constraints[idx,t1:t2] .= val
    else
        idx,t1 = serialize(env,get_a(get_path_node(c)),t1)
        _,t2 = serialize(env,get_a(get_path_node(c)),t2)
        if check
            @assert all(table.action_constraints[idx,t1:t2] .!= val) "constraint $(string(c)) is $val in table, but should be $(!val)"
        end
        table.action_constraints[idx,t1:t2] .= val
    end
    return table
end

""" Adds a `CBSConstraint` to a DiscreteConstraintTable """
add_constraint!(env,table,c,check=true) =  set_constraint!(env,table,c,true,check)

""" Removes a `CBSConstraint` to a DiscreteConstraintTable """
remove_constraint!(env,table,c,check=false) = set_constraint!(env,table,c,false,check)

function check_constraint_bounds(table,idx,t)
    if !checkbounds(Bool,table,idx,t)
        throw(SolverException("BoundsError: Trying to check constraint at $idx,$t, but size(table) = $(size(table))"))
    end
end

"""
    has_constraint(env,table,c::CBSConstraint)
"""
function has_constraint(env,table::DiscreteConstraintTable,c::CBSConstraint)
    @assert get_agent_id(table) == get_agent_id(c)
    if is_state_constraint(c)
        idx,t = serialize(env,get_sp(get_path_node(c)),get_time_of(c))
        check_constraint_bounds(table.state_constraints,idx,t)
        return table.state_constraints[idx,t]
    else
        idx,t = serialize(env,get_a(get_path_node(c)),get_time_of(c))
        check_constraint_bounds(table.action_constraints,idx,t)
        return table.action_constraints[idx,t]
    end
end

# """
#     constraint dictionary for fast constraint lookup within a_star!
# """
# @with_kw struct ConstraintTable{N}
#     # Sets
#     state_constraints::Set{CBSConstraint{N}} = Set{CBSConstraint{N}}()
#     action_constraints::Set{CBSConstraint{N}} = Set{CBSConstraint{N}}()
#     # Vectors
#     sorted_state_constraints::Vector{CBSConstraint{N}} = Vector{CBSConstraint{N}}()
#     sorted_action_constraints::Vector{CBSConstraint{N}} = Vector{CBSConstraint{N}}()
#     agent_id::Int = -1 # agent_id
# end
# get_agent_id(c::ConstraintTable) = c.agent_id
# state_constraints(c::ConstraintTable) = c.state_constraints
# state_constraints(env,c::ConstraintTable) = state_constraints(c)
# action_constraints(c::ConstraintTable) = c.action_constraints
# action_constraints(env,c::ConstraintTable) = action_constraints(c)
# sorted_state_constraints(env,c::ConstraintTable) = c.sorted_state_constraints
# sorted_action_constraints(env,c::ConstraintTable) = c.sorted_action_constraints
# function search_constraints(env,table::ConstraintTable,n::N) where {N<:PathNode}
#     idx,_ = serialize(env,get_sp(n))
#     s_constraints = Vector{CBSConstraint{node_type(env)}}()
#     for constraint in sorted_state_constraints(env,table)
#         sp = get_sp(get_path_node(constraint))
#         idxp, _ = serialize(env,sp)
#         if idxp == idx
#             push!(s_constraints,constraint)
#         end
#     end
#     idx,_ = serialize(env,get_a(n))
#     a_constraints = Vector{CBSConstraint{node_type(env)}}()
#     for constraint in sorted_action_constraints(env,table)
#         a = get_a(get_path_node(constraint))
#         idxp, _ = serialize(env,a)
#         if idxp == idx
#             push!(a_constraints,constraint)
#         end
#     end
#     return s_constraints, a_constraints
# end
#
# """
#     Adds a `CBSConstraint` to a ConstraintTable
# """
# function add_constraint!(env,table::ConstraintTable,c::CBSConstraint)
#     @assert get_agent_id(table) == get_agent_id(c)
#     if is_state_constraint(c)
#         push!(table.state_constraints, c)
#         insert_to_sorted_array!(table.sorted_state_constraints, c)
#     else
#         push!(table.action_constraints, c)
#         insert_to_sorted_array!(table.sorted_action_constraints, c)
#     end
# end
#
# function has_constraint(env,table::ConstraintTable,c::CBSConstraint)
#     @assert get_agent_id(table) == get_agent_id(c)
#     if is_state_constraint(c)
#         return (c in table.state_constraints)
#     else
#         return (c in table.action_constraints)
#     end
# end

export
    ConstraintTreeNode,
    solution_type,
    initialize_root_node,
    initialize_child_search_node,
    get_constraints,
    violates_constraints,
    generate_constraints_from_conflict

"""
A node of a constraint tree. Each node has a set of constraints, a candidate
    solution (set of robot paths), and a cost
"""
@with_kw struct ConstraintTreeNode{S,C,D} #,E<:AbstractLowLevelEnv{S,A}} # CBS High Level Node
    solution        ::S = S()
    # maps agent_id to the set of constraints involving that agent
    constraints     ::C = Dict{Int,DiscreteConstraintTable}() #{node_type(solution)}}()
    # maintains a list of all conflicts
    conflict_table  ::D = ConflictTable{SymmetricConflict{node_type(solution)}}()
    # index of parent node
    parent          ::Int = -1
    # indices of two child nodes
    children        ::Tuple{Int,Int} = (-1,-1)
    # unique id
    id              ::Int = -1
    # trace
    trace           ::Vector{Int} = Int[]
end
for op in [
    :cost_type,:state_type,:action_type,:path_type,:get_cost,:get_paths,
    :get_path_costs,:set_cost!,:set_solution_path!,:set_path_cost!,
    ]
    @eval $op(node::ConstraintTreeNode,args...) = $op(node.solution,args...)
end
solution_type(node::ConstraintTreeNode{S,C,D}) where {S,C,D} = S
constraint_table_type(node::ConstraintTreeNode) = valtype(node.constraints)
function set_trace!(node::C,parent::C,i) where {C<:ConstraintTreeNode}
    append!(node.trace,parent.trace)
    push!(node.trace,i)
    node
end

"""
    initialize_root_node

Construct an empty `ConstraintTreeNode` from a `AbstractMAPF` instance
"""
function initialize_root_node(mapf::AbstractMAPF,solution=get_initial_solution(mapf))
    ConstraintTreeNode(
        solution    = solution,
        constraints = Dict(
            i=>discrete_constraint_table(mapf,i) for i in 1:num_agents(mapf)
            ),
        id = 1)
end
function initialize_root_node(solver,mapf::AbstractMAPF,solution=get_initial_solution(mapf))
    initialize_root_node(mapf,solution)
end

"""
    initialize_child_search_node(parent_node::ConstraintTreeNode)

Initialize a new `ConstraintTreeNode` with the same `solution` and
    `constraints` as the parent node
"""
function initialize_child_search_node(parent_node::N, solution=copy(parent_node.solution)) where {N<:ConstraintTreeNode}
    N(
        solution        = solution,
        constraints     = deepcopy(parent_node.constraints),
        conflict_table  = deepcopy(parent_node.conflict_table),
        parent          = parent_node.id
    )
end
function initialize_child_search_node(mapf::AbstractMAPF,parent_node::N, solution=copy(parent_node.solution)) where {N<:ConstraintTreeNode}
    initialize_child_search_node(parent_node, solution)
end
function initialize_child_search_node(solver, mapf::AbstractMAPF,parent_node::N, solution=copy(parent_node.solution)) where {N<:ConstraintTreeNode}
    initialize_child_search_node(mapf, parent_node, solution)
end

"""
retrieve constraints corresponding to this node and this path
"""
function get_constraints(node::N, path_id::Int) where {N<:ConstraintTreeNode}
    @assert haskey(node.constraints,path_id) "path_id = $path_id"
    return node.constraints[path_id]
    # return get(node.constraints, path_id, valtype(node.constraints)())
end

"""
adds a `CBSConstraint` to a ConstraintTreeNode
"""
function add_constraint!(env,node::N,c::CBSConstraint) where {N<:ConstraintTreeNode}
    add_constraint!(env, get_constraints(node, get_agent_id(c)), c)
    return true
end

function detect_conflicts!(node::ConstraintTreeNode, args...)
    detect_conflicts!(node.conflict_table,node.solution,args...)
end
for op in [:count_conflicts,:get_next_conflict,:reset_conflict_table!,:get_conflicts,
    :get_all_state_conflicts,:get_all_action_conflicts,:get_all_conflicts]
    @eval $op(node::ConstraintTreeNode,args...) = $op(node.conflict_table,args...)
end

for op in [:state_constraints, :action_constraints,:sorted_state_constraints, :sorted_action_constraints,]
    @eval $op(env,node::ConstraintTreeNode,idx) = $op(env,get_constraints(node,idx))
end
for op in [:state_constraints, :action_constraints,:sorted_state_constraints, :sorted_action_constraints,]
    @eval $op(env,node::ConstraintTreeNode) = begin
        vcat(
            map(k->collect($op(env,get_constraints(node,k))),
            collect(keys(node.constraints)))...
            )
    end
end

"""
generates a Vector of (State or Action) Constraints from a conflict
"""
function generate_constraints_from_conflict(c::Conflict)
    constraints = Vector{CBSConstraint}()
    if is_state_conflict(c)
        push!(constraints, state_constraint(
            agent1_id(c),
            node1(c),
            time_of(c)
        ))
        push!(constraints, state_constraint(
            agent2_id(c),
            node2(c),
            time_of(c)
        ))
    else
        push!(constraints, action_constraint(
            agent1_id(c),
            node1(c),
            time_of(c)
        ))
        push!(constraints, action_constraint(
            agent2_id(c),
            node2(c),
            time_of(c)
        ))
    end
    return constraints
end

################################################################################
############################## Reservation Table ###############################
################################################################################
export ResourceReservation

"""
    ResourceReservation

`r::ResourceReservation` encodes a that resource `r.resource` is reserved by
agent `r.agent_id` over time interval `r.interval`.
"""
struct ResourceReservation{T<:Real}
    resource_id::Int
    agent_id::Int
    interval::Tuple{T,T}
end
resource_id(r::ResourceReservation)     = r.resource_id
agent_id(r::ResourceReservation)        = r.agent_id
interval(r::ResourceReservation)        = r.interval
start_time(interval::Tuple{T,T}) where {T} = interval[1]
finish_time(interval::Tuple{T,T}) where {T} = interval[2]
start_time(r::ResourceReservation)      = start_time(interval(r))
finish_time(r::ResourceReservation)     = finish_time(interval(r))
function assert_valid(r::ResourceReservation)
    @assert resource_id(r) > 0
    @assert agent_id(r) > 0
    @assert start_time(r) < finish_time(r)
end

export ReservationTable

"""
    ReservationTable

Data structure for reserving resources over a time interval. The table stores a
vector of reservations for each resource. When a new reservation is added to the
table, it is inserted into the reservation vector associated to the requested
resource.
"""
struct ReservationTable{T}
    reservations::SparseVector{Vector{ResourceReservation{T}},Int}
end
time_type(table::ReservationTable{T}) where {T} = T
SparseArrays.SparseVector{Tv,Ti}(n::Int) where {Tv,Ti} = SparseVector{Tv,Ti}(n,Ti[],Tv[])
Base.zero(::Type{Vector{R}}) where {R<:ResourceReservation} = Vector{R}()
Base.iszero(::Vector{R}) where {R<:ResourceReservation} = false
ReservationTable{T}(n::Int) where {T} = ReservationTable{T}(
    SparseVector{Vector{ResourceReservation{T}},Int}(n)
)
# ReservationTable{T}(env) where {T} = ReservationTable{T}(num_states(env)+num_actions(env))
function assert_valid(table::ReservationTable)
    idxs,vals = findnz(table.reservations)
    if length(idxs) > 0
        for (idx,vec) in zip(idxs,vals)
            t = time_type(table)(0)
            for reservation in vec
                @assert start_time(reservation) >= t
                @assert is_valid(reservation)
                # @assert resource_id(reservation) <= length(table.reservations)
                @assert resource_id(reservation) == idx
                t = finish_time(reservation)
            end
        end
    end
end

function spanning_range(vec::Vector{R},res::R) where {R<:ResourceReservation}
    start_idx = searchsorted(map(finish_time,vec),start_time(res))
    stop_idx = searchsorted(map(start_time,vec),finish_time(res))
    i = min(start_idx.start,start_idx.stop) + 1
    j = max(stop_idx.start,stop_idx.stop) - 1
    return i:j
end


export create_reservations

"""
    create_reservation(env,s,a,sp)

Must be overriden for environment env and the relevant state / action types.
"""
function create_reservations end

export
    reserve!,
    is_reserved

"""
    reserve!(table,reservation)

Attempts to add a reservation to the table. If the reservation conflicts with
an existing reservation, does nothing and returns false. If successful, returns
true.
"""
function reserve!(vec::Vector{R},res::R) where {R<:ResourceReservation}
    if length(vec) > 0
        idx = spanning_range(vec,res)
        if idx.start > idx.stop
            insert!(vec,idx.start,res)
        else
            return false # already reserved
        end
    else
        push!(vec,res)
    end
    return true
end
function reserve!(table::ReservationTable,res::ResourceReservation)
    @assert is_valid(res)
    @assert resource_id(res) <= length(table.reservations)
    vec = table.reservations[resource_id(res)]
    if reserve!(vec,res)
        table.reservations[resource_id(res)] = vec
        return true
    end
    return false
end
function reserve!(table::ReservationTable,env,s,a,sp,t=-1)
    valid = true
    for res in create_reservations(env,s,a,sp,t)
        valid &= reserve!(table,res)
    end
    return valid
end

function is_reserved(vec::Vector{R},res::R) where {R<:ResourceReservation}
    if length(vec) > 0
        idx = spanning_range(vec,res)
        if idx.start <= idx.stop
            return true # already reserved
        end
    end
    return false
end
function is_reserved(table::ReservationTable,res::ResourceReservation)
    @assert is_valid(res) string(res)
    @assert resource_id(res) <= length(table.reservations)
    is_reserved(table.reservations[resource_id(res)],res)
end
function is_reserved(table::ReservationTable,env,s,a,sp,t=-1)
    for res in create_reservations(env,s,a,sp,t)
        if is_reserved(table,res)
            return true
        end
    end
    return false
end

export reserved_by

"""
    reserved_by

Returns the IDs of agents who have reserved a resource within a specific time
window.
"""
function reserved_by(vec::Vector{R},res::R) where {R<:ResourceReservation}
    if length(vec) > 0
        idx = spanning_range(vec,res)
        if idx.start <= idx.stop
            # @show interval(res), idx, map(interval,vec)
            # @show map(interval,vec[idx])
            return Set{Int}(map(agent_id,vec[idx]))
        end
    end
    return Set{Int}()
end
function reserved_by(table::ReservationTable,res::ResourceReservation)
    @assert is_valid(res)
    @assert resource_id(res) <= length(table.reservations)
    reserved_by(table.reservations[resource_id(res)],res)
end
function reserved_by(table::ReservationTable,env,s,a,sp,t=-1)
    union(map(res->reserved_by(table,res),create_reservations(env,s,a,sp,t))...)
end

export is_available

"""
    is_available

Returns false if the proposed reservation is not available to any of the agent_ids
"""
function is_available(table::ReservationTable,env,s,a,sp)
    agents = reserved_by(table,env,s,a,sp)
    if isempty(setdiff(agents,get_agent_id(env)))
        return true
    end
    return false
end

function reset_reservations!(table::ReservationTable)
    empty!(table.reservations.nzind)
    empty!(table.reservations.nzval)
    table
end

################################################################################
################################ Fat Path Tools ################################
################################################################################

# """
#     `get_fat_path(G,D,start_vtx,goal_vtx)`
#
#     returns a fat path through `G` from `start_vtx` to `goal_vtx`. Each set
#     of vertices in the fat path contains all vertices with distance d1 from
#     start_vtx and distance d2 to goal_vtx, where d1+d2 == the length of the
#     shortest path(s) from `start_vtx` to `goal_vtx`
#
#     G is a graph, D is the distance matrix
# """
# function get_fat_path(G,D,start_vtx::Int,goal_vtx::Int)
#     fat_path = Vector{Set{Int}}([Set{Int}(start_vtx)])
#     for i in 1:D[start_vtx,goal_vtx]
#         next_set = Set{Int}()
#         for src_vtx in fat_path[end]
#             for dst_vtx in outneighbors(G,src_vtx)
#                 if D[dst_vtx,goal_vtx] <= D[src_vtx,goal_vtx] - 1
#                     push!(next_set, dst_vtx)
#                 end
#             end
#         end
#         push!(fat_path, next_set)
#     end
#     fat_path
# end
#
# """
#     `add_fat_path_to_table(CAT,fat_path)`
# """
# function add_fat_path_to_table!(CAT,fat_path,t0=0)
#     for t in 1:length(fat_path)
#         idxs = collect(fat_path[t])
#         if t+t0 > 0
#             CAT[idxs,t+t0] .+= 1.0/length(idxs)
#         end
#     end
# end
#
# """
#     `populate_soft_lookup_table!(CAT,start_times,start_vtxs,goal_vtxs)`
# """
# function populate_soft_lookup_table!(CAT,G,D,start_vtxs,goal_vtxs,start_times=zeros(Int,length(start_vtxs)))
#     for (s,t,g) in zip(start_vtxs,start_times,goal_vtxs)
#         fat_path = get_fat_path(G,D,s,g)
#         add_fat_path_to_table!(CAT,t0,fat_path)
#     end
#     CAT
# end

const FatPath{N} = Vector{Vector{N}}

export
    FatPathCostModel,
    FlatFPCost,
    NormalizedFPCost

abstract type FatPathCostModel end
struct FlatFPCost <: FatPathCostModel end
struct NormalizedFPCost <: FatPathCostModel end

"""
    get_fat_path_cost(model,nodes)

Returns a scalar cost value depending on `typeof(model)` and `length(nodes)`.

```
get_fat_path_cost(m::FlatFPCost,nodes) = 1.0
get_fat_path_cost(m::NormalizedFPCost,nodes) = 1.0/length(nodes)
```
"""
get_fat_path_cost(m::FlatFPCost,nodes) = 1.0
get_fat_path_cost(m::NormalizedFPCost,nodes) = 1.0/length(nodes)

export get_mdd_graph

"""
    get_mdd_graph(env,s,threshold,cost=get_initial_cost(env))

Construct a multi-level decision diagram (MDD) graph.
"""
function get_mdd_graph(env,s,threshold,cost=get_initial_cost(env))
    # nodes = Vector{node_type(env)}()
    fat_path = NEGraph{DiGraph,cost_type(env),action_type(env),state_type(env)}()
    add_node!(fat_path,cost,s)
    frontier = PriorityQueue{state_type(env),typeof(cost)}()
    explored = Set{state_type(env)}()
    enqueue!(frontier,s=>cost)
    while !isempty(frontier)
        s,cost = dequeue_pair!(frontier)
        push!(explored,s)
        for a in get_possible_actions(env,s)
            sp = get_next_state(env,s,a)
            if has_vertex(fat_path,sp)
                add_edge!(fat_path,s,sp,a)
                continue
            end
            if sp in explored || has_vertex(fat_path,sp)
                continue
            end
            c = accumulate_cost(env,cost,get_transition_cost(env,s,a,sp))
            h = compute_heuristic_cost(env,c,sp)
            if h <= threshold
                add_node!(fat_path,c,sp)
                add_edge!(fat_path,s,sp,a)
                enqueue!(frontier, sp=>c)
            end
        end
    end
    fat_path
end

"""
    get_level_set_nodes(env,s,threshold,cost=get_initial_cost(env))

Returns a vector of `PathNode`s, where the heuristic cost (according to `env`)
of each node falls below `threshold`.
"""
function get_level_set_nodes(env,s,threshold,cost=get_initial_cost(env))
    # nodes = Vector{node_type(env)}()
    fat_path = FatPath{node_type(env)}()
    # frontier = PriorityQueue{state_type(env),typeof(cost)}()
    next_frontier = PriorityQueue{state_type(env),typeof(cost)}()
    explored = Set{state_type(env)}()
    # enqueue!(frontier,s=>cost)
    enqueue!(next_frontier,s=>cost)
    while !isempty(next_frontier)
        push!(fat_path,Vector{node_type(env)}())
        frontier = next_frontier
        next_frontier = PriorityQueue{state_type(env),typeof(cost)}()
        while !isempty(frontier)
            s,cost = dequeue_pair!(frontier)
            push!(explored,s)
            for a in get_possible_actions(env,s)
                sp = get_next_state(env,s,a)
                if sp in explored || haskey(next_frontier,sp)
                    continue
                end
                c = accumulate_cost(env,cost,get_transition_cost(env,s,a,sp))
                h = compute_heuristic_cost(env,c,sp)
                if h <= threshold
                    # push!(nodes,PathNode(s,a,sp))
                    push!(fat_path[end],PathNode(s,a,sp))
                    # enqueue!(frontier, sp=>c)
                    enqueue!(next_frontier, sp=>c)
                end
            end
        end
    end
    # nodes
    fat_path
end
function Base.empty!(sparr::SparseMatrixCSC)
    empty!(sparr.nzval)
    empty!(sparr.rowval)
    sparr.colptr .= 1
    return sparr
end
function Base.empty!(table::SoftConflictTable)
    empty!(table.CAT)
    for p in table.paths
        empty!(p)
    end
    return table
end

"""
    update_conflict_table!(table,nodes)

Updates a conflict table with a set of nodes.
"""
function update_fat_path_conflict_table!(model::M,table::AbstractMatrix,env::E,fat_path) where {M<:FatPathCostModel,E}
# function update_fat_path_conflict_table!(table::AbstractMatrix,env::E,nodes::Vector{N}) where {E,N<:PathNode}
    for nodes in fat_path
        cost = get_fat_path_cost(model,nodes)
        for n in nodes
            idx, t = serialize_jointly(env,get_a(n),get_t(get_sp(n)))
            table[idx,t] += cost
            idx, t = serialize_jointly(env,get_sp(n))
            table[idx,t] += cost
        end
    end
    return table
end
function update_fat_path_conflict_table!(model::M,table::SoftConflictTable,idx::Int,env::E,fat_path) where {M<:FatPathCostModel,E}
    update_fat_path_conflict_table!(model,table.paths[idx],env,fat_path)
    table.CAT .+= table.paths[idx]
    return table
end
# function update_fat_path_conflict_table!(table::Union{SoftConflictTable,AbstractMatrix},args...)
#     update_fat_path_conflict_table!(FlatFPCost(),table,args...)
# end
function clear_fat_path!(table::SoftConflictTable,idx::Int)
    table.CAT .-= table.paths[idx]
    empty!(table.paths[idx])
    table
end
reset_path!(table::SoftConflictTable,idx::Int) = clear_fat_path!(table,idx)
function SoftConflictTable(mapf::AbstractMAPF)
    # sets up SoftConflictTable for use with Serialized states
    N = num_states(mapf)+num_actions(mapf)
    T = num_states(mapf)*4
    return SoftConflictTable(
        paths = map(i->spzeros(Float64,N,T),1:num_agents(mapf)),
        CAT = spzeros(Float64,N,T),
    )
end
get_fat_path_threshold_cost(env,s,cost=get_initial_cost(env)) = compute_heuristic_cost(env,cost,s)
function populate_fat_path_table!(table::SoftConflictTable,mapf::AbstractMAPF,model=FlatFPCost())
    node = initialize_root_node(mapf)
    for i in 1:num_agents(mapf)
        env = build_env(mapf,node,i)
        s = get_start(mapf,i)
        threshold = get_fat_path_threshold_cost(env,s)
        fat_path = get_level_set_nodes(env,s,threshold)
        update_fat_path_conflict_table!(model,table,i,mapf,fat_path)
    end
    return table
end

function init_fat_path_mapf(mapf,model=FlatFPCost())
    table = SoftConflictTable(mapf)
    CRCBS.populate_fat_path_table!(table,mapf,model)
    mapf = MAPF(
        base_env_type(mapf)(
            graph = mapf.env.graph,
            cost_model = construct_composite_cost_model(
                SumOfTravelTime(),
                SoftConflictCost(table)
            ),
            heuristic = construct_composite_heuristic(
                PerfectHeuristic((v1,v2)->get_dist_matrix(mapf.env.graph)[v1,v2]),
                NullHeuristic()
            )
        ),
        mapf.starts,
        mapf.goals
    )
end
