export MetaAgentCBS_Solver

"""
    MetaAgentCBS_Solver

Path planner that employs Meta Agent Conflict-Based Search
"""
@with_kw struct MetaAgentCBS_Solver{L,C} <: AbstractCBSSolver
    low_level_planner::L    = VanillaAStar()
    logger::SolverLogger{C} = SolverLogger{cost_type(low_level_planner)}()
    beta::Int               = 1
end
MetaAgentCBS_Solver(planner) = MetaAgentCBS_Solver(low_level_planner=planner)

export MetaSolution

"""
    MetaSolution{S}

Wrapper for a LowLevelSolution that allows for keeping track of groups.
"""
struct MetaSolution{S}
    solution::S
    group_idxs::Vector{Vector{Int}}
end
MetaSolution(mapf::AbstractMAPF) = MetaSolution(
    get_initial_solution(mapf),
    map(i->[i],1:num_agents(mapf))
    )
for op in [
    :state_type,
    :action_type,
    :cost_type,
    :get_paths,
    :get_path_costs,
    :get_cost,
    :get_cost_model,
    :set_solution_path!,
    :set_path_cost!,
    :set_cost!,
    :is_consistent,
    :is_valid,
    ]
    @eval $op(s::MetaSolution,args...) = $op(s.solution,args...)
end
Base.copy(solution::L) where {L<:MetaSolution} = L(
    copy(solution.solution),
    deepcopy(solution.group_idxs)
    )

function initialize_root_node(solver::MetaAgentCBS_Solver,mapf::AbstractMAPF,solution=MetaSolution(mapf))
    initialize_root_node(mapf,solution)
end

"""
    combine_agents(conflict_table, groups::Vector{Vector{Int}})

Helper for merging two (meta) agents into a meta-agent
"""
function combine_agents!(solver, node)
    groups = node.solution.group_idxs
    N = length(groups)
    conflict_counts = zeros(Int,N,N)
    for (i,idxs1) in enumerate(groups)
        for (j,idxs2) in enumerate(groups[i:end])
            conflict_counts[i,j] += count_conflicts(node.conflict_table,idxs1,idxs2)
        end
    end
    idx = argmax(conflict_counts).I
    i = minimum(idx)
    j = maximum(idx)
    if conflict_counts[i,j] > solver.beta
        # @show i,j,groups[i],groups[j],conflict_counts[i,j]
        @assert i != j
        @log_info(1,verbosity(solver),"Conflict Limit exceeded. Merging agents ", groups[i], " and ", groups[j],"\n")
        # groups = deepcopy(groups)
        groups[i] = [groups[i]..., groups[j]...]
        deleteat!(groups, j)
        # node.groups = groups
        # node.solution.group_idxs = groups
        return groups, i
    end
    return groups, -1
end

function get_group_index(groups, agent_idx)
    group_idx = -1
    for i in 1:length(groups)
        if agent_idx in groups[i]
            group_idx = i
            break
        end
    end
    return group_idx
end
function get_group_index(solution::MetaSolution, agent_idx)
    get_group_index(solution.group_idxs,agent_idx)
end

function cbs_bypass!(solver::MetaAgentCBS_Solver,mapf,node,priority_queue)
    groups, group_idx = combine_agents!(solver, node)
    @assert groups == node.solution.group_idxs
    if group_idx > 0 # New Meta Agent has been constructed
        new_node = initialize_child_search_node(solver, mapf, node)
        consistent_flag = low_level_search!(solver, mapf, new_node, [group_idx])
        if consistent_flag # is_valid(new_node.solution, mapf)
            # for agent_id in groups[group_idx]
            #     detect_conflicts!(new_node.conflict_table,new_node.solution,agent_id)
            # end
            detect_conflicts!(new_node,[group_idx])
            for i in groups[group_idx]
                for j in groups[group_idx]
                    @assert count_conflicts(new_node,i,j) == 0 "$i,$j,$(count_conflicts(node,i,j))"
                end
            end
            enqueue!(priority_queue, new_node => get_cost(new_node))
        end
        return true
    end
    return false
end

# function CRCBS.low_level_search!(solver::MetaAgentCBS_Solver,mapf,node,group_idxs=collect(1:length(node.solution.group_idxs)))
#     # group_idxs = union(map(i->get_group_index(node.solution,i), idxs))
#     low_level_search!(low_level(solver),mapf,node,group_idxs)
# end

function detect_conflicts!(conflict_table,solution::MetaSolution,
        group_idxs=collect(1:length(solution.group_idxs)),
        args...;
        kwargs...
    )
    for group_idx in group_idxs
        for idx in solution.group_idxs[group_idx]
            detect_conflicts!(conflict_table,solution.solution,idx,args...;kwargs...)
        end
    end
end

solve!(solver::MetaAgentCBS_Solver, mapf) = cbs!(solver,mapf)
