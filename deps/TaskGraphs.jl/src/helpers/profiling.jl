
CRCBS.extract_feature(solver::NBSSolver,::LowLevelIterationCount, args...) = max_iterations(route_planner(solver))

export
    ObjectPathSummaries,
    TaskAssignmentDict

struct ObjectPathSummaries <: FeatureExtractor{Dict{Int,Vector{}}} end
function get_object_path_summaries end
function CRCBS.extract_feature(solver,::ObjectPathSummaries,pc_mapf,env,timer_results)
    summaries = get_object_path_summaries(env)
    to_string_dict(to_string_dict(Dict(k=>to_string_dict(v) for (k,v) in summaries)))
end
struct TaskAssignmentDict <: FeatureExtractor{Dict{Int,Vector{}}} end
function CRCBS.extract_feature(solver,::TaskAssignmentDict,pc_tapf,env,timer_results)
    assignment_dict = get_assignment_dict(get_schedule(env))
    to_string_dict(assignment_dict)
    # dict = Dict
end

extract_object_data(node,t0,tF) = Dict{Symbol,Union{Vector{Int},Int}}()
extract_object_data(node::OBJECT_AT,t0,tF) = Dict{Symbol,Union{Vector{Int},Int}}(
    :object_id=>get_id(get_object_id(node)),
    :start_vtx=>get_id(get_initial_location_id(node)),
    :start_time=>t0,
    )
extract_object_data(node::COLLECT,t0,tF) = Dict{Symbol,Union{Vector{Int},Int}}(
    :object_id=>get_id(get_object_id(node)),
    :start_vtx=>get_id(get_initial_location_id(node)),
    :robot_ids=>map(get_id,get_valid_robot_ids(node)),
    :collect_time=>tF,
    )
extract_object_data(node::CARRY,t0,tF) = Dict{Symbol,Union{Vector{Int},Int}}(
    :object_id=>get_id(get_object_id(node)),
    :start_vtx=>get_id(get_initial_location_id(node)),
    :end_vtx=>get_id(get_destination_location_id(node)),
    :robot_ids=>map(get_id,get_valid_robot_ids(node)),
    :collect_time=>t0,
    )
extract_object_data(node::DEPOSIT,t0,tF) = Dict{Symbol,Union{Vector{Int},Int}}(
    :object_id=>get_id(get_object_id(node)),
    :end_vtx=>get_id(get_destination_location_id(node)),
    :robot_ids=>map(get_id,get_valid_robot_ids(node)),
    :deposit_time=>tF,
    )
function get_object_path_summaries(env::SearchEnv)
    summaries = Dict{Int,Dict{Symbol,Union{Vector{Int},Int}}}()
    for v in vertices(get_schedule(env))
        node = get_node(get_schedule(env),v)
        t0 = Int(round(get_t0(node)))
        tF = Int(round(get_tF(node)))
        @assert abs(t0-get_t0(node)) <= 0.01
        @assert abs(tF-get_tF(node)) <= 0.01
        dict = extract_object_data(node.node,t0,tF)
        if haskey(dict,:object_id)
            object_id = dict[:object_id]
            merge!(get!(summaries,object_id,valtype(summaries)()),dict)
        end
    end
    summaries
end

function reconstruct_object_paths(robot_paths,object_path_summaries)
    object_paths = Dict{Int,Vector{Int}}()
    object_intervals = Dict{Int,Tuple{Int,Int}}()
    for object_summary in values(object_path_summaries)
        id = object_summary[:object_id]
        t0 = object_summary[:start_time]
        tC = object_summary[:collect_time]
        tF = object_summary[:deposit_time]
        v0 = object_summary[:start_vtx]
        vF = object_summary[:end_vtx]
        robot_ids = object_summary[:robot_ids]
        if robot_ids[1] != -1
            T = length(robot_paths[robot_ids[1]])
            if T >= tC-1
                p1 = map(t->v0, 0:tC-1)
                p2 = robot_paths[robot_ids[1]][tC+1:min(tF+1,T)]
                object_paths[id] = vcat(p1,p2)
                object_intervals[id] = (t0,tF)
            end
        end
    end
    return object_paths, object_intervals
end

export SolutionAdjacencyMatrix
struct SolutionAdjacencyMatrix <: FeatureExtractor{Dict{String,Any}} end
function CRCBS.extract_feature(solver,::SolutionAdjacencyMatrix,pc_tapf,env,timer_results)
    mat = adjacency_matrix(get_graph(get_schedule(env)))
    I,J,_ = findnz(mat)
    return Dict("edge_list" => map(idx->[idx...], zip(I,J)),
        "size_i" => size(mat,1),
        "size_j" => size(mat,2)
        )
end

export
    TaskGraphsProblemLoader,
    add_env!,
    get_env!,
    problem_type

"""
    TaskGraphsProblemLoader{T}

Helper cache for loading problems of type `T`
"""
struct TaskGraphsProblemLoader{T}
    envs::Dict{String,GridFactoryEnvironment}
    prob_specs::Dict{String,ProblemSpec}
    TaskGraphsProblemLoader{T}() where {T} = new{T}(
        Dict{String,GridFactoryEnvironment}(),
        Dict{String,ProblemSpec}()
    )
end
function add_env!(loader::TaskGraphsProblemLoader,env_id::String,
    env=read_env(env_id))
    if haskey(loader.envs,env_id)
    else
        loader.envs[env_id] = env
        # loader.prob_specs[env_id] = ProblemSpec(D=get_dist_matrix(env))
        loader.prob_specs[env_id] = ProblemSpec(D=env)
    end
    loader
end
function get_env!(loader::TaskGraphsProblemLoader,env_id::String)
    if !haskey(loader.envs,env_id)
        loader.envs[env_id] = read_env(env_id)
    end
    return loader.envs[env_id]
end
problem_type(::TaskGraphsProblemLoader{T}) where {T} = T

export
    PCTAPF_Loader,
    PCTA_Loader,
    PCMAPF_Loader,
    ReplanningProblemLoader

const PCTAPF_Loader = TaskGraphsProblemLoader{PC_TAPF}
const PCMAPF_Loader = TaskGraphsProblemLoader{PC_MAPF}
const PCTA_Loader   = TaskGraphsProblemLoader{PC_TA}
const ReplanningProblemLoader = TaskGraphsProblemLoader{RepeatedPC_TAPF}

export write_problem

"""
    write_problem(loader::TaskGraphsProblemLoader,problem_def,prob_path,env_id="")

Write a problem that can later be loaded and solved.
"""
function write_problem(loader::TaskGraphsProblemLoader,problem_def,prob_path,env_id="")
    if isdir(prob_path)
        prob_path = joinpath(prob_path,"problem.toml")
    end
    toml_dict = TOML.parse(problem_def)
    toml_dict["env_id"] = env_id
    open(prob_path, "w") do io
        TOML.print(io, toml_dict)
    end
    prob_path
end

init_random_problem(loader::TaskGraphsProblemLoader,env,config) = random_pctapf_def(env,config)
init_random_problem(loader::PCMAPF_Loader,env,config) = random_pcmapf_def(env,config)
function write_problems!(loader::TaskGraphsProblemLoader,config::Dict,base_path::String,prob_counter::Int=1)
    env = get_env!(loader,config[:env_id])
    num_trials = get(config,:num_trials,1)
    for i in 1:num_trials
        prob_name = padded_problem_name(prob_counter,"problem","")
        prob_path = joinpath(base_path,prob_name)
        mkpath(prob_path)
        open(joinpath(prob_path,"config.toml"),"w") do io
            TOML.print(io,config)
        end
        prob_def = init_random_problem(loader,env,config)
        write_problem(loader,prob_def,prob_path,config[:env_id])
        prob_counter += 1
    end
    return prob_counter
end
function write_problems!(loader::TaskGraphsProblemLoader,configs::Vector{D},base_path::String,prob_counter::Int=1) where {D<:Dict}
    for config in configs
        prob_counter = write_problems!(loader,config,base_path,prob_counter)
    end
    return prob_counter
end

"""
    write_pcmapf_from_pctapf!(loader,solver_config,pcmapf_dir)

Copy assignments from `PCTAPF` or `PCTA` results into problems in `pcmapf_dir`. 
"""
function write_pcmapf_from_pctapf!(loader,solver_config,pcmapf_dir)
    for prob_path in readdir(pcmapf_dir;join=true)
        is_problem_file(loader,prob_path) ? nothing : continue
        results_path = get_results_path(loader,solver_config,prob_path)
        isfile(results_path) || isdir(results_path) ? nothing : continue 
        results_path = get_results_path(loader,solver_config,prob_path)
        results_dict = load_results(loader,results_path)
        if haskey(results_dict,:TaskAssignmentDict)
            toml_dict = Dict("assignments"=>results_dict[:TaskAssignmentDict])
            open(joinpath(prob_path,"problem.toml"), "a") do io # append
                TOML.print(io, toml_dict)
            end
        else
            @warn "assignments not stored in results at $(results_path)."
            @show results_dict
        end
    end
end

"""
    CRCBS.load_problem(loader::TaskGraphsProblemLoader,solver_config,prob_path)

Currently only impemented for PC_TAPF and PC_TA
"""
function CRCBS.load_problem(loader::TaskGraphsProblemLoader,solver_config,prob_path)
    if !isfile(prob_path)
        prob_path = joinpath(prob_path,"problem.toml")
    end
    toml_dict = TOML.parsefile(prob_path)
    def = read_problem_def(toml_dict)
    env_id = toml_dict["env_id"]
    env = get_env!(loader,env_id)
    problem_type(loader)(solver_config.solver,def,env)
end
function CRCBS.load_problem(loader::PCTA_Loader,solver_config,prob_path)
    if !isfile(prob_path)
        prob_path = joinpath(prob_path,"problem.toml")
    end
    toml_dict = TOML.parsefile(prob_path)
    def = read_problem_def(toml_dict)
    env_id = toml_dict["env_id"]
    env = get_env!(loader,env_id)
    problem_type(loader)(def,env,solver_config.objective)
end
function CRCBS.load_problem(loader::PCMAPF_Loader,solver_config,prob_path)
    if !isfile(prob_path)
        prob_path = joinpath(prob_path,"problem.toml")
    end
    toml_dict = TOML.parsefile(prob_path)
    def = read_pcmapf_problem_def(toml_dict)
    env_id = toml_dict["env_id"]
    env = get_env!(loader,env_id)
    problem_type(loader)(solver_config.solver,def,env)
end
function is_problem_file(loader::PCTA_Loader,path)
    isfile(joinpath(path,"problem.toml")) && isfile(joinpath(path,"config.toml"))
end
function is_problem_file(loader::PCTAPF_Loader,path)
    isfile(joinpath(path,"problem.toml")) && isfile(joinpath(path,"config.toml"))
end
function is_problem_file(loader::PCMAPF_Loader,path)
    isfile(joinpath(path,"problem.toml")) && isfile(joinpath(path,"config.toml"))
end


CRCBS.extract_feature(solver,::SolutionCost,  mapf,solution::OperatingSchedule,timer_results) = best_cost(solver)
get_solver_name(solver_config) = split(solver_config.results_path,"/")[end]
"""
    CRCBS.run_profiling(loader::TaskGraphsProblemLoader,solver_config,problem_dir)

Run profiling with a `TaskGraphsProblemLoader`.
"""
function CRCBS.run_profiling(loader::TaskGraphsProblemLoader,solver_config,problem_dir)
    solver = solver_config.solver
    for prob_path in readdir(problem_dir;join=true)
        is_problem_file(loader,prob_path) ? nothing : continue
        results_path = get_results_path(loader,solver_config,prob_path)
        isfile(results_path) || isdir(results_path) ? continue : nothing
        prob = load_problem(loader,solver_config,prob_path)
        solution, timer_results = profile_solver!(solver,prob)
        @info "$(get_solver_name(solver_config)) solved $(split(prob_path,"/")[end]) in $(timer_results.t) seconds"
        results_dict = compile_results(
            solver,
            solver_config.feats,
            prob,
            solution,
            timer_results
            )
        results_dict["problem_file"] = prob_path
        write_results(loader,solver_config,prob,prob_path,results_dict)
    end
end

export warmup

"""
    warmup(loader::TaskGraphsProblemLoader,solver_config,problem_dir,dummy_path = "dummy_path")

Do a small dry run of `run_profiling(loader,solver_config,problem_dir)` to 
ensure that all code is fully compiled before collecting results.
"""
function warmup(loader::TaskGraphsProblemLoader,solver_config,problem_dir,dummy_path = "dummy_path")
    solver = solver_config.solver
    for prob_path in readdir(problem_dir;join=true)
        is_problem_file(loader,prob_path) ? nothing : continue
        prob = load_problem(loader,solver_config,prob_path)
        solution, timer_results = profile_solver!(solver,prob)
        results_dict = compile_results(
            solver,
            solver_config.feats,
            prob,
            solution,
            timer_results
            )
        results_dict["problem_file"] = dummy_path
        # write_results(loader,solver_config,prob,prob_path,results_dict)
        break
    end
    run(`rm -rf $dummy_path`)
    return loader
end
