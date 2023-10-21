# This module provides an interface to the Moving AI MAPF benchmark suite found
# at https://www.movingai.com/benchmarks/index.html
export BenchmarkInterface

module BenchmarkInterface

using GraphUtils
using TOML
using ..CRCBS

struct ScenarioAgentSpec
    start::Tuple{Int,Int}
    goal::Tuple{Int,Int}
end
struct MAPFScenario
    mapfile::String
    buckets::Vector{Vector{ScenarioAgentSpec}}
end

"""
    parse_map_file

Parses a .map file (see citation below) into an indicator grid. Each cell of the map is
encoded by one of the following characters:
    . - passable terrain
    G - passable terrain
    @ - out of bounds
    O - out of bounds
    T - trees (unpassable)
    S - swamp (passable from regular terrain)
    W - water (traversable, but not passable from terrain)

Returns an array of integers encoded as `IMPASSABLE=>1, FREE=>0` (we treat only
'G', 'S', and '.' as free).

@article{sturtevant2012benchmarks,
  title={Benchmarks for Grid-Based Pathfinding},
  author={Sturtevant, N.},
  journal={Transactions on Computational Intelligence and AI in Games},
  volume={4},
  number={2},
  pages={144 -- 148},
  year={2012},
  url = {http://web.cs.du.edu/~sturtevant/papers/benchmarks.pdf},
}
"""
function parse_map_file(filename,encoding=Dict('@'=>1,'O'=>1,'T'=>1,'W'=>1,'.'=>0,'G'=>0,'S'=>0))
    @assert splitext(filename)[end] == ".map"
    grid = zeros(Int,1,1)
    open(filename,"r") do io
        line = readline(io)
        if findfirst("type",line) == nothing
            throw(ErrorException("First line of map file should give type!"))
        end
        map_type = split(line)[end]
        if map_type != "octile"
            throw(ErrorException("Parser cannot handle non-octile maps!"))
        end
        line = readline(io)
        if findfirst("height",line) == nothing
            throw(ErrorException("Second line of map file should give height!"))
        end
        height = parse(Int,split(line)[end])
        line = readline(io)
        if findfirst("width",line) == nothing
            throw(ErrorException("Third line of map file should give width!"))
        end
        width = parse(Int,split(line)[end])
        @assert readline(io) == "map"
        grid = zeros(Int,height,width)
        for i in 1:height
            line = readline(io)
            for (j,c) in enumerate(line)
                if haskey(encoding,c)
                    grid[i,j] = encoding[c]
                else
                    throw(ErrorException("unrecognized character type $c"))
                end
            end
        end
    end
    return grid
end


"""
    parse_mapf_scenario(filename,map_path="")

Parses a .scen file into a set of 'buckets', where each bucket contains a list
of (start location,goal location) pairs. Each bucket can be used to instantiate
MAPF instances some (or all) of these pairs. The benchmarking approach proposed
on the benchmark website (https://www.movingai.com/benchmarks/index.html) is to
start with a 2-agent MAPF for each bucket, and increase the number of agents
until solver time out.
"""
function parse_mapf_scenario(filename,map_path="")
    @assert splitext(filename)[end] == ".scen"
    buckets = Dict{Int,Vector{ScenarioAgentSpec}}()
    mapfile = ""
    open(filename,"r") do io
        for (i,line) in enumerate(eachline(io))
            if i == 1
                if findfirst("version", line) != nothing
                    continue
                end
            end
            split_line = split(line)
            bucket = parse(Int,split_line[1])
            mapfile = split_line[2]
            map_width = parse(Int,split_line[3])
            map_height = parse(Int,split_line[4])
            start_x = parse(Int,split_line[5]) + 1
            start_y = parse(Int,split_line[6]) + 1
            goal_x = parse(Int,split_line[7]) + 1
            goal_y = parse(Int,split_line[8]) + 1
            optimal_length = parse(Float64,split_line[9])
            if !haskey(buckets,bucket)
                buckets[bucket] = valtype(buckets)()
            end
            push!(buckets[bucket],ScenarioAgentSpec(
                (start_x,start_y),
                (goal_x,goal_y)
            ))
        end
    end
    if isfile(map_path)
        mapfile = map_path
    else
        mapfile = joinpath(map_path,mapfile)
    end
    @assert isfile(mapfile)
    return MAPFScenario(
        mapfile,
        map(k->buckets[k],sort(collect(keys(buckets))))
    )
end

function generate_cbs_env(indicator_grid)
    CBSEnv.LowLevelEnv(graph=construct_factory_env_from_indicator_grid(indicator_grid))
end
function generate_state_map(env::GridFactoryEnvironment)
    (x,y) -> CBSEnv.State(env.vtx_map[x,y],0)
end

function construct_base_mapf(scen::MAPFScenario,env_generator=generate_cbs_env)
    grid = parse_map_file(scen.mapfile)
    env = env_generator(grid)
    return MAPF(env,Vector{state_type(env)}(),Vector{state_type(env)}())
end

function gen_mapf_problem_from_scenario(mapf::M,
        scen::MAPFScenario,
        bucket_id::Int,
        n_agents::Int,
        state_map=generate_state_map(mapf.env.graph),
        ) where {M<:MAPF}
    @assert(length(scen.buckets) >= bucket_id,
        "bucket_id $bucket_id out of range!")
    bucket = scen.buckets[bucket_id]
    @assert(length(bucket) >= n_agents,
        string("bucket only has $(length(bucket)) agents.",
        " Cannot instantiate a problem with $n_agents agents."))
    starts = Vector{state_type(mapf)}()
    goals = Vector{state_type(mapf)}()
    for i in 1:n_agents
        push!(starts, state_map(bucket[i].start...))
        push!(goals, state_map(bucket[i].goal...))
    end
    M(mapf.env,starts,goals)
end



"""
    MovingAIBenchmarkFile

Points to a TOML-formatted file that contains the following elements:
    scenario = "/path/to/scenario/file.scen"
    map_file = "/path/to/map/file.map"
    bucket_idx = 1 # an integer
    n_agents = 2 # an integer

Extension is .bm
"""
struct MovingAIBenchmarkFile
    path::String
end

function pad_file_number(id,pad=4)
    # hacky 0 padding to avoid dependency on Sprintf
    padding = prod(map(i->"0",1:(pad-length(string(id)))))
    return "$(padding)$(id)"
end

function generate_problem_files_from_moving_ai(
        scenario_paths,
        base_map_path,
        problem_dir
        ;
        verbose=true,
        pad=6,
        recursive=false
        )
    problem_id = 1
    mkpath(problem_dir)
    for scen_file in scenario_paths
        scenario = parse_mapf_scenario(scen_file,base_map_path)
        for (bucket_idx,bucket) in enumerate(scenario.buckets)
            for n_agents in 2:length(bucket)
                config_dict = Dict(
                    "scenario" => scen_file,
                    "map_file" => scenario.mapfile,
                    "bucket_idx" => bucket_idx,
                    "n_agents" => n_agents,
                )
                problem_filename = joinpath(
                    problem_dir,"problem_$(pad_file_number(problem_id,pad)).bm")
                if isfile(problem_filename)
                    if verbose
                        println("File ",problem_filename," already exists.",
                            " Skipping...")
                    end
                else
                    open(problem_filename,"w") do io
                        TOML.print(io,config_dict)
                    end
                end
                problem_id += 1
            end
        end
    end
    return true
end

"""
    ProblemLoader

Can be queried via `get_problem(iterator,problem_filename)` to return MAPF
instances.
"""
struct ProblemLoader{M}
    scenarios::Dict{String,MAPFScenario}
    base_mapfs::Dict{String,M}
    ProblemLoader() = new{MAPF}(
        Dict{String,MAPFScenario}(),
        Dict{String,MAPF}()
    )
end
function get_scenario!(loader::ProblemLoader,problem_config::Dict)
    scen_file   = problem_config["scenario"]
    map_file    = problem_config["map_file"]
    if !haskey(loader.scenarios,scen_file)
        loader.scenarios[scen_file] = parse_mapf_scenario(scen_file, map_file)
    end
    return loader.scenarios[scen_file]
end
function get_base_mapf!(loader::ProblemLoader,problem_config::Dict)
    scen_file   = problem_config["scenario"]
    map_file    = problem_config["map_file"]
    scenario = get_scenario!(loader,problem_config)
    if !haskey(loader.base_mapfs,map_file)
        loader.base_mapfs[map_file] = construct_base_mapf(scenario)
    end
    return loader.base_mapfs[map_file]
end

struct LoaderState
    probfile_idx::Int
end

function init_mapf_loader(problem_dir)
    loader = ProblemLoader()
    for problem_file in readdir(problem_dir;join=true)
        problem_config = TOML.parsefile(problem_file)
        get_scenario!(loader,problem_config)
        get_base_mapf!(loader,problem_config)
    end
    return loader
end

function CRCBS.load_problem(loader::ProblemLoader,probfile)
    problem_config = TOML.parsefile(probfile)
    scenario = get_scenario!(loader,problem_config)
    base_mapf = get_base_mapf!(loader,problem_config)
    bucket_idx  = problem_config["bucket_idx"]
    n_agents    = problem_config["n_agents"]
    mapf = gen_mapf_problem_from_scenario(
        base_mapf, scenario, bucket_idx, n_agents)
    return mapf
end

"""
    is_same_scenario_and_bucket(config1,config2)

Checks that two problem_configs are on the same scenario and bucket.
"""
function is_same_scenario_and_bucket(config1,config2)
    for k in ["scenario","bucket_idx"]
        if !(haskey(config1,k) && haskey(config2,k))
            return false
        end
        if !(config1[k] == config2[k])
            return false
        end
    end
    return true
end

"""
    profile_with_skipping!(config,loader)

Follow the Moving AI benchmarking paradigm, where problems are skipped if the
solver has already failed on the same problem instance with fewer agents.
"""
function profile_with_skipping!(config,loader)
    for solver_config in config.solver_configs
        mkpath(solver_config.results_path)
    end
    previous_config = Dict{String,Any}()
    for problem_file in readdir(config.problem_dir;join=true)
        problem_name = splitext(splitdir(problem_file)[end])[1]
        problem_config = TOML.parsefile(problem_file)
        # Reset all solvers if we have moved to a new scenario or bucket
        if !is_same_scenario_and_bucket(problem_config,previous_config)
            previous_config = problem_config
            for solver_config in config.solver_configs
                reset_solver!(solver_config.solver)
            end
        end
        scenario = get_scenario!(loader,problem_config)
        base_mapf = get_base_mapf!(loader,problem_config)
        mapf = load_problem(loader,problem_file)
        for solver_config in config.solver_configs
            solver = solver_config.solver
            outfile = joinpath(solver_config.results_path,string(problem_name,".results"))
            if isfile(outfile)
                @log_info(-1,verbosity(solver),"Results already stored for ",problem_name,
                    " at ", outfile)
                continue
            end
            if failed_status(solver)
                @log_info(1,verbosity(solver),"Skipping problem ",problem_name)
                continue # Keep skipping until new bucket
            end
            solution, timer_results = profile_solver!(solver,mapf)
            results_dict = compile_results(
                solver,
                config.feats,
                mapf,
                solution,
                timer_results
                )
            results_dict["problem_file"] = problem_file
            open(outfile,"w") do io
                TOML.print(io,results_dict)
            end
        end
    end
end

end # moduler
