using TOML
using DataFrames
using TaskGraphs
Revise.includet(joinpath(pathof(TaskGraphs),"..","helpers","render_tools.jl"))

base_graphics_path = "/scratch/Repositories/Sandbox/thesis_graphics/LEGO"

xkeys = [Dict(:model=>n) for n in [
    "tractor.mpd",
    "X-wingMini.mpd",
    "X-wingFighter.mpd",
    "StarDestroyer.mpd",
    "Saturn.mpd",
]]
ykeys = [Dict(:metric=>n) for n in [
    "numobjects",
    "numassemblies",
    "numrobots",
    "PreExecutionRuntime",
    "ConfigTransportUnitsTime",
    "StagingPlanTime",
    "AssigmentTime",
    "OptimisticMakespan",
    "ExecutionRuntime",
    "Makespan",
]]
df = DataFrame(Dict(
    "model"=>[],
    "rvo"=>[],
    "greedy"=>[],
    "numobjects"=>[],
    "numassemblies"=>[],
    "numrobots"=>[],
    "PreExecutionRuntime"=>[],
    "AssigmentTime"=>[],
    "ConfigTransportUnitsTime"=>[],
    "StagingPlanTime"=>[],
    "OptimisticMakespan"=>[],
    "ExecutionRuntime"=>[],
    "Makespan"=>[],
    ))

name_key = Dict(
    "tractor.mpd"=>"Tractor",
    "X-wingMini.mpd"=>"X-Wing Mini",
    "X-wingFighter.mpd"=>"X-Wing",
    "StarDestroyer.mpd"=>"Destroyer",
    "Saturn.mpd"=>"Saturn V",
    "numobjects"=>      "Num. Parts",
    "numassemblies"=>   "Num. Assemblies",
    "numrobots"=>       "Num. Robots",
    "PreExecutionRuntime"=>"Preprocessing Time (s)",
    "AssigmentTime"=>"Assignment Time (s)",
    "ConfigTransportUnitsTime"=>"T.U. Config Time (s)",
    "StagingPlanTime"=>"Staging Plan Time (s)",
    "OptimisticMakespan"=>"Predicted Makespan (s)",
    "ExecutionRuntime"=>"Execution Runtime (s)",
    "Makespan"=>"Makespan (s)",
)

for dict in xkeys
    mod_name = dict[:model]
    for prefix_a in ["greedy_","optimal_"]
        for prefix_b in ["with_rvo","without_rvo"]
            results_dict = Dict()
            results_dict["model"] = mod_name
            results_dict["greedy"] = (prefix_a == "greedy_")
            results_dict["rvo"] = (prefix_b == "with_rvo")
            model_path = joinpath(mod_name,string(prefix_a,prefix_b))
            stats_path = joinpath(base_graphics_path,model_path,"stats.toml")
            if isfile(stats_path)
                toml_dict = TOML.parsefile(stats_path)
            else
                toml_dict = Dict()
            end
            for (i,dict) in enumerate(ykeys) 
                get!(toml_dict,dict[:metric],"--")
            end
            merge!(results_dict,toml_dict)
            push!(df,results_dict;cols=:intersect)
        end
    end
end

tables = Dict()
tab = init_table(xkeys,
    [Dict(:metric=>n) for n in [
        "numobjects",
        "numassemblies",
        "numrobots",
        ]])
fill!(tab.data,"--")
fill_tab_from_columns!(tab,df;xkey=:model,ykey=:metric,
        f = vals -> get(filter(v->isa(v,Real),vals),1,nothing),
    )
tables["regular"] = tab

tab = init_table(xkeys,
    [Dict(:metric=>n) for n in [
        "ConfigTransportUnitsTime",
        "StagingPlanTime",
        "AssigmentTime",
        "PreExecutionRuntime",
        ]])
fill!(tab.data,"--")
fill_tab_from_columns!(tab,df;
    xkey=:model,
    ykey=:metric,
    include_keys = [:greedy=>true],
    f = vals -> begin 
        fvals = filter(v->isa(v,Real),vals)
        isempty(fvals) ? nothing : minimum(fvals)
    end
)
tables["greedy_all"] = tab

for val in (true,false)
    if val == true
    tab = tables[string("greedy","_",val)] = init_table(xkeys,
        [Dict(:metric=>n) for n in [
            "AssigmentTime",
            # "PreExecutionRuntime",
            "OptimisticMakespan",
            ]]
        )
    else
    tab = tables[string("greedy","_",val)] = init_table(xkeys,
        [Dict(:metric=>n) for n in [
            # "PreExecutionRuntime",
            "AssigmentTime",
            "OptimisticMakespan",
            ]]
        )
    end
    fill_tab_from_columns!(tab,df;
        xkey=:model,
        ykey=:metric,
        include_keys = [:greedy=>val],
        f = vals -> begin 
            fvals = filter(v->isa(v,Real),vals)
            isempty(fvals) ? nothing : minimum(fvals)
        end
        )
end
for val in (true,)
    tab = tables["execution"] = init_table(xkeys,
    [Dict(:metric=>n) for n in [
        "ExecutionRuntime",
        "OptimisticMakespan",
        "Makespan",
        ]]
        )
    fill_tab_from_columns!(tab,df;
        xkey=:model,
        ykey=:metric,
        include_keys = [:rvo=>val],
        f = vals -> begin 
            fvals = filter(v->isa(v,Real),vals)
            isempty(fvals) ? nothing : minimum(fvals)
        end
        )
    tab = tables["predicted_makespan"] = init_table(xkeys,
    [Dict(:metric=>n) for n in [
        "OptimisticMakespan",
        ]]
        )
    fill_tab_from_columns!(tab,df;
        xkey=:model,
        ykey=:metric,
        include_keys = [:rvo=>val],
        f = vals -> begin 
            fvals = filter(v->isa(v,Real),vals)
            isempty(fvals) ? nothing : minimum(fvals)
        end
        )
end
for val in (true,false)
    tab = tables[string("rvo","_",string(val))] = init_table(xkeys,
    [Dict(:metric=>n) for n in [
        "ExecutionRuntime",
        "Makespan",
        ]]
        )
    fill_tab_from_columns!(tab,df;
        xkey=:model,
        ykey=:metric,
        include_keys = [:rvo=>val],
        f = vals -> begin 
            fvals = filter(v->isa(v,Real),vals)
            isempty(fvals) ? nothing : minimum(fvals)
        end
        )
end
for tab_name in collect(keys(tables))
    tab = tables[tab_name] = transpose(tables[tab_name])
    for i in 1:size(tab,1)
        for j in 1:size(tab,2)
            if tab.data[i,j] === nothing || isinf(tab.data[i,j])
                tab.data[i,j] = "--"
            end
        end
    end
    # for dict in get_xkeys(tab)
    #     for k in collect(keys(dict))
    #         dict[k] = get(name_key,dict[k],dict[k])
    #     end
    # end
end

for (k,tab) in tables
    outpath = joinpath(base_graphics_path,"demo_statistics_tables")
    mkpath(outpath)
    path = joinpath(outpath,string(k,".tex"))
    write_tex_table(path,tab;
        print_func = print_real,
        print_header=false,
        print_footer=false,
        print_column_labels=false,
        col_label_func=(k,v)->"\\textbf{$(name_key[v])}",
        row_label_func=(k,v)->"$(name_key[v])",
        colspec="l ",
        initial_colspec="@{}l",
        terminal_spec="@{}",
        group_delim = " ",
        precision=2,
    )
end
