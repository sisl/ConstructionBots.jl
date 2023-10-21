import Cairo #, Fontconfig
using GraphPlottingBFS
using Compose
using Colors
# using Gadfly
using DataFrames
using GraphUtils
using FactoryRendering

const FR = FactoryRendering

using PGFPlotsX
latexengine!(PGFPlotsX.PDFLATEX)
push!(PGFPlotsX.CUSTOM_PREAMBLE,"\\usetikzlibrary{backgrounds}")
push!(PGFPlotsX.CUSTOM_PREAMBLE,"\\tikzset{background rectangle/.style={fill=white,}}")
push!(PGFPlotsX.CUSTOM_PREAMBLE,"\\tikzset{use background/.style={show background rectangle,}}")
using PGFPlots
using Printf
using Parameters
using Measures
using Graphs
using MetaGraphs

ROBOT_COLOR = RGB(0.1,0.5,0.9)

interpolate(a,b,t) = (1 - t)*a + t*b

function interp_position(path::Vector{Int},vtxs,t)
    t1 = Int(floor(t))+1
    idx1 = path[max(1,min(t1,length(path)))]
    idx2 = path[max(1,min(t1+1,length(path)))]
    x=interpolate(vtxs[idx1][1],vtxs[idx2][1],t-(t1-1))
    y=interpolate(vtxs[idx1][2],vtxs[idx2][2],t-(t1-1))
    return x,y
end
function interp_position(paths::Vector{Vector{Int}},vtxs,t)
    coords = map(p->interp_position(p,vtxs,t),paths)
end
function interp_path(path::Vector{Int},vtxs,t)
    x,y = interp_position(path,vtxs,t)
    t1 = Int(floor(t))+1
    idx1 = max(1,min(t1,length(path)))
    idx2 = max(1,min(t1+1,length(path)))
    xpath = [x, map(v->v[1],vtxs[path[idx2:end]])...]
    ypath = [y, map(v->v[2],vtxs[path[idx2:end]])...]
    return xpath,ypath
end

function gen_object_colors(n)
  cs = distinguishable_colors(n,
      [colorant"#FE4365", colorant"#eca25c"], # seed colors
      lchoices=Float64[58, 45, 72.5, 90],     # lightness choices
      transform=c -> deuteranopic(c, 0.1),    # color transform
      cchoices=Float64[20,40],                # chroma choices
      hchoices=[75,51,35,120,180,210,270,310] # hue choices
  )

  convert(Vector{Color}, cs)
end

function record_video(outfile_name,render_function,
        save_function = (t,path,s)-> render_function(t) |> PNG(path,s[1],s[2])
    ;
        t0 = 0,
        tf = 10,
        dt = 0.25,
        t_history=t0:dt:tf,
        fps = 10,
        ext = "png",
        s = (Compose.default_graphic_width,Compose.default_graphic_height),
        res1=1080,
        res = "$(res1)x$(Int(round(res1*s[2]/s[1])))",
    )
    tmpdir = mktempdir()
    for (i,t) in enumerate(t_history)
        filename = joinpath(tmpdir,string(i,".",ext))
        # render_function(t) |> PNG(filename, s[1], s[2])
        save_function(t,filename,s)# |> PNG(filename, s[1], s[2])
    end
    outfile = outfile_name
    run(pipeline(`ffmpeg -y -r $fps -f image2 -i $tmpdir/%d.$ext -crf 20 -s $res $outfile`, stdout=devnull, stderr=devnull))
    run(pipeline(`rm -rf $tmpdir`, stdout=devnull, stderr=devnull))
end

function get_grid_world_layer(vtxs,color,cw)
    if length(vtxs) > 0
        return layer(
            xmin=map(vec->vec[1], vtxs) .- cw,
            ymin=map(vec->vec[2], vtxs) .- cw,
            xmax=map(vec->vec[1], vtxs) .+ cw,
            ymax=map(vec->vec[2], vtxs) .+ cw,
            Geom.rect, Theme(default_color=color) )
    else
        return layer(x=[],y=[])
    end
end

@with_kw struct SolutionSummary
    robot_paths::Vector{Vector{Int}} = Vector{Vector{Int}}()
    object_paths::Dict{Int,Vector{Int}} = Dict{Int,Vector{Int}}()
    object_intervals::Dict{Int,Tuple{Int,Int}} = Dict{Int,Tuple{Int,Int}}()
end
function SolutionSummary(env::SearchEnv)
    t0 = 0
    tf = maximum(map(length,get_paths(get_route_plan(env))))
    sched = get_schedule(env)
    history = map(t->get_env_state(env,t),t0:tf)
    robot_ids = sort(collect(keys(get_robot_ICs(sched))))
    # object_ids = sort(collect(keys(get_object_ICs(get_schedule(env)))))
    object_ids = sort(
        [get_object_id(n) for n in get_nodes(sched) if matches_template(BOT_CARRY,n)]
        )
    robot_paths = map(
        id->map(s->get_id(get_location_id(s.robot_positions[id])),history),
        robot_ids)
    object_paths = Dict(
        get_id(id)=>map(s->get_id(get_location_id(get(object_positions(s),id,OBJECT_AT(id,-1)))),
        history) for id in object_ids)
    active_objects = Dict(
        get_id(id)=>map(s->get(objects_active(s),id,false),history) for id in object_ids)
    object_intervals = Dict(
        k=>(findfirst(v)-1,findlast(v)-1) for (k,v) in active_objects)
    SolutionSummary(robot_paths,object_paths,object_intervals)
end
function get_robot_positions(env,env_summary,t)
    vec = Vector{Point3{Float64}}()
    for (i,p) in enumerate(env_summary.robot_paths)
        if length(p) > 0
            x,y = interp_position(p,get_vtxs(env),t)
            push!(vec,Point(x,y,0.0))
        end
    end
    return vec
end
function get_object_positions(env,env_summary,t)
    object_paths = env_summary.object_paths
    object_intervals = env_summary.object_intervals
    vec = Vector{Point3{Float64}}()
    for (i,k) in enumerate(sort(collect(keys(object_paths)))) # dictionary
        p = object_paths[k]
        if length(p) > 0
            x,y = interp_position(p,get_vtxs(env),t)
            push!(vec,Point(x,y,0.0))
        end
    end
    return vec
end

abstract type RenderModel end
struct RenderStack{T}
    models::T
end
@with_kw mutable struct RobotPositions <: RenderModel
    label_robots    ::Bool              = true
    colors_vec      ::Vector{Color}     = Color[]
    rsize           ::Measures.Length   = 5pt
    line_width      ::Measures.Length   = 2pt
    show_paths      ::Bool              = false
end
function RobotPositions(n::Int)
    color_scale = Scale.color_discrete_hue()
    RobotPositions(colors_vec = color_scale.f(n))
end
@with_kw mutable struct ObjectPositions <: RenderModel
    label_objects           ::Bool          = true
    show_inactive_objects   ::Bool          = true
    # show_completed_objects   ::Bool          = true
    osize                   ::Measures.Length = 5pt
    inactive_object_colors  ::Vector{Color} = Color[]
    completed_object_colors ::Vector{Color} = Color[]
    active_object_colors    ::Vector{Color} = Color[]
    object_sizes            ::Vector{Tuple{Int,Int}} = Tuple{Int,Int}[]
end
function ObjectPositions(n::Int)
    ObjectPositions(
        active_object_colors = map(i->colorant"Black",1:n),
        inactive_object_colors = map(i->colorant"Gray",1:n),
        completed_object_colors = map(i->colorant"Gray",1:n),
        object_sizes            = map(i->(1,1),1:n)
    )
end
@with_kw mutable struct Floor <: RenderModel
    cell_width      ::Float64   = 0.9
    floor_color     ::Color     = colorant"LightGray"
    pickup_color    ::Color     = colorant"LightBlue"
    dropoff_color   ::Color     = colorant"Pink"
end
function render_layer(model::RobotPositions,env::GridFactoryEnvironment,summary,t,theme=Theme())
    robot_paths = summary.robot_paths
    vtxs = get_vtxs(env)
    robot_layers = []
    path_layers = []
    for (i,p) in enumerate(robot_paths)
        if length(p) > 0
            x,y = interp_position(p,vtxs,t)
            label = model.label_robots ? string("R",i) : ""
            push!(robot_layers,
                layer(
                    x=[x],
                    y=[y],
                    label=[label],
                    Geom.label(position=:centered,hide_overlaps=false),
                    Geom.point,
                    size=[model.rsize],
                    Theme(
                        theme,
                        default_color=model.colors_vec[i],
                        # highlight_width=0pt,
                        # point_label_font=point_label_font,
                        # point_label_color=point_label_color,
                        # point_label_font_size=point_label_font_size
                        )
                    )
            )
            if model.show_paths
                xpath,ypath = interp_path(p,vtxs,t)
                push!(path_layers,
                    layer(x=xpath,y=ypath,Geom.path,
                        Theme(theme,line_width=model.line_width,default_color=model.colors_vec[i])
                    )
                )
            end
        end
    end
    return [robot_layers..., path_layers...]
end
function render_layer(model::ObjectPositions,env::GridFactoryEnvironment,summary,t,theme=Theme())
    object_paths = summary.object_paths
    object_intervals = summary.object_intervals
    object_layers = []
    for (i,k) in enumerate(sort(collect(keys(object_paths)))) # dictionary
        p = object_paths[k]
        interval = object_intervals[k]
        if interval[1] > t
            object_color = model.inactive_object_colors[i]
        elseif interval[2] < t
            object_color = model.completed_object_colors[i]
        else
            object_color = model.active_object_colors[i]
        end
        if length(p) > 0 && interval[2] > t
            if (interval[1] <= t) #|| model.show_inactive_objects
                x,y = interp_position(p,get_vtxs(env),t)
                label = model.label_objects ? string("O",k) : ""
                if model.object_sizes[i] == (1,1)
                    push!(object_layers,layer(
                            x=[x],
                            y=[y],
                            label=[label],
                            Geom.label(position=:centered,hide_overlaps=false),
                            Geom.point,
                            size=[model.osize],
                            Theme(theme,default_color=object_color,)
                            ))
                else
                    x = [x]
                    y = [y]
                    s = model.object_sizes[i]
                    if s == (2,1)
                        push!(x,x[1] + env.cell_width)
                        push!(y,y[1])
                    elseif s == (1,2)
                        push!(x,x[1])
                        push!(y,y[1] + env.cell_width)
                    elseif s == (2,2)
                        append!(x,[
                            x[1] + env.cell_width,
                            x[1] + env.cell_width,
                            x[1],
                            x[1],
                            ])
                        append!(y,[
                            y[1],
                            y[1] + env.cell_width,
                            y[1] + env.cell_width,
                            y[1],
                            ])
                    end
                    push!(object_layers,layer(x=x,y=y,
                        label=[label,map(j->"",prod(s)-1)],
                        Geom.label(position=:centered,hide_overlaps=false),
                            Geom.path,Geom.point,size=[model.osize],
                            Theme(theme,
                            default_color=object_color,
                            line_width=model.osize)
                            ))
                end
            end
        end
    end
    return object_layers
end
function render_layer(model::Floor,env::GridFactoryEnvironment,summary,t,theme=Theme())
    map(args->get_grid_world_layer(args...,model.cell_width/2.0),
        [
            (get_pickup_vtxs(env),  model.pickup_color),
            (get_dropoff_vtxs(env), model.dropoff_color),
            (get_vtxs(env),         model.floor_color),
        ]
    )
end

function render_env(rstack::RenderStack,env,summary,t;
        cell_width=1.0,
        plot_padding=[1mm],
        background_color="Gray",
        highlight_width=0pt,
        point_label_font="arial",
        point_label_color="White",
        point_label_font_size=10pt,
        theme = Theme(
            plot_padding=plot_padding,
            background_color=background_color,
            highlight_width=highlight_width,
            point_label_font=point_label_font,
            point_label_color=point_label_color,
            point_label_font_size=point_label_font_size,
        )
    )
    layers = vcat(map(m->render_layer(m,env,summary,t,theme),rstack.models)...)
    xpts = map(v->v[1],get_vtxs(env))
    ypts = map(v->v[2],get_vtxs(env))
    Gadfly.plot(
        layers...,
        Coord.cartesian(fixed=true,
            xmin=minimum(xpts) - cell_width/2,
            ymin=minimum(ypts) - cell_width/2,
            xmax=maximum(xpts) + cell_width/2,
            ymax=maximum(ypts) + cell_width/2),
        Guide.xticks(;ticks=nothing),
        Guide.yticks(;ticks=nothing),
        Guide.xlabel(nothing),
        Guide.ylabel(nothing),
        theme
    )
end

"""
    `visualize_env`

    Displays the current agent and object locations as well as the current
    planned path segment of each agent.
"""
function visualize_env(search_env::S,vtxs,pickup_vtxs,dropoff_vtxs,t=0;
        robot_vtxs=[],
        object_vtxs=[],
        robot_paths=[],
        object_paths=[],
        search_patterns=[],
        goals=[],
        vpad = 0.05,
        n=80,
        rsize=5pt,
        osize=3pt,
        cell_width=1.0,
        floor_color = "LightGray",
        pickup_color= "LightBlue",
        dropoff_color="Pink",
        line_width=2pt) where {S<:SearchEnv}

    cache = get_cache(search_env)
    sched = get_schedule(search_env)

    color_scale = Scale.color_discrete_hue()
    colors_vec = color_scale.f(n)

    cw = cell_width/2 - vpad
    xpts = map(vtx->cell_width*vtx[1], vtxs)
    ypts = map(vtx->cell_width*vtx[2], vtxs)

    t1 = Int(floor(t))+1
    path_layers = []
    for v in 1:get_num_vtxs(sched)
        vtx_id = get_vtx_id(sched, v)
        if typeof(vtx_id) <: ActionID
            node = get_node_from_id(sched,vtx_id)
            if (get_t0(sched,v) <= t) && (get_tF(sched,v) >= t)
                spec = get_path_spec(sched, v)
                agent_id = get_id(get_robot_id(node))
                agent_path = robot_paths[agent_id]
                p = agent_path[max(1,min(t1+1,get_tF(sched,v)+1)):get_tF(sched,v)+1]
                if length(p) > 0
                    idx1 = agent_path[max(1,min(t1,length(agent_path)))]
                    idx2 = agent_path[max(1,min(t1+1,length(agent_path)))]
                    x0=interpolate(vtxs[idx1][1],vtxs[idx2][1],t-(t1-1))
                    y0=interpolate(vtxs[idx1][2],vtxs[idx2][2],t-(t1-1))
                    push!(path_layers,
                        layer(
                            x=[x0, map(v->vtxs[v][1],p)...],
                            y=[y0, map(v->vtxs[v][2],p)...],Geom.path,
                            Theme(line_width=line_width,default_color=colors_vec[agent_id]))
                    )
                end
            end
            # end
        end
    end
    robot_layers = []
    for (i,p) in enumerate(robot_paths)
        if length(p) > 0
            idx1 = p[max(1,min(t1,length(p)))]
            idx2 = p[max(1,min(t1+1,length(p)))]
            push!(robot_layers,
                layer(
                    x=[interpolate(vtxs[idx1][1],vtxs[idx2][1],t-(t1-1))],
                    y=[interpolate(vtxs[idx1][2],vtxs[idx2][2],t-(t1-1))],
                    Geom.point,size=[rsize],Theme(default_color=colors_vec[i]))
            )
        end
    end
    search_pattern_layers = []
    for (i,p) in enumerate(search_patterns)
        df = DataFrame(x=map(s->vtxs[s[1]][1],p),y=map(s->vtxs[s[1]][2],p),t=map(s->s[2],p))
        push!(path_layers, layer(df,x="x",y="y",color="t",Geom.point,size=[3pt]))
    end
    goal_layers = []
    for (i,s) in enumerate(goals)
        push!(goal_layers,
            layer(x=[vtxs[s[1]][1]],y=[vtxs[s[1]][2]],Geom.point,size=[vsize],shape=[Shape.diamond],Theme(
                    default_color=colors_vec[i]))
        )
    end
    object_layers = []
    for (i,p) in enumerate(object_paths)
        if length(p) > 0
            interpolate(p[min(t1,length(p))],p[min(t1+1,length(p))],t1-(t+1))
            idx1 = p[max(1,min(t1,length(p)))]
            idx2 = p[max(1,min(t1+1,length(p)))]
            push!(object_layers,
                layer(
                    x=[interpolate(vtxs[idx1][1],vtxs[idx2][1],t-(t1-1))],
                    y=[interpolate(vtxs[idx1][2],vtxs[idx2][2],t-(t1-1))],
                    Geom.point,size=[osize],Theme(default_color="Black"))
            )
        end
    end
    # object_layer = layer( x=map(v->vtxs[v][1], object_vtxs), y=map(v->vtxs[v][2], object_vtxs),
    # size=[3pt], Geom.point, Theme(default_color="Black") )

    plot(
        object_layers...,
        # object_layer,
        robot_layers...,
        path_layers...,
        # search_pattern_layers...,
        get_grid_world_layer(pickup_vtxs,pickup_color,cw),
        get_grid_world_layer(dropoff_vtxs,dropoff_color,cw),
        get_grid_world_layer(vtxs,floor_color,cw),
        Coord.cartesian(fixed=true,
            xmin=minimum(xpts) - cell_width/2,
            ymin=minimum(ypts) - cell_width/2,
            xmax=maximum(xpts) + cell_width/2,
            ymax=maximum(ypts) + cell_width/2),
        Guide.xticks(;ticks=nothing),
        Guide.yticks(;ticks=nothing),
        Guide.xlabel(nothing),
        Guide.ylabel(nothing),
        Theme(
            plot_padding=[1mm],
            # default_color="LightGray",
            background_color="Gray"
        )
    )
end
function visualize_env(search_env::S,env::E,t=0;kwargs...) where {S<:SearchEnv,E<:GridFactoryEnvironment}
    visualize_env(search_env,get_vtxs(env),get_pickup_vtxs(env),get_dropoff_vtxs(env),t;kwargs...)
end
function visualize_env(vtxs,pickup_vtxs,dropoff_vtxs,t=0;
        robot_vtxs=[],
        object_vtxs=[],
        robot_paths=[],
        object_paths=[],
        object_intervals=map(p->[0,length(p)], object_paths),
        search_idx=1,
        show_search_paths=false,
        search_patterns=[],
        goals=[],
        vpad = 0.05,
        n=80,
        rsize=4pt,
        osize=3pt,
        search_size=1.5pt,
        goal_size=2.0pt,
        cell_width=1.0,
        line_width=2pt,
        color_scale = Scale.color_discrete_hue(),
        colors_vec = color_scale.f(n),
        floor_color = "LightGray",
        pickup_color= "LightBlue",
        dropoff_color="Pink",
        active_object_color="Black",
        project_idxs=map(i->1,object_paths),
        proj_colors_vec = gen_object_colors(maximum([1,project_idxs...])),
        active_object_colors=map(idx->proj_colors_vec[idx],project_idxs),
        label_objects=false,
        label_robots=false,
        point_label_font="arial",
        point_label_color="White",
        point_label_font_size=10pt,
        inactive_object_color="Gray",
        inactive_object_colors=map(idx->inactive_object_color,project_idxs),
        completed_object_color="Gray",
        completed_object_colors=map(idx->completed_object_color,project_idxs),
        show_inactive_objects=true,
        show_paths=true
    )

    cw = cell_width/2 - vpad
    vtxs = map(vtx->(cell_width*vtx[1],cell_width*vtx[2]),vtxs)
    xpts = map(vtx->vtx[1], vtxs)
    ypts = map(vtx->vtx[2], vtxs)

    theme = Theme(
        plot_padding=[1mm],
        background_color="Gray",
        highlight_width=0pt,
        point_label_font=point_label_font,
        point_label_color=point_label_color,
        point_label_font_size=point_label_font_size
    )

    t1 = Int(floor(t))+1
    path_layers = []
    if show_paths
        for (i,p) in enumerate(robot_paths)
            if length(p) > 0
                push!(path_layers,
                    layer(x=map(v->vtxs[v][1],p),y=map(v->vtxs[v][2],p),Geom.path,
                        Theme(theme,line_width=line_width,default_color=colors_vec[i]))
                )
            end
        end
    end
    robot_layers = []
    for (i,p) in enumerate(robot_paths)
        if length(p) > 0
            # interpolate(p[min(t1,length(p))],p[min(t1+1,length(p))],t1-(t+1))
            idx1 = p[max(1,min(t1,length(p)))]
            idx2 = p[max(1,min(t1+1,length(p)))]
            label = label_robots ? string("R",i) : ""
            push!(robot_layers,
                layer(
                    x=[interpolate(vtxs[idx1][1],vtxs[idx2][1],t-(t1-1))],
                    y=[interpolate(vtxs[idx1][2],vtxs[idx2][2],t-(t1-1))],
                    label=[label],
                    Geom.label(position=:centered,hide_overlaps=false),
                    Geom.point,
                    size=[rsize],
                    Theme(
                        theme,
                        default_color=colors_vec[i],
                        )
                    )
            )
        end
    end
    search_pattern_layers = []
    if show_search_paths
        for (i,p) in enumerate(search_patterns)
            if length(p) > 0
                idx = max(1,min(search_idx,length(p)))
                t_search = p[idx][2] # time step
                df = DataFrame(x=map(s->vtxs[s[1]][1],p[1:idx]),y=map(s->vtxs[s[1]][2],p[1:idx]),t=map(s->s[2],p[1:idx]))
                push!(search_pattern_layers,
                    layer(x=[df.x[end]],y=[df.y[end]],Geom.point,size=[search_size],
                        Theme(default_color="Black",highlight_width=0pt))
                    )
                push!(search_pattern_layers,
                    layer(df,x="x",y="y",Geom.point,size=[search_size],
                        Theme(default_color="Gray",highlight_width=0pt))
                    )
            end
        end
    end
    goal_layers = []
    for (i,s) in enumerate(goals)
        push!(goal_layers,
            layer(x=[vtxs[s[1]][1]],y=[vtxs[s[1]][2]],Geom.point,size=[goal_size],
                Theme(default_color="red",highlight_width=0pt)
                # Theme(default_color=colors_vec[i])
            )
        )
    end
    object_layers = []
    for (i,(p,interval)) in enumerate(zip(object_paths,object_intervals))
        if interval[1] > t
            object_color = inactive_object_colors[i]
        elseif interval[2] < t
            object_color = completed_object_colors[i]
        else
            object_color = active_object_colors[i]
        end
        if length(p) > 0 && interval[2] > t
            if (interval[1] <= t) || show_inactive_objects
                interpolate(p[min(t1,length(p))],p[min(t1+1,length(p))],t1-(t+1))
                idx1 = p[max(1,min(t1,length(p)))]
                idx2 = p[max(1,min(t1+1,length(p)))]
                label = label_objects ? string("O",i) : ""
                push!(object_layers,
                    layer(
                        x=[interpolate(vtxs[idx1][1],vtxs[idx2][1],t-(t1-1))],
                        y=[interpolate(vtxs[idx1][2],vtxs[idx2][2],t-(t1-1))],
                        label=[label],
                        Geom.label(position=:centered,hide_overlaps=false),
                        Geom.point,
                        size=[osize],
                        Theme(
                            theme,
                            default_color=object_color,
                            )
                        )
                )
            end
        end
    end
    if length(object_vtxs) > 0
        push!(object_layers,layer( x=map(v->vtxs[v][1], object_vtxs), y=map(v->vtxs[v][2], object_vtxs),
            size=[3pt], Geom.point, Theme(default_color="Black")))
    end
    for (i,idx) in enumerate(robot_vtxs)
        push!(robot_layers,
            layer(
                x=[vtxs[idx][1]],
                y=[vtxs[idx][2]],
                Geom.point,size=[rsize],Theme(default_color=colors_vec[i]))
        )
    end

    Gadfly.plot(
        search_pattern_layers...,
        goal_layers...,
        object_layers...,
        robot_layers...,
        path_layers...,
        get_grid_world_layer(pickup_vtxs,pickup_color,cw),
        get_grid_world_layer(dropoff_vtxs,dropoff_color,cw),
        get_grid_world_layer(vtxs,floor_color,cw),
        Coord.cartesian(fixed=true,
            xmin=minimum(xpts) - cell_width/2,
            ymin=minimum(ypts) - cell_width/2,
            xmax=maximum(xpts) + cell_width/2,
            ymax=maximum(ypts) + cell_width/2),
        Guide.xticks(;ticks=nothing),
        Guide.yticks(;ticks=nothing),
        Guide.xlabel(nothing),
        Guide.ylabel(nothing),
        theme
    )
end
function visualize_env(env::GridFactoryEnvironment,t=0;kwargs...)
    visualize_env(get_vtxs(env),get_pickup_vtxs(env),get_dropoff_vtxs(env),t;kwargs...)
end

render_env(t) = visualize_env(factory_env,t;robot_vtxs=r0,object_vtxs=s0,paths=paths,search_patterns=[],goals=[])
render_paths(t,factory_env,robot_paths,object_paths=[];kwargs...) = visualize_env(factory_env,t;robot_paths=robot_paths,object_paths=object_paths,kwargs...)
render_both(t,paths1,paths2) = hstack(render_paths(t,paths1),render_paths(t,paths2))
render_search(t_start,factory_env;kwargs...) = visualize_env(factory_env,t_start;show_search_paths=true,kwargs...)


"""
    ResultsTable

A simple Table data structure for compiling tabular results.
"""
struct ResultsTable
    xkeys::Vector{Dict{Any,Any}}
    ykeys::Vector{Dict{Any,Any}}
    data::Matrix{Any}
end
get_data(tab::ResultsTable) = tab.data
get_data(tab) = tab
get_xkeys(tab::ResultsTable) = tab.xkeys
get_ykeys(tab::ResultsTable) = tab.ykeys
Base.size(tab::ResultsTable) = size(get_data(tab))
Base.size(tab::ResultsTable,dim) = size(get_data(tab),dim)
Base.transpose(tab::ResultsTable) = ResultsTable(get_ykeys(tab),get_xkeys(tab),
    [tab.data[i,j] for j in 1:size(tab,2), i in 1:size(tab,1)],
    # transpose(get_data(tab)),
    )

_dict_vec(d::Dict) = Vector{Dict{Any,Any}}([d])
_dict_vec(d::Vector) = d
Base.getindex(tab::ResultsTable,idxs1,idxs2) = ResultsTable(
    _dict_vec(tab.xkeys[idxs1]),
    _dict_vec(tab.ykeys[idxs2]),
    Matrix{Any}(reshape([tab.data[idxs1,idxs2]...],(length(idxs1),length(idxs2)))),
    )
function index_by_keys(tab::ResultsTable,include_keys::Dict=Dict(),exclude_keys::Dict=Dict())
    x_idxs = Int[]
    y_idxs = Int[]
    for (idxs,keylist) in [(x_idxs,get_xkeys(tab)),(y_idxs,get_ykeys(tab))]
        for (i,dict) in enumerate(keylist)
            keep = true
            for (k,v) in dict
                if haskey(include_keys,k) && !(v in include_keys[k])
                    keep = false
                    break
                end
                if haskey(exclude_keys,k) && (v in exclude_keys[k])
                    keep = false
                    break
                end
            end
            if keep
                push!(idxs,i)
            end
        end
    end
    tab[x_idxs,y_idxs]
end
function init_table(xkeys,ykeys)
    tab = ResultsTable(xkeys, ykeys, zeros(length(xkeys),length(ykeys)))
end
function Base.vcat(tab1::ResultsTable,tabs...)
    ResultsTable(
        vcat(get_xkeys(tab1),map(get_xkeys, tabs)...),
        get_ykeys(tab1),
        vcat(get_data(tab1),map(get_data,tabs)...)
    )
end
function Base.hcat(tab1::ResultsTable,tab2)
    ResultsTable(
        get_xkeys(tab2),
        vcat(get_ykeys(tab1),get_ykeys(tab2)),
        hcat(tab1.data,tab1.data)
    )
end

"""
    build_table

Takes in a dataframe, builds a table of `obj` values along axes `xkey` and
`ykey`, where the `obj` values are aggregated via the function `f`.
"""
function build_table(df;
    obj=:tasks_per_second,
    xkey=:M,
    ykey = :arrival_interval,
    include_keys=[],
    exclude_keys=[],
    xsort_reverse=false,
    ysort_reverse=false,
    xvals = sort(unique(df[!,xkey]);rev=xsort_reverse),
    yvals = sort(unique(df[!,ykey]);rev=ysort_reverse),
    aggregator = vals -> sum(vals)/length(vals),
    )

    base_idxs = trues(nrow(df))
    if !isempty(include_keys)
        base_idxs = .&(base_idxs, [(df[!,k] .== v) for (k,v) in include_keys]...)
    end
    if !isempty(exclude_keys)
        base_idxs = .&(base_idxs, [(df[!,k] .!= v) for (k,v) in exclude_keys]...)
    end
    if !isempty(include_keys) || !isempty(exclude_keys)
        xvals = sort(unique(df[base_idxs,xkey]);rev=xsort_reverse)
        yvals = sort(unique(df[base_idxs,ykey]);rev=ysort_reverse)
    end

    xkeys = map(x->Dict(xkey=>x), xvals)
    ykeys = map(y->Dict(ykey=>y), yvals)
    tab = zeros(length(xvals),length(yvals))
    tab = ResultsTable(xkeys, ykeys, tab)
    for (i,x) in enumerate(xvals)
        for (j,y) in enumerate(yvals)
            idxs = .&(base_idxs,(df[!,xkey] .== x),(df[!,ykey] .== y))
            # if any(idxs)
                vals = df[idxs,obj]
                tab.data[i,j] = aggregator(vals)
            # else
            #     tab.data[i,j] = nothing
            # end
        end
    end
    # xkeys = map(x->Dict(xkey=>x), xvals)
    # ykeys = map(y->Dict(ykey=>y), yvals)
    # return ResultsTable(xkeys, ykeys, tab)
    return tab
end
function table_product(tables;
        mode=:horizontal,
        # _xheader_keys=[Dict() for i in 1:length(tables)],
        # _yheader_keys=[Dict() for j in 1:length(tables)],
    )
    @assert length(unique(size.(tables))) == 1 "All tables must have same dimensions"
    if mode == :horizontal
        data = collect(zip(map(get_data, tables)...))
        xkeys = map(tup->merge(tup...), collect(zip(map(get_xkeys, tables)...)))
        # xkeys = [merge(a,b) for (a,b) in zip(_xkeys,_xheader_keys)]
        ykeys = map(tup->merge(tup...), collect(zip(map(get_ykeys, tables)...)))
        # ykeys = [merge(a,b) for (a,b) in zip(_ykeys,_yheader_keys)]
        return ResultsTable(xkeys,ykeys,data)
    else
        return transpose(table_product(map(transpose,tables),:horizontal))
    end
end
function table_reduce(table,op)
    return ResultsTable(get_xkeys(table),get_ykeys(table),map(op,get_data(table)))
end
function flatten_table(tab;
        axis = :y,
        _header_keys = Dict(),
    )
    if axis == :y
        data = reshape(vcat([tab.data[:,j] for j in 1:size(tab,2)]...),prod(size(tab)),1)
        xkeys = [[merge(xk,yk) for (i,xk) in enumerate(get_xkeys(tab)), (j,yk) in enumerate(get_ykeys(tab))]...]
        ykeys = [_header_keys]
        return ResultsTable(xkeys,ykeys,data)
    else
        return transpose(flatten_table(transpose(tab);axis=:y,_header_keys=_header_keys))
    end
end


"""
    fill_tab_from_columns!(tab,df;

tab has row indices for values of df.xkey and column indices for column names
    df.
"""
function fill_tab_from_columns!(tab,df;
        xkey = :model,
        ykey = :metric,
        include_keys = [],
        exclude_keys = [],
        f = vals -> get(filter(v->isa(v,Real),vals),1,nothing),
    )
    idxs = get_idxs(df;include_keys=include_keys,exclude_keys=exclude_keys)
    for (i,xdict) in enumerate(get_xkeys(tab))
        xval = xdict[xkey]
        for (j,ydict) in enumerate(get_ykeys(tab))
            yval = ydict[ykey]
            vals = df[!,yval][idxs .& (df[!,xkey] .== xval)]
            if !isempty(vals)
                tab.data[i,j] = f(vals)
            end
        end
    end
    tab
end

"""
    table_inner_product(tables,xkeys,ykeys)

Given an `m × n` matrix of tables, construct a new `m × n` table `new_tab` such
that `new_tab[i,j]` is a `m × n` matrix whose `a,b`th element is the `i,j`th
element of table `tables[a,b]`.
"""
function table_inner_product(tables,xkeys,ykeys)
    # @show @assert length(unique(size.(tables))) == 1 "All tables must have same dimensions"
    tab1 = tables[1]
    dims = size(tab1)
    tab = init_table(deepcopy(get_xkeys(tab1)),deepcopy(get_ykeys(tab1)))
    for (i,x) in enumerate(get_xkeys(tab))
        for (j,y) in enumerate(get_ykeys(tab))
            tab.data[i,j] = ResultsTable(deepcopy(xkeys),deepcopy(ykeys),map(t->t.data[i,j],tables))
        end
    end
    tab
end

""" utility for printing real-valued elements of a table """
function print_real(io,v;
        precision=3,
        kwargs...,
        )
    if isa(v,Int)
        print(io,v)
    elseif !isa(v,Real)
        print(io,v)
    elseif isnan(v)
        print(io,"")
    else
        if precision == 0
            print(io,Int(round(v;digits=precision)))
        else
            print(io,round(v;digits=precision))
        end
    end
end
""" """
function print_multi_value_real(io,vals;
        delim=" & ",
        surround=["",""],
        stopchar="",
        comp_func=vals->-1,
        print_func=print_real,
        kwargs...)
    best_idx = comp_func(vals)
    if 1 <= best_idx <= length(vals)
        best_idxs = findall(vals .== vals[best_idx])
    else
        best_idxs = []
    end
    for (i,v) in enumerate(vals)
        print(io,surround[1])
        i in best_idxs ? print(io,"\\textbf{") : nothing
        print_func(io,v;kwargs...)
        i in best_idxs ? print(io,"}") : nothing
        print(io,surround[2])
        if i < length(vals)
            print(io,delim)
        else
            print(io,stopchar)
        end
    end
end
print_multi_value_real(io,v::Real;kwargs...) = print_real(io,v;kwargs...)
span_length(::Real) = 1
span_length(v::Union{Vector,Tuple}) = length(v)
span_length(::ResultsTable) = 1
span_length(::String) = 1
function print_latex_header(io,tab;
        span=span_length(get_data(tab)[1,1]),
        delim=" & ",
        newline=" \\\\\n",
        start_char="& ",
        group_delim=" | ",
        colspec="l ",
        alignspec="c",
        # initial_colspec=colspec,
        # terminal_spec="",
        terminal_spec="@{}",
        initial_colspec="@{}l",
        print_row_start=true,
        kwargs...
        )

    print(io,"\\begin{tabular}[$(alignspec)]{")
    if print_row_start
        print(io,initial_colspec,)
    end
    for i in 1:size(tab,2)
        if (i > 1) || print_row_start
            print(io, group_delim)
        end
        for j in 1:span
            print(io,colspec)
        end
    end
    print(io,terminal_spec,"}","\n")
end
function print_latex_column_labels(io,tab;
        span=span_length(get_data(tab)[1,1]),
        delim=" & ",
        newline=" \\\\\n",
        start_char="& ",
        group_delim=" | ",
        print_row_start=true,
        col_label_func=(k,v)->"\$$(k)=$(v)\$",
        col_label_wrap=("\\multicolumn{$span}{c}{","}"),
        kwargs...
    )
    # labels
    if print_row_start
        print(io,start_char)
    end
    for (j,ykeys) in enumerate(get_ykeys(tab))
        print(io,col_label_wrap[1],
            [col_label_func(k,v) for (k,v) in ykeys]...,
            col_label_wrap[2])
        if j < size(tab,2)
            print(io,delim)
        else
            print(io,newline)
        end
    end
end
function print_latex_row_start(io,tab,i;
        delim=" & ",
        row_label_func=(k,v)->"\$$(k)=$(v)\$",
        row_label_delim=",",
        kwargs...
        )
    xkeys = get_xkeys(tab)[i]
    for (idx,(k,v)) in enumerate(xkeys)
        print(io,row_label_func(k,v))
        if !(idx == length(xkeys))
            print(io,row_label_delim)
        end
    end
    print(io,delim)
end
function write_tex_table(io,tab;
        delim=" & ",
        newline=" \\\\\n",
        final_newline=newline,
        # print_func = (io,v) -> print_real(io,v),
        print_func = print_multi_value_real,
        header_printer = print_latex_header,
        row_start_printer = print_latex_row_start,
        print_content=true,
        print_header=print_content,
        print_column_labels=print_content,
        print_row_start=print_content,
        print_footer=print_content,
        pre_footer="",
        post_header="",
        row_dividers=Dict(),
        kwargs...,
    )
    if print_header
        header_printer(io,tab;
            print_row_start=print_row_start,
            kwargs...)
        print(io,post_header)
    end
    if print_column_labels
        print_latex_column_labels(io,tab;
            print_row_start=print_row_start,
            kwargs...)
    end
    if haskey(row_dividers,0)
        print(io, row_dividers[0],"\n")
    end
    for (i,xkeys) in enumerate(get_xkeys(tab))
        if print_row_start
            row_start_printer(io,tab,i;delim=delim,kwargs...)
        end
        if print_content
            for (j,ykeys) in enumerate(get_ykeys(tab))
                print_func(io,get_data(tab)[i,j];kwargs...)
                if j < size(tab,2)
                    print(io,delim)
                else
                    if i < size(tab,1)
                        print(io,newline)
                    else
                        print(io, final_newline)
                    end
                end
            end
        end
        if haskey(row_dividers,i)
            print(io, row_dividers[i],"\n")
        end
    end
    if print_footer
        print(io,pre_footer)
        print(io,"\\end{tabular}")
    end
end
function write_tex_table(fname::String,args...;kwargs...)
    open(fname,"w") do io
        write_tex_table(io,args...;kwargs...)
    end
end
function write_row_labels(io,tab;
        kwargs...,
    )
    write_tex_table(io,tab;
        print_content=false,
        print_row_start=true,
        delim = " \\\\\n",
        kwargs...,
    )
end

"""
    nested_label_func(nested_xkeys::Vector)

Return a function that outputs a nested table row or column (depends on `mode`)
label.
"""
function nested_label_func(nested_xkeys::Vector;
        mode=:row,
        kwargs...,
        )
    label_func = (k,v) -> begin
        io = IOBuffer()
        xlabels = ResultsTable(
            [Dict(:x=>x) for x in nested_xkeys],
            [Dict()],
            reshape([x for x in nested_xkeys],(:,1)),
            )
        if mode != :row
            xlabels = ResultsTable(
                xlabels.ykeys,
                xlabels.xkeys,
                reshape(xlabels.data,(1,:))
            )
        end
        write_tex_table(io,xlabels;
            print_column_labels=false,
            print_row_start=false,
            print_func=print_real,
            kwargs...
        )
        row_prefix = String(take!(io))
        @show row_prefix
        tab = ResultsTable([Dict(k=>v)],[Dict(k=>v)],reshape([row_prefix],(1,1)))
        @show tab
        write_tex_table(io,tab;
            print_column_labels=(mode == :col),
            print_row_start=(mode == :row),
            print_func=print_real,
            kwargs...
        )
        return String(take!(io))
    end
    return label_func
end


print_nested_tex_table(io,tab::ResultsTable;kwargs...,) = write_nested_tex_table(io,tab;kwargs...)
print_nested_tex_table(io,data;print_func = print_multi_value_real,kwargs...,) = print_func(io,data;kwargs...)
print_nested_tex_table(io,data::String;kwargs...,) = print(io,data)
function write_nested_tex_table(f::String,tab;kwargs...)
    open(f,"w") do io
        write_nested_tex_table(io,tab;kwargs...)
    end
end
function write_nested_tex_table(io,tab;
        delim=" & ",
        newline=" \\\\\n",
        print_func = print_nested_tex_table,
        header_printer = print_latex_header,
        row_start_printer = print_latex_row_start,
        print_header=true,
        print_column_labels=true,
        print_row_start=true,
        print_footer=true,
        row_dividers=Dict(),
        kwargs...,
    )
    if print_header
        # header_printer(io,tab)
        header_printer(io,tab;
            print_row_start=print_row_start,
            kwargs...)
    end
    if print_column_labels
        print_latex_column_labels(io,tab;
            print_row_start=true,
            kwargs...)
    end
    if haskey(row_dividers,0)
        print(io, row_dividers[0],"\n")
    end
    for (i,xkeys) in enumerate(get_xkeys(tab))
        if print_row_start
            row_start_printer(io,tab,i;kwargs...)
        else
            print(io,delim)
        end
        for (j,ykeys) in enumerate(get_ykeys(tab))
            print_func(io,get_data(tab)[i,j];
                print_column_labels = (i == 1),
                print_row_start = (j == 1),
                kwargs...)
            if j < size(tab,2)
                print(io,delim)
            else
                if i < size(tab,1)
                    print(io,newline)
                else
                    print(io, "\n")
                end
            end
        end
        if haskey(row_dividers,i)
            print(io, row_dividers[i],"\n")
        end
    end
    if print_footer
        print(io,"\\end{tabular}")
    end
end


# Plotting
function preprocess_results!(df)
    if nrow(df) > 0
        if :depth_bias in names(df)
            begin df[!,:depth_bias_string] = string.(df.depth_bias)
                df
            end
        end
        if :N in names(df)
            begin df[!,:N_string] = string.(df.N)
                df
            end
        end
        sort!(df, (:M,:N))
    end
    df
end
function preprocess_results!(df_dict::Dict)
    for (k,df) in df_dict
        preprocess_results!(df)
    end
    df_dict
end

function robots_vs_task_vs_time_box_plot(df;
        title="Solution time by Number of Robots (N) and Number of Tasks (M)",
        yticks=[-1,0,1,2],
        ymin=-1.5,
        ymax=2.3,
        y_bounds=[0.01,100.0],
        big_font=14pt,
        small_font=12pt,
        y=:time,
        x=:N_string,
        xgroup=:M,
        color=:N_string,
        ylabel="computation time (s)",
        scale_y = Scale.y_log10(minvalue=y_bounds[1],maxvalue=y_bounds[2])
    )
    latex_fonts = Theme(major_label_font="CMU Serif", major_label_font_size=big_font,
                    minor_label_font="CMU Serif", minor_label_font_size=small_font,
                    key_title_font="CMU Serif", key_title_font_size=small_font,
                    key_label_font="CMU Serif", key_label_font_size=small_font)

    plot(df, xgroup=xgroup, x=x, y=y, color=color,
        Geom.subplot_grid(
            Geom.boxplot(;suppress_outliers=false),
            Coord.cartesian(; ymin=ymin, ymax=ymax),
            Guide.yticks(;ticks=yticks),
            Guide.xticks(;label=false),
            ),
        Guide.title(title),
        Guide.colorkey(title="number of robots", labels=["10","20","30","40"], pos=[0.1w,-0.32h]),
        Scale.group_discrete(labels=M->string(M," Tasks"),levels=[10,20,30,40,60]),
        Guide.xlabel("number of tasks"),
        Guide.ylabel(ylabel),
        scale_y,
        latex_fonts
    )
end

function get_box_plot_group_plot(df;
        obj=:time,
        outer_key=:M,
        inner_key=:N,
        outer_range=sort(unique(df[!,outer_key])),
        inner_range=sort(unique(df[!,inner_key])),
        xmin=0,
        xmax=length(inner_range)+1,
        ymin=minimum(df[!,obj]),
        ymax=maximum(df[!,obj]),
        xtick=[],
        xticklabels=xtick,
        tickpos="left",
        ytick=[0.1,1,10,100],
        ylabel_shift="0pt",
        title="",
        title_shift=[3.3,2.55],
        inner_sym="n",
        outer_sym="m",
        xlabels=map(m->string(outer_sym," = ",m), outer_range),
        ylabels=map(n->string(inner_sym," = ",n), inner_range),
        ylabel="$(string(obj)) (s)",
        draw_labels=true,
        ymode="log",
        width="3.25cm",
        height="6cm",
        legend_draw="black",
        legend_fill="white",
        legend_x_shift="2pt",
        mark="*",
        axis_bg_color="white",
        plot_type="boxplot",
        legend_pos="north west",
        legend_idx=1,
    )
    @pgf gp = PGFPlotsX.GroupPlot({group_style = {
                "group name"="myPlots",
                "group size"="$(length(outer_range)) by 1",
                "xlabels at"="edge bottom",
                "xticklabels at"="edge bottom",
                "vertical sep"="0pt",
                "horizontal sep"="2pt"
            },
            ymode=ymode,
            # footnotesize,
            width=width,
            height=height,
            xmin=xmin,
            xmax=xmax,
            ymin=ymin,
            ymax=ymax,
            xtick=xtick,
            xticklabels=xtick,
            tickpos=tickpos,
            ytick=ytick,
            yticklabels=[],
            "axis background/.style"="{fill=$axis_bg_color}",
            "ylabel shift"=ylabel_shift,
            "ytick align"="outside",
            "xtick align"="outside",
        }

        );

        if plot_type == "boxplot"
            gp.options[plot_type] = nothing
            gp.options["boxplot/draw_direction"] = "y"
        end

    @pgf for (i,m) in enumerate(outer_range)
        legend_options = {"legend style"={
            draw=legend_draw,
            fill=legend_fill,
            xshift=legend_x_shift,
            yshift="0pt"},
            "legend pos"=legend_pos,
            }
        if i == 1 && draw_labels
            label_options = {xlabel=@sprintf("\$%s\$",xlabels[i]),
                ylabel=ylabel,
                yticklabels=ytick,}
        else
            label_options = {
                xlabel=@sprintf("\$%s\$",xlabels[i]),
                ymajorticks="false",
                yminorticks="false"}
        end
        if i == legend_idx
            push!(gp,merge(label_options,legend_options),
                map(j->PGFPlotsX.LegendEntry({},@sprintf("\$%s\$",ylabels[j]),false),1:length(inner_range))...,
                """
                \\addlegendimage{only marks, blue}
                \\addlegendimage{only marks, red}
                \\addlegendimage{only marks, brown}
                \\addlegendimage{only marks, black}
                """,
            )
        else
            push!(gp,label_options)
        end
        for (i,n) in enumerate(inner_range)
            y = df[(df[:,outer_key] .== m) .& (df[:,inner_key] .== n),obj]
            if plot_type == "boxplot"
                p = PGFPlotsX.PlotInc({boxplot,mark=mark},PGFPlotsX.Table(
                    {"y index"=0},
                    [:data=>y]
                    )
                )
            else
                p = PGFPlotsX.PlotInc({"only marks",mark=mark},PGFPlotsX.Table(
                    [:x=>[i for _ in y],:y=>y]
                    )
                )
            end
            push!(gp,p)
        end
    end;
    gp
end
function get_titled_group_box_plot(df;
        title="",
        title_shift=[3.3,2.55],
        scale=1.0,
        axis_bg_color="white",
        kwargs...)
    gp = get_box_plot_group_plot(df;kwargs...)
    if title != ""
        tikzpic = @pgf TikzPicture({scale=scale,
                "background rectangle/.style"="{fill=$(axis_bg_color)}",
                # "use background rectangle/.style"="{show background rectangle}",
                },
            """
            \\centering
            \\pgfplotsset{set layers=standard,cell picture=true}
            """,
            gp,
            @sprintf("""
            \\node (title) at (\$(myPlots c1r1.center)!0.5!(myPlots c2r1.center)+(%2.2fcm,%2.2fcm)\$) {\\textbf{%s}};
            """,title_shift...,title),
            # """
            # \\node[anchor= north west] (leg) at (myPlots c1r1.north west){\\pgfplotslegendfromname{grouplegend}};
            # """
        )
    else
        tikzpic = @pgf TikzPicture({scale=scale},
            """
            \\centering
            \\pgfplotsset{set layers=standard,cell picture=true}
            """,
            gp,
            # """
            # \\node[anchor= north west] (leg) at (myPlots c1r1.north west){\\pgfplotslegendfromname{grouplegend}};
            # """
        )
    end
    return tikzpic
end

function preprocess_collab_results!(df_dict)
    num_tasks = [12,18,24]
    num_robots = [24]
    depth_biases=[0.1]
    task_size_distributions = [1,2,3,4]
    num_trials=16
    task_ratios = Int[]
    for (M,N,task_sizes,trial) in Base.Iterators.product(
            num_tasks,num_robots,task_size_distributions,1:num_trials
            )
        push!(task_ratios, task_sizes)
    end

    for (k,df) in df_dict
        if nrow(df) > 0
            begin df[!,:depth_bias_string] = string.(df.depth_bias)
                df
            end
            begin df[!,:N_string] = string.(df.N)
                df
            end
            begin df[!,:M_string] = string.(df.M)
                df
            end
            begin df[!,:TaskRatio] = task_ratios[df.problem_id]
                df
            end
        end
    end
    df_dict
end
function plot_collab_runtimes(df;
        title="Solution time by Number of Tasks (M) and ratio of tasks (task ratio )",
        yticks=[-1,0,1,2],
        ymin=-1.5,
        ymax=2.3,
        y_bounds=[0.01,100.0],
        big_font=14pt,
        small_font=12pt,
        xgroup=:TaskRatio,
        x=:M_string,
        y=:time,
        suppress_outliers=true,
        scale_y = Scale.y_log10(minvalue=y_bounds[1],maxvalue=y_bounds[2])
    )
    latex_fonts = Theme(major_label_font="CMU Serif", major_label_font_size=big_font,
                    minor_label_font="CMU Serif", minor_label_font_size=small_font,
                    key_title_font="CMU Serif", key_title_font_size=small_font,
                    key_label_font="CMU Serif", key_label_font_size=small_font)

    plot(df, xgroup=xgroup, x=x, y=y, color=x,
        Geom.subplot_grid(
            Geom.boxplot(;suppress_outliers=suppress_outliers),
            Coord.cartesian(; ymin=ymin, ymax=ymax),
            Guide.yticks(;ticks=yticks),
            Guide.xticks(;label=false),
            ),
        Guide.title(title),
        Guide.colorkey(title="num tasks", labels=["12","18","24"], pos=[0.1w,-0.32h]),
        # Scale.group_discrete(labels=M->string(M," Tasks"),levels=[1,2,3,4]),
        Guide.xlabel("task ratio"),
        Guide.ylabel("computation time (s)"),
        scale_y,
        latex_fonts
    )
end
function plot_collab_counts(df;
        title="# failures vs. # Tasks (M) x Task Ratio",
        yticks=[0,4,8,12,16],
        ymin=0,
        ymax=16,
        y_bounds=[0.01,100.0],
        big_font=14pt,
        small_font=12pt,
        task_ratios = [
            ( 1=>1.0, 2=>0.0, 4=>0.0 ),
            ( 1=>1.0, 2=>1.0, 4=>0.0 ),
            ( 1=>1.0, 2=>1.0, 4=>1.0 ),
            ( 1=>0.0, 2=>1.0, 4=>1.0 ),
        ],
        key=df.optimal
    )


    opt_df = DataFrame( M = Int[], task_ratio = Int[], num_no = Int[], num_yes = Int[] )

    for (i,ratio) in enumerate(task_ratios)
        for m in Set(collect(df.M))
        push!(
            opt_df,
            Dict(
                :M =>m,
                :task_ratio => i,
                :num_no => nrow(df[(df.M .== m) .* (df.TaskRatio .== i) .* (key .== false),:]),
                :num_yes => nrow(df[(df.M .== m) .* (df.TaskRatio .== i) .* (key .== true),:]),
            )
        )
        end
    end
    begin opt_df[!,:M_string] = string.(opt_df.M)
        opt_df
    end

    latex_fonts = Theme(major_label_font="CMU Serif", major_label_font_size=big_font,
                    minor_label_font="CMU Serif", minor_label_font_size=small_font,
                    key_title_font="CMU Serif", key_title_font_size=small_font,
                    key_label_font="CMU Serif", key_label_font_size=small_font)

    plot(opt_df, xgroup=:task_ratio, x=:M_string, y=:num_no, color=:M_string,
        Geom.subplot_grid(
            Geom.bar,
            Coord.cartesian(; ymin=ymin, ymax=ymax),
            Guide.yticks(;ticks=yticks),
            Guide.xticks(;label=false),
            ),
        Guide.title(title),
        Guide.colorkey(title="num tasks", labels=["12","18","24"], pos=[0.1w,-0.32h]),
        Scale.group_discrete(labels=M->string(M," Tasks"),levels=[1,2,3,4]),
        Guide.xlabel("task ratio"),
        Guide.ylabel("computation time (s)"),
        latex_fonts
    )
end

function plot_histories_pgf(df_dict::Dict,
    key_list=sort(string.(collect(keys(df_dict)))),args...;
    legend_entries = key_list,
    kwargs...)
    plot_histories_pgf([df_dict[k] for k in key_list],args...;
        legend_entries = legend_entries,
        kwargs...
    )
end
function plot_histories_pgf(df_list::Vector,ax=PGFPlots.Axis();
        y_key=:makespans,
        x_key=:arrival_times,
        m_key=:M,
        m_vals=sort(intersect([unique(df[!,m_key]) for df in df_list]...)),
        n_key=:arrival_interval,
        n_vals=sort(intersect([unique(df[!,n_key]) for df in df_list]...)),
        include_keys=[],
        exclude_keys=[],
        opt_key=:backup_flags,
        use_y_lims=false,
        y_min=minimum(map(df->minimum(map(minimum,df[!,y_key])),df_list)),
        y_max=maximum(map(df->maximum(map(maximum,df[!,y_key])),df_list)),
        xlabel=string("\$",string(m_key)," = ",m_vals...,"\$"),
        ylabel=string("\$",string(n_key)," = ",n_vals...,"\$"),
        colors=["red","blue","black","brown"],
        legend_entries = ["" for c in colors],
        legend_flag = true,
        ytick_show=false,
        ymode="log",
        legend_pos = "outer north east",
        legend_font = "\\footnotesize",
        lines=true,
        opt_marks=map(d->"x",df_list),
        non_opt_marks=opt_marks,
    )

    ax.ymode    =   ymode
    if length(xlabel) > 0
        ax.xlabel = xlabel
    end
    if ytick_show
        ax.ylabel = ylabel
    else
        ax.ylabel = nothing
        if ax.style === nothing
            ax.style="yticklabels=none"
        elseif findfirst(ax.style,"yticklabels") === nothing
            ax.style=string(ax.style,", ","yticklabels=none")
        end
    end
    if use_y_lims
        ax.ymin = y_min
        ax.ymax = y_max
    end
    if length(legend_pos) > 0
        ax.legendPos = legend_pos
    end
    ax.legendStyle = "font=$(legend_font)"
    counters = zeros(length(df_list))
    for (i,m_val) in enumerate(m_vals)
        for (j,n_val) in enumerate(n_vals)
            for (df_idx,(df,color,l_entry,opt_mark,non_opt_mark)) in enumerate(
                    zip(df_list,colors,legend_entries,opt_marks,non_opt_marks))
                idxs = .&(
                    (df[!,m_key] .== m_val),
                    (df[!,n_key] .== n_val),
                    )
                if !any(idxs)
                    continue
                end
                if !isempty(include_keys)
                    idxs = .&(idxs, [(df[!,k] .== v) for (k,v) in include_keys]...)
                end
                if !isempty(exclude_keys)
                    idxs = .&(idxs, [(df[!,k] .!= v) for (k,v) in exclude_keys]...)
                end
                df_cut = df[idxs,:]
                style=string(color,",solid, mark options={solid,fill=",color,"}")
                for k in 1:nrow(df_cut)
                    y_arr = df_cut[!,y_key][k]
                    x_arr = x_key == :none ? collect(1:length(y_arr)) : df_cut[!,x_key][k]
                    opt_vals = df_cut[!,opt_key][k] # tracks whether the fall back was employed

                    idx = minimum([length(x_arr),length(y_arr),length(opt_vals)])
                    # idx = minimum([length(x_arr),length(y_arr)])
                    # Overall trend
                    if lines
                        push!(ax,
                            Plots.Linear(x_arr[1:idx], y_arr[1:idx], style=style, mark="none")
                        )
                    end
                    # Timeouts
                    if 0 < idx < length(y_arr)
                        # End points
                        push!(ax,
                            Plots.Linear(
                                [x_arr[idx]],
                                [y_arr[idx]],
                                mark="o",
                                style=style,
                                onlyMarks=true, )
                        )
                        idx -= 1 # decrement so that the final point is not covered twice
                    end

                    # tmidxs=findall(.~opt_vals[1:idx])
                    # tmidxs=1:idx
                    for (tmidxs, mark) in [
                        (findall(.!opt_vals[1:idx]),opt_mark),
                        (findall(opt_vals[1:idx]),non_opt_mark),
                        ]
                        if length(tmidxs) > 0
                            p = Plots.Linear(
                                        x_arr[tmidxs],
                                        y_arr[tmidxs],
                                        style=style,
                                        onlyMarks=true,
                                        mark=mark,
                                        )
                            if legend_flag && counters[df_idx] == 0 && length(l_entry) > 0
                                p.legendentry = l_entry
                                counters[df_idx] += 1
                                insert!(ax.plots,1,p)
                            else
                                push!(ax,p)
                            end
                        end
                    end
                end
            end
        end
    end
    return ax
end
function plot_table(tab;
        xkey=:M,
        ykey=:alg,
        # xticks = join(string.(1:size(tab,1)),","),
        # xticklabels = join(string.([d[xkey] for d in get_xkeys(tab)]),","),
        # style="xtick={$(xticks)},xticklabels={$(xticklabels)},xlabel=\$m\$",
        title="",
        plot_3d=true,
        x_val=0,
        legend_func=i->get_ykeys(tab)[i][ykey],
        z_func=i->Float64.(get_data(tab)[:,i]),
        x_func=i->collect(1:size(tab,1)),
        y_func=i->[x_val for i in 1:size(tab,1)],
        kwargs...,
    )
    plts = Vector{PGFPlots.Plots.Plot}()
    for i in 1:size(tab,2)
        z = z_func(i)
        y = y_func(i)
        x = x_func(i)
        if plot_3d
            push!(plts,PGFPlots.Plots.Linear3(x,y,z,legendentry=legend_func(i)))
        else
            push!(plts,PGFPlots.Plots.Linear(z,legendentry=legend_func(i)))
        end
    end
    ax = PGFPlots.Axis(
        plts,
        # style=style,
        title=title,
        kwargs...
    )
end

# function mesh_table_points(tab,mask)
#     pts = []
#     explored = Set()
#     frontier = Set([findfirst(mask).I])
#     while !isempty(frontier)
#         (i0,j0) = pop!(frontier)
#         for d in [(-1,0),(1,0),(0,-1),(0,1)]
#             (i,j) = (i0,j0) .+ d
#             if get(mask,(i,j),false)
#                 push!(pts,(i,j))
#                 push!(pts,(i0,j0))
#             end
#         end
#     end
#     for i in 1:size(table,1)
#         for j in 1:size(table,2)
#             if mask

#             end
#         end
#     end
# end

function plot_history_layers(df,keylist,ax=PGFPlots.Axis();
        legend_entries = map(string,keylist),
        colors = ["red","blue","black","brown"],
        opt_marks=["diamond","x","triangle","+"],
        kwargs...
    )
    plt = ax
    for (i,k) in enumerate(keylist)
        plt = plot_histories_pgf([df],plt;
            y_key = k,
            legend_entries=[legend_entries[i]],
            colors = [colors[i]],
            opt_marks = [opt_marks[i]],
            kwargs...,
            )
    end
    return plt
end

function group_history_plot(df_dict::Dict,
    key_list=sort(string.(collect(keys(df_dict))));
    kwargs...)
    group_history_plot([df_dict[k] for k in key_list];
        kwargs...
    )
end
function group_history_plot(df_list::Vector;
    m_key=:M,
    m_vals=sort(intersect([unique(df[m_key]) for df in df_list]...)),
    n_key=:arrival_interval,
    n_vals=sort(intersect([unique(df[n_key]) for df in df_list]...)),
    y_key=:makespans,
    y_min=minimum(map(df->minimum(map(minimum,df[!,y_key])),df_list)),
    y_max=maximum(map(df->maximum(map(maximum,df[!,y_key])),df_list)),
    legend_location = (1,1),
    kwargs...
    )
    g = PGFPlots.GroupPlot(length(m_vals),length(n_vals))
    for (i,m_val) in enumerate(m_vals)
        ytick_show = (i == 1)
        for (j,n_val) in enumerate(n_vals)
            legend_flag = (legend_location[1] == i) && (legend_location[2] == j)
            plt = plot_histories_pgf(df_list;
                y_key=y_key,
                m_key=m_key,
                m_vals=[m_val,],
                n_key=n_key,
                n_vals=[n_val,],
                ytick_show = ytick_show,
                legend_flag = legend_flag,
                y_min=y_min,
                y_max=y_max,
                kwargs...)
            push!(g,plt)
        end
    end
    return g
end

function get_idxs(df;
        include_keys=[],
        exclude_keys=[],
    )
    idxs = trues(nrow(df))
    if !isempty(include_keys)
        idxs .&= .&([df[!,key] .== val for (key,val) in include_keys]...)
    end
    if !isempty(exclude_keys)
        idxs .&= .&([df[!,key] .!= val for (key,val) in exclude_keys]...)
    end
    idxs
end

"""
    quantile_traces(df;
        obj=:primary_runtimes,
        include_keys=Dict(),
        color="red",
        quants=[0.25],
        opacity=0.6,
        fade=0.6,
    )

Fill in the space between the alpha quantiles of some histories in `df`.
"""
function quantile_traces(df;
        obj=:primary_runtimes,
        include_keys=Dict(),
        color="red",
        quants=[0.25],
        opacity=0.6,
        fade=0.6,
        buffer=0.0,
    )
    idxs = .&([df[!,key] .== val for (key,val) in include_keys]...)
    if !any(idxs)
        return PGFPlots.Axis()
    end
    # quantiles
    mat = vcat(map(v->reshape(v,1,:),df[idxs,obj])...)
    plts = Vector{PGFPlots.Plot}()
    for alpha in quants
        lo = map(j->Statistics.quantile(mat[:,j],alpha),1:size(mat,2)) .- buffer
        hi = map(j->Statistics.quantile(mat[:,j],1.0-alpha),1:size(mat,2)) .+ buffer
        c = "$(color)!$(fade)"
        # prepend because of overlaps
        append!(plts,[
            PGFPlots.Plots.Linear(hi,style="draw=$(c),draw opacity=0,mark=none,name path=A,forget plot"),
            PGFPlots.Plots.Linear(lo,style="draw=$(c),draw opacity=0,mark=none,name path=B,forget plot"),
            PGFPlots.Plots.Command("\\addplot[fill=$(c),fill opacity=$(opacity)] fill between[of=A and B];"),
            ])
    end
    PGFPlots.Axis(plts)
end

# For rendering the factory floor as a custom ImageAppearance in Webots
function caution_tape(d,n;yellow_color=RGB(0.8,0.7,0.0),black_color=RGB(0.0,0.0,0.0))
    compose(context(),map(i->(context(),
                (context(),polygon([
                            (0,(i-1)/n),
                            (d,(2*i-1)/(2*n)),
                            (d,i/n),
                            (0,(2*i-1)/(2*n))]),fill(yellow_color)),
            ), 1:n)...,
            (context(),polygon([(0,0),(0,1),(d,1),(d,0)]),fill(black_color))
    )
end
function taped_square(d=0.1;n=10,yellow_color=RGB(0.8,0.7,0.0),black_color=RGB(0.0,0.0,0.0))
    compose(
        context(),
        map(i->(context(rotation=Rotation(i*π/2,0.5,0.5)),
                caution_tape(d,n;yellow_color=yellow_color,black_color=black_color)), 0:3)...
    )
end
function render_factory_floor(env::GridFactoryEnvironment;
        d=0.1,n=10,yellow_color=RGB(0.8,0.7,0.0),black_color=RGB(0.0,0.0,0.0),
        floor_color=RGB(0.6,0.6,0.6)
    )
    x_dim = get_x_dim(env)
    y_dim = get_y_dim(env)
    compose(
        context(units=UnitBox()),
        map(vtx->(context((vtx[1]-1)/x_dim,(vtx[2]-1)/y_dim,1/x_dim,1/y_dim),
                taped_square(d;n=n,yellow_color=yellow_color,black_color=black_color)), get_pickup_vtxs(env))...,
        map(vtx->(context((vtx[1]-1)/x_dim,(vtx[2]-1)/y_dim,1/x_dim,1/y_dim),
                taped_square(d;n=n,yellow_color=yellow_color,black_color=black_color)), get_dropoff_vtxs(env))...,
        (context(), rectangle(),fill(floor_color))
    )
end

function get_node_shape(search_env::SearchEnv,graph,v,x,y,r)
    sched = get_schedule(search_env)
    cache = get_cache(search_env)
    n_id = get_vtx_id(sched,get_prop(graph,v,:vtx_id))
    if typeof(n_id) <: ActionID
        return Compose.circle(x,y,r)
    elseif typeof(n_id) <: RobotID
        return Compose.ngon(x,y,r,4)
    elseif typeof(n_id) <: ObjectID
        return Compose.ngon(x,y,r,3)
    elseif typeof(n_id) <: OperationID
        return Compose.circle(x,y,r)
    end
end
function get_node_color(search_env::SearchEnv,graph,v,x,y,r)
    sched = get_schedule(search_env)
    cache = get_cache(search_env)
    n_id = get_vtx_id(sched,get_prop(graph,v,:vtx_id))
    if typeof(n_id) <: ActionID
        return "cyan"
    elseif typeof(n_id) <: CleanUpBotID
        return "purple"
    elseif typeof(n_id) <: BotID
        return "lime"
    elseif typeof(n_id) <: ObjectID
        return "orange"
    elseif typeof(n_id) <: OperationID
        return "red"
    end
end
function get_node_text(search_env::SearchEnv,graph,v,x,y,r)
    sched = get_schedule(search_env)
    cache = get_cache(search_env)
    v_ = get_prop(graph,v,:vtx_id)
    string(get_t0(sched,v_)," - ",get_tF(sched,v_),"\n",get_local_slack(sched,v_)," - ",get_slack(sched,v_))
end

function show_times(sched::OperatingSchedule,v,scalar_slack=true)
    if scalar_slack
        slack = get_slack(sched,v)[1]
        slack_string = slack == Inf ? string(slack) : string(Int(slack))
    else
        slack_string = string(get_slack(sched,v))
    end
    return string(get_t0(sched,v),", ",get_tF(sched,v),", ",slack_string)
end

global TITLE_MODE = Dict()
_title_mode(n) = get(TITLE_MODE,typeof(n),get(TITLE_MODE,Any,:TYPE))
function _set_title_mode!(n,val)
    global TITLE_MODE
    TITLE_MODE[n] = val
end

global SUBTITLE_MODE = Dict()
_subtitle_mode(n) = get(SUBTITLE_MODE,typeof(n),get(SUBTITLE_MODE,Any,:TYPE))
function _set_subtitle_mode!(n,val)
    global SUBTITLE_MODE
    SUBTITLE_MODE[n] = val
end

GraphPlottingBFS._title_string(n::BOT_GO)       = _title_mode(n) == :TYPE ? "G" : "G"
GraphPlottingBFS._title_string(n::BOT_COLLECT)  = _title_mode(n) == :TYPE ? "C" : "C"
GraphPlottingBFS._title_string(n::BOT_CARRY)    = _title_mode(n) == :TYPE ? "T" : "T"
GraphPlottingBFS._title_string(n::BOT_DEPOSIT)  = _title_mode(n) == :TYPE ? "D" : "D"
GraphPlottingBFS._title_string(n::Operation)    = _title_mode(n) == :TYPE ? "OP" : get_id(get_operation_id(n))
GraphPlottingBFS._title_string(n::OBJECT_AT)    = _title_mode(n) == :TYPE ? "O" : get_id(get_object_id(n))
GraphPlottingBFS._title_string(n::BOT_AT)       = _title_mode(n) == :TYPE ? "R" : get_id(get_robot_id(n))

for op in (
    :(GraphPlottingBFS._node_shape),
    :(GraphPlottingBFS._node_color),
    # :(GraphPlottingBFS._node_bg_color),
    :(GraphPlottingBFS._title_string),
    :(GraphPlottingBFS._text_color),
    :(GraphPlottingBFS._subtitle_text_scale),
    # :(GraphPlottingBFS._subtitle_string),
    # :(GraphPlottingBFS.draw_node)
    )
    @eval $op(n::ScheduleNode,args...) = $op(n.node,args...)
end


_info_string(n::AbstractPlanningPredicate)     = ""
_info_string(n::BOT_AT)        = "r$(get_id(get_robot_id(n))),x$(get_id(get_initial_location_id(n)))"
_info_string(n::OBJECT_AT)     = "o$(get_id(get_object_id(n))),x$(get_id(get_initial_location_id(n)))"
_info_string(n::BOT_GO)        = "r$(get_id(get_robot_id(n))),x$(get_id(get_initial_location_id(n)))=>x$(get_id(get_destination_location_id(n)))"
_info_string(n::BOT_COLLECT)   = "r$(get_id(get_robot_id(n))),o$(get_id(get_object_id(n))),x$(get_id(get_initial_location_id(n)))"
_info_string(n::BOT_DEPOSIT)   = "r$(get_id(get_robot_id(n))),o$(get_id(get_object_id(n))),x$(get_id(get_initial_location_id(n)))"
_info_string(n::BOT_CARRY)     = "r$(get_id(get_robot_id(n))),o$(get_id(get_object_id(n))),x$(get_id(get_initial_location_id(n)))=>x$(get_id(get_destination_location_id(n)))"
_time_summary(n::ScheduleNode) = "t: $(get_t0(n)), $(get_tF(n))"
_info_string(n::ScheduleNode) = string(_info_string(n.node),",",_time_summary(n))
GraphPlottingBFS._subtitle_string(n::ScheduleNode) = _subtitle_mode(n) == :INFO ? _info_string(n) : ""
GraphPlottingBFS._subtitle_string(n::AbstractPlanningPredicate) = _subtitle_mode(n) == :INFO ? _info_string(n) : ""

# global SPACE_GRAY = RGB(0.2,0.2,0.2)
# global BRIGHT_RED = RGB(0.6,0.0,0.2)
# global LIGHT_BROWN = RGB(0.6,0.3,0.2)
# global LIME_GREEN = RGB(0.2,0.6,0.2)
# global BRIGHT_BLUE = RGB(0.0,0.4,1.0)

GraphPlottingBFS._node_color(::BOT_AT)                      = FactoryRendering.get_render_param(:Color,:Robot)
GraphPlottingBFS._node_color(::OBJECT_AT)                   = FactoryRendering.get_render_param(:Color,:Object)
GraphPlottingBFS._node_color(::Operation)                   = RGB(0.8,0.0,0.2)
GraphPlottingBFS._node_bg_color(n::AbstractPlanningPredicate)= GraphPlottingBFS._node_color(n)
GraphPlottingBFS._node_shape(::Operation,args...)           = Compose.rectangle(0.0,0.0,1.0,1.0)
GraphPlottingBFS._node_color(::AbstractSingleRobotAction)   = RGB(0.0,0.4,1.0)
GraphPlottingBFS._node_color(::BOT_AT{R}) where {R<:CleanUpBot} = RGB(1.0,0.0,1.0)
GraphPlottingBFS._node_color(::A) where {R<:CleanUpBot,A<:AbstractSingleRobotAction{R}} = RGB(1.0,0.0,1.0)

GraphPlottingBFS._subtitle_text_scale(n::AbstractPlanningPredicate) = 0.3
GraphPlottingBFS._text_color(n::AbstractPlanningPredicate) = "black"

# function GraphPlottingBFS.display_graph(g::OperatingSchedule;
#         kwargs...)
#     draw_node_func=(_,v)->GraphPlottingBFS.draw_node(g,v)
# end
function GraphPlottingBFS.draw_node(g::OperatingSchedule,v,args...;kwargs...)
    draw_node(get_node(g,v))
end
function GraphPlottingBFS.draw_node(g::ProjectSpec,v,args...;kwargs...)
    draw_node(get_node(g,v))
end

function build_frame(ctx;
        h1 = Compose.default_graphic_height,
        w1 = Compose.default_graphic_width,
    )
    frame = (dims = (w1,h1),ctx=ctx)
end
function hstack_canvases(frame1,frame2;
        stackmode=:horizontal,
        align_mode=:centered_align,
        bgcolor="white",
    )
    dims1 = frame1.dims
    dims2 = frame2.dims

    max_dims = max.(dims1,dims2)
    sum_dims = dims1 .+ dims2

    # horizontal stack
    if stackmode == :horizontal
        dx = sum_dims[1]
        dy = max_dims[2]
    else
        dx = max_dims[1]
        dy = sum_dims[2]
    end
    _dims = (dx,dy) # of new canvas
    set_default_graphic_size(_dims...)
    _dims1 = dims1 ./ _dims
    _dims2 = dims2 ./ _dims


    if stackmode == :horizontal
        if align_mode == :centered_align
            _y1 = (1.0 - _dims1[2])/2
            _y2 = (1.0 - _dims2[2])/2
        elseif align_mode == :top_align
            _y1 = 0.0
            _y2 = 0.0
        elseif align_mode == :bottom_align
            _y1 = (1.0 - _dims1[2])
            _y2 = (1.0 - _dims2[2])
        end
        ctx1 = Compose.context(0.0,         _y1, _dims1...)
        ctx2 = Compose.context(_dims1[1],   _y2, _dims2...)
    else
        if align_mode == :centered_align
            _x1 = (1.0 - _dims1[1])/2
            _x2 = (1.0 - _dims2[1])/2
        elseif align_mode == :top_align
            _x1 = 0.0
            _x2 = 0.0
        elseif align_mode == :bottom_align
            _x1 = (1.0 - _dims1[1])
            _x2 = (1.0 - _dims2[1])
        end
        ctx1 = Compose.context(_x1, 0.0,        _dims1...)
        ctx2 = Compose.context(_x2, _dims1[2],  _dims2...)
    end

    if !(bgcolor === nothing)
        bg = (context(),Compose.rectangle(0,0,1,1),fill(bgcolor))
    else
        bg = (contect(),)
    end
    # composed context
    Compose.compose(context(),
        (ctx1,frame1.ctx),
        (ctx2,frame2.ctx),
        bg
    )
end

# function show_times(cache::PlanningCache,v)
#     slack = minimum(get_slack(sched,v))
#     slack_string = slack == Inf ? string(slack) : string(Int(slack))
#     return string(get_t0(sched,v),",",get_tF(sched,v),",",slack_string)
# end

function plot_project_schedule(sched::OperatingSchedule;
        mode=:root_aligned,
        verbose=true,
        shape_function = (G,v,x,y,r)->Compose.circle(x,y,r),
        color_function = (G,v,x,y,r)->get_prop(G,v,:color),
        text_function = (G,v,x,y,r)->string(
            title_string(
                get_node_from_id(sched,
                    get_vtx_id(sched, v)),
                verbose),
            "\n",show_times(sched,v)
            )
        )
    rg = get_display_metagraph(sched;
        # f=(v,p)->string(v,",",get_id(get_default_robot_id(get_node(sched,v)))
        )
    plot_graph_bfs(rg;
        mode=mode,
        shape_function=shape_function,
        color_function=color_function,
        text_function=text_function
    )
    # `inkscape -z project_schedule1.svg -e project_schedule1.png`
    # OR: `for f in *.svg; do inkscape -z $f -e $f.png; done`
end
plot_project_schedule(search_env::SearchEnv;kwargs...) = plot_project_schedule(get_schedule(search_env);kwargs...)

function print_project_schedule(filename::String,args...;kwargs...)
    plot_project_schedule(args...;kwargs...) |> Compose.SVG(string(filename,".svg"))
end

function snapshot_color_func(env,t)
    sched = get_schedule(env)
    active = Set(filter(v->get_t0(sched,v) <= t < get_tF(sched,v) + maximum(get_local_slack(sched,v)),vertices(sched)))
    for v in collect(active)
        setdiff!(active,outneighbors(get_graph(sched),v))
    end
    # active = Set(filter(v->get_t0(sched,v) <= t <= get_tF(sched,v)))
    # active, fixed = get_active_and_fixed_vtxs(sched,cache,t)
    return (G,v,x,y,r)-> v in active ? get_prop(G,v,:color) : "gray"
end
function snapshot_text_func(env,t;verbose = true)
    sched = get_schedule(env)
    active, fixed = get_active_and_fixed_vtxs(sched,t)
    (G,v,x,y,r)->string(
    title_string(get_node_from_id(sched,get_vtx_id(sched, v)),verbose && (v in active)),
    "\n",show_times(sched,v)
    )
end
function plot_schedule_snapshot(env,t;
    mode=:leaf_aligned,
    verbose=true,
    color_function=snapshot_color_func(env,t),
    text_function=snapshot_text_func(env,t;verbose=verbose),
    kwargs...
    )
    plot_project_schedule(env;
        mode=:leaf_aligned,
        color_function=color_function,
        text_function=text_function,
        )
end

function convert_svg_file_to_png(path,outpath=string(splitext(path)[1],".png");
    width = 512,
    )
    run(`inkscape -z -w $width $(path) -e $(outpath)`)
end
function concat_image_files(path1,path2,outpath)
    run(`convert $(path1) $(path2) -append $(outpath)`)
end


"""
    get_graphic_dims

Return `width`, `height` to plot a graphic of size `dx × dy` at scale `scale`,
such that the aspect_ratio falls within appropriate bounds.
"""
function get_graphic_dims(dx,dy;
        scale = 1,
        desired_aspect = 0.7,
        aspect_bounds = [0.5,0.8],
    )
    aspect_ratio = min(max(desired_aspect, aspect_bounds[1]),aspect_bounds[2])
    gw = dx*scale
    gh = dy*scale
    if gh > gw*aspect_ratio
        gw = gh/aspect_ratio
    else
        gh = gw*aspect_ratio
    end
    gw,gh
end

"""
    render_env_and_schedule

Saves a ".png" file that shows the env state at time `t` as well as the schedule
state (i.e., the active nodes) at time `t`.
"""
function render_env_and_schedule(outpath,rstack,env::SearchEnv,t;
        pix_width = 512,
        env_scale = 1cm,
        min_env_aspect = 0.3,
        max_aspect = 0.5,
        base_dir = tempdir(),
        kwargs...
    )
    env_summary = SolutionSummary(env)
    sched_plot = plot_schedule_snapshot(env,t;kwargs...)
    dx,dy = Compose.default_graphic_width, Compose.default_graphic_height
    sched_plot |> SVG(joinpath(base_dir,"plot2.svg"),dx,dy)

    gw, gh = get_graphic_dims(get_graph(env).x_dim,get_graph(env).y_dim;
        scale=1cm,
        desired_aspect=max_aspect - dy/dx,
        aspect_bounds=[min_env_aspect,max_aspect]
    )
    Compose.set_default_graphic_size(gw,gh)
    env_plot = render_env(rstack,get_graph(env),env_summary,t)
    env_plot |> SVG(joinpath(base_dir,"plot1.svg"),gw,gh)

    convert_svg_file_to_png(joinpath(base_dir,"plot1.svg");width=pix_width)
    convert_svg_file_to_png(joinpath(base_dir,"plot2.svg");width=pix_width)
    concat_image_files(joinpath(base_dir,"plot1.png"),joinpath(base_dir,"plot2.png"),outpath)
end
