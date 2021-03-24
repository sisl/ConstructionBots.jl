using MeshCat
using Plots
using LightGraphs, GraphUtils
using GeometryBasics
using LDrawParser
using HierarchicalGeometry
using Colors

import Cairo #, Fontconfig
using GraphPlottingBFS
using Compose

struct AnimationWrapper
    anim::MeshCat.Animation
    step::Counter
end
AnimationWrapper(step::Int) = AnimationWrapper(MeshCat.Animation(),Counter(step))
current_frame(a::AnimationWrapper) = get_counter_status(a.step)
current_frame(::Nothing) = 0
step_animation!(::Nothing,step=1) = nothing
step_animation!(a::AnimationWrapper,step=1) = set_counter_status!(a.step,current_frame(a)+step)
set_current_frame!(a::AnimationWrapper,val) = set_counter_status!(a.step,val)

MeshCat.atframe(f, anim::AnimationWrapper, frame::Integer) = atframe(f,anim.anim,frame)
MeshCat.atframe(f, anim::Nothing, frame::Integer) = f()

"""
    construct_color_map(model_spec,id_map)

Given the color code in the `SubFileRef`s of model_spec, create a dict mapping
node ids to colors. Should only be applicable to models (not assemblies).
"""
function construct_color_map(model_spec,id_map)
    color_map = Dict{Union{String,AbstractID},AlphaColor}()
    ldraw_color_dict = LDrawParser.get_color_dict() 
    for n in get_nodes(model_spec)
        if isa(node_val(n),SubFileRef)
            color_key = node_val(n).color
            if haskey(id_map,node_id(n))
                id = id_map[node_id(n)]
                if haskey(ldraw_color_dict,color_key) && isa(id,ObjectID)
                    color_map[id] = ldraw_color_dict[color_key]
                    color_map[id_map[id]] = color_map[id]
                end
            end
        end
    end
    return color_map
end

"""
    populate_visualizer!(scene_tree,vis,id_map)

Populate the geometry (and transforms) of a MeshCat visualizer.
"""
function populate_visualizer!(scene_tree,vis;
        color_map=Dict(),
        material_type=nothing,
        wireframe=false,
        kwargs...
    )
    vis_root = vis["root"]
    vis_nodes = Dict{AbstractID,Any}()
    base_geom_nodes = Dict{AbstractID,Any}()
    for v in topological_sort_by_dfs(scene_tree)
        node = get_node(scene_tree,v)
        id = node_id(node)
        # if is_root_node(scene_tree,v)
        #     vis_nodes[id] = vis_root[string(id)]
        # else
        #     p = get_vtx_id(scene_tree,get_parent(scene_tree,v))
        #     vis_nodes[id] = vis_nodes[p][string(id)]
        # end
        vis_nodes[id] = vis_root[string(id)]
        vis_node = vis_nodes[id]
        geom_vis_node = vis_node["base_geom"]
        base_geom_nodes[id] = geom_vis_node
        # geometry
        geom = get_base_geom(node)
        if !(geom === nothing)
            if isa(geom,GeometryPrimitive)
                M = geom
            else
                filtered_geom = filter(m->!isa(m,GeometryBasics.Line),geom)
                M = GeometryBasics.Mesh(
                    coordinates(filtered_geom),
                    faces(filtered_geom)
                )
            end
            if !(material_type === nothing)
                mat = material_type(wireframe=wireframe,kwargs...)
                mat.color = get(color_map,id,mat.color)
                setobject!(geom_vis_node,M,mat)
            else
                setobject!(geom_vis_node,M)
            end
        end
        # settransform!(vis_node,local_transform(node))
        settransform!(vis_node,global_transform(node))
    end
    vis_nodes, base_geom_nodes
end

convert_to_renderable(geom::Ball2) = GeometryBasics.Sphere(geom)
convert_to_renderable(geom::Hyperrectangle) = GeometryBasics.HyperRectangle(geom)

function show_geometry_layer!(scene_tree,vis_nodes,key=HypersphereKey();
        color=RGBA{Float32}(0, 1, 0, 0.3),
        wireframe=true,
        material=MeshPhongMaterial(color=color,wireframe=wireframe),
    )
    geom_nodes = Dict{AbstractID,Any}()
    for (id,vis_node) in vis_nodes
        node = get_node(scene_tree,id)
        geom = get_base_geom(node,key)
        node_name = string(key)
        setobject!(vis_node[node_name],convert_to_renderable(geom),material)
        geom_nodes[id] = vis_node[node_name]
    end
    geom_nodes
end

function render_staging_areas!(vis,scene_tree,sched,staging_circles,root_key="staging_circles";
        color=RGBA{Float32}(1, 0, 1, 0.1),
        wireframe=false,
        material=MeshLambertMaterial(color=color,wireframe=wireframe),
        include_build_steps=true,
        dz=0.0,
    )
    staging_nodes = Dict{AbstractID,Any}()
    staging_vis = vis[root_key]
    for (id,geom) in staging_circles
        node = get_node(scene_tree,id)
        sphere = Ball2([geom.center..., 0.0],geom.radius)
        ctr = Point(HG.project_to_2d(sphere.center)..., 0.0)
        cylinder = Cylinder(ctr, Point((ctr.-[0.0,0.0,0.01])...),sphere.radius)
        setobject!(staging_vis[string(id)],
            cylinder,
            material,
            )
        staging_nodes[id] = staging_vis[string(id)]
        cargo = get_node(scene_tree,id)
        if isa(cargo,AssemblyNode) 
            cargo_node = get_node(sched,AssemblyComplete(cargo))
        else
            cargo_node = get_node!(sched,ObjectStart(cargo,TransformNode()))
        end
        t = HG.project_to_3d(HG.project_to_2d(global_transform(start_config(cargo_node)).translation))
        tform = CT.Translation(t) ∘ identity_linear_map()
        settransform!(staging_nodes[id],tform)
        # settransform!(staging_nodes[id],global_transform(start_config(cargo_node)))
    end
    zval=-dz
    for n in node_iterator(sched, topological_sort_by_dfs(sched))
    # for n in get_nodes(sched)
        if matches_template(OpenBuildStep,n)
            id = node_id(n)
            sphere = get_cached_geom(node_val(n).staging_circle)
            # ctr = Point(HG.project_to_2d(sphere.center)..., 0.0)
            ctr = Point(HG.project_to_2d(sphere.center)..., zval)
            zval -= dz
            cylinder = Cylinder(ctr, Point((ctr.-[0.0,0.0,0.01])...),sphere.radius)
            setobject!(staging_vis[string(id)],
                # convert_to_renderable(sphere),
                cylinder,
                material,
                )
            staging_nodes[id] = staging_vis[string(id)]
            # t = HG.project_to_3d(HG.project_to_2d(global_transform(start_config(cargo_node)).translation))
            # tform = CT.Translation(t) ∘ identity_linear_map()
            # settransform!(staging_nodes[id],tform)
        end
    end
    staging_nodes
end

"""
    visualize_staging_plan(vis,sched,scene_tree)

Render arrows pointing from start->goal for LiftIntoPlace nodes.
"""
function visualize_staging_plan(vis,sched,scene_tree;
        objects=false
    )
    vis_arrows = vis["arrows"]
    vis_triads = vis["triads"]
    bounding_vis = vis["bounding_circles"]
    delete!(vis_arrows)
    delete!(vis_triads)
    delete!(bounding_vis)
    for n in get_nodes(sched)
        if matches_template(LiftIntoPlace,n)
            # LiftIntoPlace path
            p1 = Point(global_transform(start_config(n)).translation...)
            p2 = Point(global_transform(goal_config(n)).translation...)
            lift_vis = ArrowVisualizer(vis_arrows[string(node_id(n))])
            setobject!(lift_vis,MeshPhongMaterial(color=RGBA{Float32}(0, 1, 0, 1.0)))
            settransform!(lift_vis,p1,p2)
            # Transport path
            cargo = entity(n)
            if isa(cargo,AssemblyNode)
                c = get_node(sched,AssemblyComplete(cargo))
                color = RGBA{Float32}(1, 0, 0, 1.0)
            elseif objects == true
                # continue
                c = get_node(sched,ObjectStart(cargo))
                color = RGBA{Float32}(0, 0, 1, 1.0)
            else
                continue
            end
            p3 = Point(global_transform(start_config(c)).translation...)
            transport_vis = ArrowVisualizer(vis_arrows[string(node_id(c))])
            setobject!(transport_vis,MeshPhongMaterial(color=color))
            settransform!(transport_vis,p3,p1)
        elseif matches_template(AssemblyComplete,n)
            triad_vis = vis_triads[string(node_id(n))]
            setobject!(triad_vis,Triad(0.25))
            settransform!(triad_vis,global_transform(goal_config(n)))
        end
    end
    for (id,geom) in bounding_circles
        if isa(id,AssemblyID)
            node = get_node(scene_tree,id)
        else
            node = get_node(scene_tree,node_val(get_node(sched,id)).assembly)
        end
        sphere = Ball2([geom.center..., 0.0],geom.radius)
        b_vis = bounding_vis[string(id)]
        setobject!(b_vis,
            convert_to_renderable(sphere),
            MeshPhongMaterial(color=RGBA{Float32}(1, 0, 1, 0.1)),
            )
        cargo_node = get_node(sched,AssemblyComplete(node))
        settransform!(b_vis,global_transform(start_config(cargo_node)))
    end
    return vis_triads, vis_arrows, bounding_vis
end

function animate_reverse_staging_plan!(vis,vis_nodes,scene_tree,sched,nodes=get_nodes(scene_tree);
    v_max = 1.0,
    ω_max = 1.0,
    ϵ_v = 1e-4,
    ϵ_ω = 1e-4,
    dt = 0.1,
    interp=false,
    interp_steps=10,
    objects=false,
    anim=nothing,
    )
    # HG.jump_to_final_configuration!(scene_tree;set_edges=true)
    graph = deepcopy(get_graph(scene_tree))
    # initialize goal sequences for each ObjectNode and AssemblyNode
    goals = Dict{AbstractID,Vector{CT.AffineMap}}()
    goal_idxs = Dict{AbstractID,Int}()
    interp_idxs = Dict{AbstractID,Int}()
    for node in nodes
        if matches_template(AssemblyNode,node)
            start_node = get_node(sched,AssemblyStart(node))
        elseif matches_template(ObjectNode,node)
            start_node = get_node(sched,ObjectStart(node))
        else
            continue
        end
        if !has_vertex(sched,node_id(LiftIntoPlace(node)))
            continue
        end
        lift_node = get_node(sched,LiftIntoPlace(node))
        goal_list = goals[node_id(node)] = Vector{CT.AffineMap}()
        goal_idxs[node_id(node)] = 1
        interp_idxs[node_id(node)] = interp_steps
        push!(goal_list, global_transform(start_config(start_node)))
        # push!(goal_list, global_transform(goal_config(lift_node)))
    end
    # animate
    active_set = get_all_root_nodes(graph)
    closed_set = Set{Int}()
    _close_node(graph,v,active_set,closed_set) = begin
        push!(closed_set,v)
        union!(active_set,outneighbors(graph,v))
    end
    while !isempty(active_set)
        setdiff!(active_set,closed_set)
        for v in collect(active_set)
            node = get_node(scene_tree,v)
            if !haskey(goals, node_id(node))
                _close_node(graph,v,active_set,closed_set)
                continue
            end
            goal_idx = goal_idxs[node_id(node)]
            if goal_idx > length(goals[node_id(node)])
                _close_node(graph,v,active_set,closed_set)
                continue
            end
            goal = goals[node_id(node)][goal_idx]
            tf_error = relative_transform(global_transform(node),goal)
            twist = optimal_twist(tf_error,v_max,ω_max,dt)
            if norm(twist.vel) <= ϵ_v && norm(twist.ω) <= ϵ_ω
                goal_idxs[node_id(node)] += 1
                interp_idxs[node_id(node)] = interp_steps
            end
            if interp
                isteps = interp_idxs[node_id(node)]
                @assert isteps > 0
                tform = HG.interpolate_transforms(global_transform(node),goal,1.0/isteps)
                interp_idxs[node_id(node)] -= 1
            else
                tform = global_transform(node) ∘ integrate_twist(twist,dt)
            end
            HG.set_desired_global_transform!(HG.get_transform_node(node), tform)
        end
        to_update = collect_descendants(graph,active_set,true)
        animate_update_visualizer!(scene_tree,vis_nodes,[get_node(scene_tree,v) for v in to_update];anim=anim)
        render(vis)
        sleep(dt)
    end
end

function staging_plan_bbox(staging_circs)
    xlo = minimum(c.center[1]-c.radius for (k,c) in staging_circs)
    ylo = minimum(c.center[2]-c.radius for (k,c) in staging_circs)
    xhi = maximum(c.center[1]+c.radius for (k,c) in staging_circs)
    yhi = maximum(c.center[2]+c.radius for (k,c) in staging_circs)
    GeometryBasics.Rect2D(xlo,ylo,xhi-xlo,yhi-ylo)
end

"""
    plot_staging_plan_2d(sched,scene_tree;

Plot the staging plan in 2D.
"""
function plot_staging_plan_2d(sched,scene_tree;
        nominal_width=10cm,
        _fontsize=20pt,
        _show_final_stages=true,
        _show_intermediate_stages=false,
        _show_bounding_circs=false,
        _show_dropoffs=false,
        _assembly_dropoffs_only=false,
        _show_base_geom=true,
        base_geom_layer=plot_assemblies(sched,scene_tree),
    )
    staging_circs = []
    final_staging_circs = []
    bounding_circs = []
    dropoff_circs = []
    base_geoms = []
    for n in node_iterator(sched,topological_sort_by_dfs(sched))
        if matches_template(CloseBuildStep,n)
            # if _show_intermediate_stages
                push!(staging_circs,node_id(n) => get_cached_geom(node_val(n).staging_circle))
            # end
            if _show_bounding_circs
                push!(bounding_circs,node_id(n) => get_cached_geom(node_val(n).bounding_circle))
            end
            if _show_final_stages
                is_terminal_build_step = true
                for v in outneighbors(sched,n)
                    if matches_template(OpenBuildStep,get_node(sched,v))
                        is_terminal_build_step = false
                    end
                end
                is_terminal_build_step ? nothing : continue
                push!(final_staging_circs,node_id(n) => get_cached_geom(node_val(n).staging_circle))
            end
        elseif matches_template(DepositCargo,n)
            if _show_dropoffs
                tu = entity(n)
                a = get_node(scene_tree,cargo_id(tu))
                if isa(a,AssemblyNode) || !_assembly_dropoffs_only
                    tf = global_transform(goal_config(n))
                    push!(dropoff_circs,node_id(n)=>tf(get_base_geom(tu,HypersphereKey())))
                end
            end
        end
    end
    bbox = staging_plan_bbox(staging_circs)
    Compose.set_default_graphic_size(nominal_width,(bbox.widths[2]/bbox.widths[1])*nominal_width)
    text_ctx = (context(),
        Compose.fontsize(_fontsize),
            (context(),[Compose.text(c.center[1],c.center[2],
                string(get_id(node_id(node_val(get_node(sched,k)).assembly))),
                hcenter,vcenter,
                ) for (k,c) in final_staging_circs]...,
            Compose.fill("black")
            ),
            (context(),[Compose.text(c.center[1],c.center[2],
                string(get_id(k)),
                hcenter,vcenter,
                ) for (k,c) in dropoff_circs if _show_dropoffs]...,
            Compose.fill(RGB(0.0,1.0,0.0)),
            ),
        )
    circles_ctx = (context(),
        (context(),
        [Compose.circle(c.center[1],c.center[2],c.radius) for (k,c) in dropoff_circs]...,
        Compose.stroke("green"),
        Compose.fill(RGBA(0.0,1.0,0.0,0.5)),
        ),
        (context(),
        [Compose.circle(c.center[1],c.center[2],c.radius) for (k,c) in bounding_circs]...,
        Compose.stroke("blue"),
        Compose.fill(RGBA(0.0,0.0,1.0,0.5)),
        ),
        (context(),
        [Compose.circle(c.center[1],c.center[2],c.radius) for (k,c) in final_staging_circs]...,
        Compose.stroke("red"),
        Compose.fill(RGBA(1.0,0.0,0.0,0.5)),
        ),
        (context(),
        [Compose.circle(c.center[1],c.center[2],c.radius) for (k,c) in staging_circs if _show_intermediate_stages]...,
        Compose.stroke("yellow"),
        Compose.fill(RGBA(1.0,1.0,0.0,0.5)),
        )
    )
    Compose.compose(
        context(units=UnitBox(bbox.origin...,bbox.widths...)),
        base_geom_layer,
        text_ctx,
        circles_ctx,
    )
end

function plot_base_geom_2d(node,scene_tree;
        fill_color=RGBA(1.0,0.0,0.0,0.5),
        stroke_color=RGBA(0.0,0.0,0.0,1.0),
        _show_lines=false,
        _show_triangles=false,
        _show_quadrilaterals=true,
        _use_unit_box=false,
        tform=identity_linear_map(),
        geom_key=BaseGeomKey(),
    )
    forms = Vector{Compose.Form}()
    for geom_element in recurse_child_geometry(node,scene_tree,geom_key)
        if isa(geom_key,BaseGeomKey)
            for geom in geom_element
                if isa(geom,GeometryBasics.Line) && _show_lines
                    push!(forms,Compose.line(Vector([(p[1],p[2]) for p in tform(geom).points])))
                elseif isa(geom,Triangle) && _show_triangles
                    push!(forms,Compose.polygon(Vector([(p[1],p[2]) for p in tform(geom).points])))
                elseif isa(geom,GeometryBasics.Quadrilateral) && _show_quadrilaterals
                    push!(forms,Compose.polygon(Vector([(p[1],p[2]) for p in tform(geom).points])))
                end
            end
        elseif isa(geom_key,HyperrectangleKey)
            r = tform(geom_element)
            push!(forms,Compose.rectangle(
                    r.center[1]-r.radius[1],
                    r.center[2]-r.radius[2],
                    2*r.radius[1],
                    2*r.radius[2],
                ))
        end
    end
    if _use_unit_box
        r = get_base_geom(node,HyperrectangleKey())
        unit_box = unit_box_from_rect(r)
        ctx = context(units=unit_box)
    else
        ctx = context()
    end
    ctx = context()
    Compose.compose(
        ctx,
        forms...,
        Compose.fill(fill_color),
        Compose.stroke(stroke_color)
        )
end
function unit_box_from_rect(r)
    unit_box = UnitBox(
        r.center[1]-r.radius[1],
        r.center[2]-r.radius[2],
        2*r.radius[1],
        2*r.radius[2],
        )
end
function plot_assemblies(sched,scene_tree;
        base_ctx=context(),
        kwargs...,
        )
    ctxs = []
    for n in get_nodes(sched)
        if matches_template(AssemblyComplete,n)
            tform = global_transform(goal_config(n))
            push!(ctxs,plot_base_geom_2d(entity(n),scene_tree;tform=tform,kwargs...))
        end
    end
    Compose.compose(base_ctx,ctxs...)
end


"""
    animate_preprocessing_steps!

Step through the different phases of preprocessing
"""
function animate_preprocessing_steps!(
        vis,
        vis_nodes,
        scene_tree,
        sched,
        rect_nodes,
        ;
        dt_animate=0.0,
        dt=0.0,
        anim=nothing
    )

    atframe(anim,current_frame(anim)) do
        # Hide robots 
        for n in get_nodes(scene_tree)
            if isa(n,Union{TransportUnitNode,RobotNode})
                setvisible!(vis_nodes[node_id(n)],false)
            end
        end
        setvisible!(base_geom_nodes,true)
        setvisible!(rect_nodes,false)
        HG.jump_to_final_configuration!(scene_tree;set_edges=true)
        update_visualizer!(scene_tree,vis_nodes)
    end
    step_animation!(anim)
    # Begin video
    for n in get_nodes(scene_tree)
        if isa(n,Union{AssemblyNode,ObjectNode})
            atframe(anim,current_frame(anim)) do
                setvisible!(rect_nodes[node_id(n)],true)
                setvisible!(base_geom_nodes[node_id(n)],false)
                update_visualizer!(scene_tree,vis_nodes,[n])
            end
            step_animation!(anim)
            sleep(dt_animate)
        end
    end
    # Show staging plan
    animate_reverse_staging_plan!(vis,vis_nodes,scene_tree,sched,
        filter(n->isa(n,AssemblyNode),get_nodes(scene_tree))
        ;
        dt=dt, interp=true, interp_steps=40, anim=anim,
    )
    # Animate objects moving to their starting positions
    for n in get_nodes(scene_tree)
        if isa(n,ObjectNode)
            atframe(anim,current_frame(anim)) do
                setvisible!(base_geom_nodes[node_id(n)],true)
                setvisible!(rect_nodes[node_id(n)],false)
                update_visualizer!(scene_tree,vis_nodes,[n])
            end
            step_animation!(anim)
            sleep(dt_animate)
        end
    end
    animate_reverse_staging_plan!(vis,vis_nodes,scene_tree,sched,
        filter(n->isa(n,ObjectNode),get_nodes(scene_tree));
        dt=0.0, interp=true, interp_steps=80, anim=anim,
    )
    atframe(anim,current_frame(anim)) do
        setvisible!(rect_nodes,false)
        setvisible!(base_geom_nodes,true)
    end
    step_animation!(anim)
    # Make robots visible again
    for n in get_nodes(scene_tree)
        if isa(n,Union{TransportUnitNode,RobotNode})
            atframe(anim,current_frame(anim)) do
                setvisible!(vis_nodes[node_id(n)],true)
                update_visualizer!(scene_tree,vis_nodes,[n])
            end
            step_animation!(anim)
            sleep(dt_animate)
        end
    end
    atframe(anim,current_frame(anim)) do
        set_scene_tree_to_initial_condition!(scene_tree,sched;remove_all_edges=true)
        update_visualizer!(scene_tree,vis_nodes)
    end
    step_animation!(anim)
    vis
end

function MeshCat.setvisible!(vis_nodes::Dict{AbstractID,Any},val)
    for (k,v) in vis_nodes
        setvisible!(v,val)
    end
end

"""
    update_visualizer!(scene_tree,vis_nodes)

Update the MeshCat transform tree.
"""
function update_visualizer!(scene_tree,vis_nodes,nodes=get_nodes(scene_tree))
    for n in nodes
        # settransform!(vis_nodes[node_id(n)],local_transform(n))
        settransform!(vis_nodes[node_id(n)],global_transform(n))
    end
    return vis_nodes
end

function animate_update_visualizer!(args...;anim=nothing,step=1)
    if anim === nothing
        return update_visualizer!(args...)
    else
        atframe(anim.anim,current_frame(anim)) do
            return update_visualizer!(args...)
        end
        step_animation!(anim,step)
    end
end

function construct_visualizer_update_function(vis,vis_nodes,staging_nodes;
        render_stages=true,
        anim=nothing,
    )
    update_visualizer_function(env,newly_updated=Set{Int}()) = begin
        atframe(anim,current_frame(anim)) do
            agents = Set{SceneNode}()
            for id in get_vtx_ids(ConstructionBots.rvo_global_id_map())
                agent = get_node(env.scene_tree, id)
                push!(agents, agent)
                for vp in collect_descendants(env.scene_tree,agent)
                    push!(agents,get_node(env.scene_tree,vp))
                end
            end
            if render_stages
                closed_steps = setdiff(keys(staging_nodes),env.active_build_steps)
                for id in closed_steps
                    setvisible!(staging_nodes[id],false)
                end
                for id in env.active_build_steps
                    setvisible!(staging_nodes[id],true)
                end
            end
            for id in env.active_build_steps
                setvisible!(staging_nodes[id],true)
            end
            for v in union(env.cache.active_set,newly_updated)
                node = get_node(env.sched,v)
                if matches_template(EntityGo,node)
                    agent = entity(node)
                elseif matches_template(Union{FormTransportUnit,DepositCargo},node)
                    agent = get_node(env.scene_tree,cargo_id(entity(node)))
                else
                    agent = nothing
                end
                if !(agent === nothing) && !(agent in agents)
                    push!(agents,agent)
                    for vp in collect_descendants(env.scene_tree,agent)
                        push!(agents,get_node(env.scene_tree,vp))
                    end
                end
            end
            update_visualizer!(env.scene_tree,vis_nodes,agents)
            render(vis)
        end
        step_animation!(anim)
    end
    # if !(anim === nothing)
    #     anim_function(args...) = begin
    #         atframe(anim.anim, current_frame(anim)) do
    #             update_visualizer_function(args)
    #         end
    #     end
    # end
    return update_visualizer_function
end

function call_update!(scene_tree,vis_nodes,nodes,dt)
    update_visualizer!(scene_tree,vis_nodes,nodes)
    render(vis)
    sleep(dt)
end

function visualize_construction_plan!(scene_tree,sched,vis,vis_nodes;
    dt=0.2,
    )
    for v in topological_sort_by_dfs(sched)
        update = true
        node = get_node(sched,v)
        update_nodes = []
        if matches_template(ProjectComplete,node)
        # elseif matches_template(RobotStart,node)
        #     robot_node = entity(node)
        #     @assert has_parent(robot_node,robot_node)
        # elseif matches_template(ObjectStart,node)
        #     part_node = get_node(scene_tree,node_id(entity(node)))
        #     @assert has_parent(part_node,part_node)
        #     set_local_transform!(part_node,local_transform(goal_config(node)))
        elseif matches_template(RobotGo,node)
            robot_node = entity(node)
            @assert has_parent(robot_node,robot_node)
            set_local_transform!(robot_node,global_transform(goal_config(node)))
            push!(update_nodes,robot_node)
        elseif matches_template(FormTransportUnit,node)
            transport_unit = entity(node)
            part_node = get_node(scene_tree,cargo_id(transport_unit))
            @assert has_parent(part_node,part_node)
            @assert has_parent(transport_unit,transport_unit)
            set_local_transform!(transport_unit,global_transform(goal_config(node)))
            set_child!(scene_tree,transport_unit,part_node)
            @assert has_parent(part_node,transport_unit)
            append!(update_nodes,[transport_unit,part_node])
        elseif matches_template(TransportUnitGo,node)
            transport_unit = entity(node)
            set_local_transform!(transport_unit,global_transform(goal_config(node)))
            append!(update_nodes,[transport_unit])
        elseif matches_template(DepositCargo,node)
            transport_unit = entity(node)
            part_node = get_node(scene_tree,cargo_id(transport_unit))
            @assert has_parent(part_node,transport_unit)
            disband!(scene_tree,transport_unit)
            @assert has_parent(part_node,part_node)
            set_local_transform!(part_node,global_transform(cargo_deployed_config(node)))
            append!(update_nodes,[transport_unit,part_node])
        elseif matches_template(LiftIntoPlace,node)
            part_node = get_node(scene_tree,node_id(entity(node)))
            @assert has_parent(part_node,part_node)
            set_local_transform!(part_node,global_transform(goal_config(node)))
            if matches_template(AssemblyNode,part_node)
                @info "$(string(node_id(node)))" part_node
            end
            append!(update_nodes,[part_node])
        elseif matches_template(CloseBuildStep,node)
            for (id,_) in assembly_components(node)
                set_child!(scene_tree,ConstructionBots.get_assembly(node),id)
                push!(update_nodes,get_node(scene_tree,id))
            end
        end
        if !isempty(update_nodes)
            # id = node_id(node)
            # ids = map(n->string(node_id(n))=>string(global_transform(n).translation), update_nodes)
            # @info "Updating nodes" id ids
            # call_update!(scene_tree,vis_nodes,update_nodes,dt)
            update_visualizer!(scene_tree,vis_nodes)
            render(vis)
            sleep(dt)
        end
    end
end

GraphUtils.get_id(s::String) = s
# Rendering schedule nodes
GraphPlottingBFS._title_string(n::S) where {S<:SceneNode} = split(string(typeof(n)),".")[end][1]
GraphPlottingBFS._title_string(n::RobotNode)            = "R"
GraphPlottingBFS._title_string(n::ObjectNode)           = "O"
GraphPlottingBFS._title_string(n::AssemblyNode)         = "A"
GraphPlottingBFS._title_string(n::TransportUnitNode)    = "T"

GraphPlottingBFS._title_string(::BuildingStep)          = "B"
GraphPlottingBFS._title_string(::SubFileRef)            = "S"
GraphPlottingBFS._title_string(::SubModelPlan)          = "M"

GraphPlottingBFS._title_string(n::ConstructionBots.EntityConfigPredicate) = _title_string(n.entity)
GraphPlottingBFS._title_string(::ConstructionBots.RobotStart)        = "R"
GraphPlottingBFS._title_string(n::ConstructionBots.ObjectStart)      = "O"
GraphPlottingBFS._title_string(::ConstructionBots.AssemblyStart)     = "M"
GraphPlottingBFS._title_string(::ConstructionBots.AssemblyComplete)  = "M"
GraphPlottingBFS._title_string(::ConstructionBots.OpenBuildStep)     = "oB"
GraphPlottingBFS._title_string(::ConstructionBots.CloseBuildStep)    = "cB"
GraphPlottingBFS._title_string(::ConstructionBots.RobotGo)           = "G"
GraphPlottingBFS._title_string(::ConstructionBots.FormTransportUnit) = "F"
GraphPlottingBFS._title_string(::ConstructionBots.DepositCargo)      = "D"
GraphPlottingBFS._title_string(::ConstructionBots.TransportUnitGo)   = "T"
GraphPlottingBFS._title_string(::ConstructionBots.LiftIntoPlace)     = "L"
GraphPlottingBFS._title_string(::ConstructionBots.ProjectComplete)   = "P"

for op in (
    :(GraphPlottingBFS._node_shape),
    :(GraphPlottingBFS._node_color),
    :(GraphPlottingBFS._title_string),
    :(GraphPlottingBFS._subtitle_string)
    )
    @eval $op(n::CustomNode,args...) = $op(node_val(n),args...)
    @eval $op(n::ScheduleNode,args...) = $op(n.node,args...)
end
# GraphPlottingBFS._node_shape(n::CustomNode,args...) = GraphPlottingBFS._node_shape(node_val(n),args...)
# GraphPlottingBFS._node_color(n::CustomNode,args...) = GraphPlottingBFS._node_color(node_val(n),args...)
# GraphPlottingBFS._title_string(n::CustomNode,args...) = GraphPlottingBFS._title_string(node_val(n),args...)

GraphPlottingBFS._subtitle_string(n::SceneNode) = "$(get_id(node_id(n)))"
GraphPlottingBFS._subtitle_string(n::Union{EntityGo,EntityConfigPredicate,FormTransportUnit,DepositCargo}) = GraphPlottingBFS._subtitle_string(entity(n))
GraphPlottingBFS._subtitle_string(n::BuildPhasePredicate) = GraphPlottingBFS._subtitle_string(n.assembly)
GraphPlottingBFS._subtitle_string(n::ObjectNode) = "o$(get_id(node_id(n)))"
GraphPlottingBFS._subtitle_string(n::RobotNode) = "r$(get_id(node_id(n)))"
GraphPlottingBFS._subtitle_string(n::AssemblyNode) = "a$(get_id(node_id(n)))"
GraphPlottingBFS._subtitle_string(n::TransportUnitNode) = "t$(get_id(node_id(n)))"

SPACE_GRAY = RGB(0.2,0.2,0.2)
BRIGHT_RED = RGB(0.6,0.0,0.2)
LIGHT_BROWN = RGB(0.6,0.3,0.2)
LIME_GREEN = RGB(0.2,0.6,0.2)
BRIGHT_BLUE = RGB(0.0,0.4,1.0)

GraphPlottingBFS._node_color(::RobotNode)                           = SPACE_GRAY
GraphPlottingBFS._node_color(::ObjectNode)                          = SPACE_GRAY
GraphPlottingBFS._node_color(::AssemblyNode)                        = BRIGHT_BLUE
GraphPlottingBFS._node_color(::TransportUnitNode)                   = LIME_GREEN

GraphPlottingBFS._node_color(::BuildingStep)                        = LIGHT_BROWN
GraphPlottingBFS._node_color(::SubFileRef)                          = BRIGHT_RED
GraphPlottingBFS._node_color(::SubModelPlan)                        = SPACE_GRAY

GraphPlottingBFS._node_color(::ConstructionBots.EntityConfigPredicate) = _node_color(n.entity)
GraphPlottingBFS._node_color(::ConstructionBots.RobotStart)         = SPACE_GRAY
GraphPlottingBFS._node_color(::ConstructionBots.ObjectStart)        = SPACE_GRAY
GraphPlottingBFS._node_color(::ConstructionBots.AssemblyStart)      = SPACE_GRAY
GraphPlottingBFS._node_color(::ConstructionBots.AssemblyComplete)   = SPACE_GRAY
GraphPlottingBFS._node_color(::ConstructionBots.OpenBuildStep)      = LIGHT_BROWN
GraphPlottingBFS._node_color(::ConstructionBots.CloseBuildStep)     = LIGHT_BROWN
GraphPlottingBFS._node_color(::ConstructionBots.ProjectComplete)    = SPACE_GRAY
GraphPlottingBFS._node_color(::ConstructionBots.RobotGo)            = LIME_GREEN
GraphPlottingBFS._node_color(::ConstructionBots.FormTransportUnit)  = LIME_GREEN
GraphPlottingBFS._node_color(::ConstructionBots.TransportUnitGo)    = LIME_GREEN
GraphPlottingBFS._node_color(::ConstructionBots.DepositCargo)       = LIME_GREEN
GraphPlottingBFS._node_color(::ConstructionBots.LiftIntoPlace)      = BRIGHT_BLUE

function GraphPlottingBFS.draw_node(g::AbstractCustomNGraph,v,args...;kwargs...) 
    GraphPlottingBFS.draw_node(get_node(g,v),args...;kwargs...)
end