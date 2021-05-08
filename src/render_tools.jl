using MeshCat
using Plots
using LightGraphs, GraphUtils
using GeometryBasics
using LDrawParser
using HierarchicalGeometry
using Colors
using FactoryRendering
using Printf
using Parameters

const FR = FactoryRendering

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

@with_kw mutable struct FactoryVisualizer
    vis             = nothing
    scene_tree      = SceneTree()
    vis_nodes       = Dict{AbstractID,Any}() # Dict from AbstractID -> vis[string(id)]
    geom_nodes      = Dict{HG.GeometryKey,Dict{AbstractID,Any}}() # Dict containing the overapproximation layers (HypersphereKey,etc.)
    active_flags    = Dict{AbstractID,Any}() # contains vis nodes for flagging agents as active or not
    staging_nodes   = Dict{AbstractID,Any}()
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
                mat = material_type(;wireframe=wireframe,kwargs...)
                mat.color = get(color_map,id,mat.color)
                setobject!(geom_vis_node,M,mat)
            else
                setobject!(geom_vis_node,M)
            end
        end
        # settransform!(vis_node,local_transform(node))
        settransform!(vis_node,global_transform(node))
    end
    factory_vis = FactoryVisualizer(
        vis=vis,
        scene_tree=scene_tree,
        vis_nodes=vis_nodes
        )
    factory_vis.geom_nodes[BaseGeomKey()] = base_geom_nodes 
    factory_vis
end

function add_indicator_nodes!(factory_vis;
        cylinder_depth=0.0025,
        cylinder_radius_pad=0.1,
        active_color=RGB(0.0,1.0,0.0),
        kwargs...
        )
    @unpack vis, vis_nodes, scene_tree = factory_vis
    active_flags = Dict{AbstractID,Any}()
    for n in get_nodes(scene_tree)
        if matches_template(Union{TransportUnitNode,RobotNode},n)
            vis_node = vis_nodes[node_id(n)]
            geom = get_base_geom(n,HypersphereKey())
            cylinder = GeometryBasics.Cylinder(
                Point(HG.get_center(geom)...),
                Point((HG.get_center(geom) .- [0.0,0.0,cylinder_depth])...),
                HG.get_radius(geom)+cylinder_radius_pad,
            )
            setobject!(vis_node["active"],
                cylinder,
                MeshLambertMaterial(color=active_color,kwargs...))
            active_flags[node_id(n)] = vis_node["active"]
        end
    end
    factory_vis.active_flags = active_flags
    factory_vis
end

convert_to_renderable(geom) = geom
convert_to_renderable(geom::Ball2) = GeometryBasics.Sphere(geom)
convert_to_renderable(geom::Hyperrectangle) = GeometryBasics.HyperRectangle(geom)
convert_to_renderable(geom::HG.BufferedPolygonPrism) = GeometryBasics.Mesh(coordinates(geom),faces(geom))

function show_geometry_layer!(factory_vis,key=HypersphereKey();
        color=RGBA{Float32}(0, 1, 0, 0.3),
        wireframe=true,
        material=MeshLambertMaterial(color=color,wireframe=wireframe),
    )
    @unpack vis_nodes, scene_tree = factory_vis
    geom_nodes = Dict{AbstractID,Any}()
    for (id,vis_node) in vis_nodes
        node = get_node(scene_tree,id)
        geom = get_base_geom(node,key)
        node_name = string(key)
        setobject!(vis_node[node_name],convert_to_renderable(geom),material)
        geom_nodes[id] = vis_node[node_name]
    end
    factory_vis.geom_nodes[key] = geom_nodes
    factory_vis
end

"""
    rotate_camera!(vis,anim;

Rotate camera at fixed angular rate while keeping focus on a given point
"""
function rotate_camera!(vis,anim;
    rc = 10,
    dθ = 0.005,
    hc = 2,
    θ_start=0,
    k_start=0,
    k_final=current_frame(anim),
    radial_decay_factor=0.0,
    origin=[0.0,0.0,0.0],
    )
    θ = θ_start
    for k in k_start:k_final
        rc -= k*radial_decay_factor # spiral
        # Y-coord is up for camera
        camera_tform = CT.Translation(rc*cos(θ),hc,-rc*sin(θ)) 
        θ = wrap_to_pi(θ+dθ)
        atframe(anim,k) do
            setprop!(vis["/Cameras/default/rotated/<object>"],"position",camera_tform.translation)
            settransform!(vis["/Cameras/default"],CT.Translation(origin...))
        end
    end
    anim
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
        # push!(goal_list, global_transform(start_config(lift_node)))
        push!(goal_list, global_transform(start_config(start_node)))
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
            # @show tf_error, twist
            if norm(twist.vel) <= ϵ_v && norm(twist.ω) <= ϵ_ω || interp_idxs[node_id(node)] == 0
                goal_idxs[node_id(node)] += 1
                interp_idxs[node_id(node)] = interp_steps
            end
            if interp
                isteps = interp_idxs[node_id(node)]
                # @show summary(node_id(node)), goal_idxs[node_id(node)], goal, isteps
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
staging_plan_bbox(sched::AbstractGraph) = staging_plan_bbox(extract_build_step_staging_circles(sched))

function extract_build_step_staging_circles(sched)
    circs = Dict()
    for n in get_nodes(sched)
        if matches_template(CloseBuildStep,n)
            circs[node_id(n)] = get_cached_geom(node_val(n).staging_circle)
        end
    end
    circs
end

function transform_final_assembly_staging_circles(sched,scene_tree,staging_circles)
    circs = Dict()
    for (k,v) in staging_circles
        tform = global_transform(goal_config(get_node(sched,
            AssemblyComplete(get_node(scene_tree,k)))))
        circ = LazySets.translate(v,tform.translation[1:2])
        circs[k] = circ
    end
    circs
end

"""
    plot_staging_plan_2d(sched,scene_tree;

Plot the staging plan in 2D.
"""
function plot_staging_plan_2d(sched,scene_tree;
        nominal_width=10cm,
        _fontsize=20pt,
        _show_final_stages=true,
        _final_stage_stroke_color=FR.get_render_param(:Color,:Stroke,:StagingCircle,:Final,default="red"),
        _final_stage_bg_color=FR.get_render_param(:Color,:Fill,:StagingCircle,:Final,default=RGBA(1.0,0.0,0.0,0.5)),
        _stage_label_color=FR.get_render_param(:Color,:Label,:StagingCircle,:Final,default="black"),
        _show_intermediate_stages=false,
        _stage_stroke_color=FR.get_render_param(:Color,:Stroke,:StagingCircle,default="yellow"),
        _stage_bg_color=FR.get_render_param(:Color,:Fill,:StagingCircle,default=RGBA(1.0,1.0,0.0,0.5)),
        _show_bounding_circs=false,
        _bounding_stroke_color=FR.get_render_param(:Color,:Stroke,:BoundingCircle,default="blue"),
        _bounding_bg_color=FR.get_render_param(:Color,:Fill,:BoundingCircle,default=RGBA(0.0,0.0,1.0,0.5)),
        _show_dropoffs=false,
        _dropoff_stroke_color=FR.get_render_param(:Color,:Stroke,:DropoffCircle,default="green"),
        _dropoff_bg_color=FR.get_render_param(:Color,:Fill,:DropoffCircle,default=RGBA(0.0,1.0,0.0,0.5)),
        _dropoff_label_color=FR.get_render_param(:Color,:Label,:DropoffCircle,default="black"),
        _assembly_dropoffs_only=false,
        _show_base_geom=true,
        base_geom_layer=plot_assemblies(sched,scene_tree),
        bg_color="white",
        text_placement_func=circ->circ.center[1:2],
        node_list=node_iterator(sched,topological_sort_by_dfs(sched)),
        bbox = staging_plan_bbox(sched),
    )
    staging_circs = []
    final_staging_circs = []
    bounding_circs = []
    dropoff_circs = []
    base_geoms = []
    for n in node_list
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
    # bbox = staging_plan_bbox(staging_circs)
    bg_ctx = (context(),Compose.rectangle(),Compose.fill(bg_color))
    Compose.set_default_graphic_size(nominal_width,(bbox.widths[2]/bbox.widths[1])*nominal_width)
    text_ctx = (context(),
        Compose.fontsize(_fontsize),
            (context(),[Compose.text(text_placement_func(c)...,
                string(get_id(node_id(node_val(get_node(sched,k)).assembly))),
                hcenter,vcenter,
                ) for (k,c) in final_staging_circs]...,
            Compose.fill(_stage_label_color),
            ),
            (context(),[Compose.text(text_placement_func(c)...,
                string(get_id(k)),
                hcenter,vcenter,
                ) for (k,c) in dropoff_circs if _show_dropoffs]...,
            Compose.fill(_dropoff_label_color),
            ),
        )
    circles_ctx = (context(),
        (context(),
        [Compose.circle(c.center[1],c.center[2],c.radius) for (k,c) in final_staging_circs]...,
        Compose.stroke(_final_stage_stroke_color),
        Compose.fill(_final_stage_bg_color),
        ),
        (context(),
        [Compose.circle(c.center[1],c.center[2],c.radius) for (k,c) in staging_circs if _show_intermediate_stages]...,
        Compose.stroke(_stage_stroke_color),
        Compose.fill(_stage_bg_color),
        ),
        (context(),
        [Compose.circle(c.center[1],c.center[2],c.radius) for (k,c) in dropoff_circs]...,
        Compose.stroke(_dropoff_stroke_color),
        Compose.fill(_dropoff_bg_color),
        ),
        (context(),
        [Compose.circle(c.center[1],c.center[2],c.radius) for (k,c) in bounding_circs]...,
        Compose.stroke(_bounding_stroke_color),
        Compose.fill(_bounding_bg_color),
        ),
    )
    Compose.compose(
        context(units=UnitBox(bbox.origin...,bbox.widths...)),
        text_ctx,
        base_geom_layer,
        circles_ctx,
        bg_ctx,
    )
end

function render_transport_unit_2d(scene_tree,tu,cvx_hull=[];
    scale=4cm,
    hull_thickness=0.01,
    line_thickness=0.0025,
    hull_color=RGB(0.0,1.0,0.0),
    config_pt_radius=2*hull_thickness,
    config_pt_color=FR.get_render_param(:Color,:Object),
    robot_color=FR.get_render_param(:Color,:Robot,default=RGB(0.0,0.0,1.0)),
    geom_fill_color=RGBA(0.9,0.9,0.9,0.3),
    geom_stroke_color=RGBA(0.0,0.0,0.0,1.0),
    robot_radius = FR.get_render_param(:Radius,:Robot,default=ROBOT_RADIUS),
    rect2d = HG.project_to_2d(get_base_geom(tu,HyperrectangleKey())),
    xpad=(0,0),
    ypad=(0,0),
    bg_color=nothing,
    overlay_for_single_robot=false,
    )
    if length(robot_team(tu)) == 1 && overlay_for_single_robot
        robot_overlay = (context(),
            [Compose.circle(HG.project_to_2d(tform.translation)...,ROBOT_RADIUS) for (id,tform) in robot_team(tu)]...,
            Compose.fill(RGBA(robot_color,0.8)),
        )
    else
        robot_overlay = (context(),)
    end
    set_default_graphic_size(rect2d.radius[1]*scale,rect2d.radius[2]*scale)
    Compose.compose(
        context(units=unit_box_from_rect(rect2d,xpad=xpad,ypad=ypad)),
        (context(),
            robot_overlay,
            plot_base_geom_2d(get_node(scene_tree,cargo_id(tu)),scene_tree;
                fill_color=geom_fill_color,
                stroke_color=geom_stroke_color,
            ),
            Compose.linewidth(line_thickness*max(h,w)),
            (context(),
            Compose.polygon([(p[1],p[2]) for p in cvx_hull]),
            Compose.stroke(hull_color),
            Compose.linewidth(hull_thickness*max(h,w)),
            Compose.fill(nothing),
            ),
            (context(),
                [Compose.circle(HG.project_to_2d(tform.translation)...,config_pt_radius*max(h,w)) for (id,tform) in robot_team(tu)]...,
                Compose.fill(config_pt_color),
            ),
        ),
        (context(),
            [Compose.circle(HG.project_to_2d(tform.translation)...,robot_radius) for (id,tform) in robot_team(tu)]...,
            Compose.fill(robot_color),
        ),
        (context(),Compose.rectangle(),Compose.fill(bg_color))
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
    polygon_pts = []
    for geom_element in recurse_child_geometry(node,scene_tree,geom_key)
        if isa(geom_key,BaseGeomKey)
            for geom in geom_element
                if isa(geom,GeometryBasics.Line) && _show_lines
                    push!(polygon_pts,Vector([(p[1],p[2]) for p in tform(geom).points]))
                elseif isa(geom,Triangle) && _show_triangles
                    push!(polygon_pts,Vector([(p[1],p[2]) for p in tform(geom).points]))
                elseif isa(geom,GeometryBasics.Quadrilateral) && _show_quadrilaterals
                    push!(polygon_pts,Vector([(p[1],p[2]) for p in tform(geom).points]))
                end
            end
        elseif isa(geom_key,HyperrectangleKey)
            r = tform(geom_element)
            push!(polygon_pts,[
                (r.center[1]-r.radius[1],r.center[2]-r.radius[2]),
                (r.center[1]-r.radius[1],r.center[2]+r.radius[2]),
                (r.center[1]+r.radius[1],r.center[2]+r.radius[2]),
                (r.center[1]+r.radius[1],r.center[2]-r.radius[2]),
            ])
        end
    end
    if _use_unit_box
        r = get_base_geom(node,HyperrectangleKey())
        unit_box = unit_box_from_rect(r)
        ctx = context(units=unit_box)
    else
        ctx = context()
    end
    Compose.compose(
        ctx,
        Compose.polygon(polygon_pts),
        Compose.fill(fill_color),
        Compose.stroke(stroke_color),
        )
end
function unit_box_from_rect(r;
        pad=0.0,
        xpad=(pad,pad),
        ypad=(pad,pad),
        )
    unit_box = UnitBox(
        r.center[1]-r.radius[1]-xpad[1],
        r.center[2]-r.radius[2]-ypad[1],
        2*r.radius[1]+sum(xpad),
        2*r.radius[2]+sum(ypad),
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

function render_node_types_and_table(
        sched,scene_tree;
        graphics_path=pwd(),
        robot=get_node(scene_tree,RobotID(1)),
        object=get_node(scene_tree,ObjectID(1)),
        assembly=get_node(scene_tree,AssemblyID(2)),
        tu = get_node(scene_tree,TransportUnitNode(assembly)),
    )
    mkpath(graphics_path)
    r = get_node(sched,RobotStart(robot))
    rg = get_node(sched,outneighbors(sched,r)[1])
    o = get_node(sched,ObjectStart(object))
    s = get_node(sched,AssemblyStart(assembly))
    c, f, tg, d, l = ConstructionBots.get_transport_node_sequence(sched,assembly)
    cb = get_node(sched,inneighbors(sched,c)[1])
    ob = get_node(sched,OpenBuildStep(node_val(cb)))
    p = get_node(sched,ProjectComplete(1))

    set_default_graphic_size(5cm,5cm)
    paths = Dict()
    for n in [r,rg,o,s,f,tg,d,l,c,p,ob,cb]
        plt = draw_node(n;subtitle_text="",title_scale=0.5)
        display(plt)
        path = joinpath(graphics_path,string(typeof(node_val(n)).name,".pdf"))
        paths[string(typeof(node_val(n)).name)] = path
        paths[node_id(n)] = path
        # draw node
        draw(PDF(path),plt)
    end
    # build table with required predecessos and successors
    neighbor_dict = Dict()
    xkeys = []
    name_func(k) = string("\\schednodeinline{$(paths[string(k)])}")
    # for n in [r,rg,o,s,f,tg,d,l,c,p,ob,cb]
    for n in [p,o,s,c,ob,cb,r,rg,f,tg,d,l]
        nname = name_func(typeof(node_val(n)).name)
        push!(xkeys,Dict(:type=>nname))
        my_dict = neighbor_dict[nname] = Dict()
        for (neighbor_list,neighbor_key) in [
                (required_predecessors(sched,n), "\\textsc{Req. Pred.}"),
                (required_successors(sched,n), "\\textsc{Req. Succ.}"),
                (eligible_predecessors(sched,n), "\\textsc{El. Pred.}"),
                (required_successors(sched,n), "\\textsc{El. Succ.}"),
            ]
            pred_strings = []
            for (k,v) in neighbor_list
                if v > 1
                    v = "\\textsc{variable}"
                end
                if isa(k,Union)
                    type_key = k
                    pred_list = []
                    while isa(type_key,Union)
                        push!(pred_list,name_func(type_key.a))
                        type_key = type_key.b
                    end 
                    push!(pred_list,name_func(type_key))
                    push!(pred_strings,string(join(pred_list," / ")," \$\\times\$ $v"))
                else
                    push!(pred_strings,string(name_func(k)," \$\\times\$ $v"))
                end
            end
            my_dict[neighbor_key] = ""
            if length(pred_strings) > 0
                cell_tab = init_table(
                    [Dict(:row=>k) for k in 1:length(pred_strings)],
                    [Dict(:col=>1)],
                )
                for (k,p) in enumerate(pred_strings)
                    cell_tab.data[k] = p
                end
                io = IOBuffer()
                print(io,"{\\renewcommand{\\arraystretch}{1.0}\n")
                write_tex_table(io,cell_tab;
                    print_func=print_real,
                    print_row_start=false,
                    print_column_labels=false,
                    alignspec="t"
                )
                print(io,"}\n")
                my_dict[neighbor_key] = String(take!(io))
            end
            # my_dict[neighbor_key] = join(pred_strings,"; ")
        end
    end
    # @show neighbor_dict
    # xkeys = [Dict(:type=>k) for k in sort(collect(keys(neighbor_dict)))]
    ykeys = [Dict(:direction=>k) for k in sort(collect(keys(collect(values(neighbor_dict))[1])))]
    tab = init_table(xkeys,ykeys)
    for (i,xdict) in enumerate(get_xkeys(tab))
        k = xdict[:type]
        dict = neighbor_dict[k]
        for (j,ydict) in enumerate(get_ykeys(tab))
            tab.data[i,j] = dict[ydict[:direction]]
        end
    end
    # tab.data
    # io = IOBuffer()
    # write_tex_table(io,tab;
    #     print_func=print)
    # println(String(take!(io)))
    write_tex_table(joinpath(graphics_path,"node_table.tex"),tab;
        # print_func=print,
        print_func=print_real,
        row_label_func=(k,v)->"\$$(v)\$",
        col_label_func=(k,v)->"\$$(v)\$",
        alignspec="t",
        group_delim=" ",
        )
end


"""
    animate_preprocessing_steps!

Step through the different phases of preprocessing
"""
function animate_preprocessing_steps!(
        factory_vis,sched,
        # vis,
        # vis_nodes,
        # scene_tree,
        # sched,
        # rect_nodes,
        # base_geom_nodes,
        ;
        dt_animate=0.0,
        anim=nothing,
        dt=0.0,
        interp_steps=40,
        kwargs...,
    )
    @unpack vis, vis_nodes, scene_tree, geom_nodes = factory_vis
    base_geom_nodes = geom_nodes[BaseGeomKey()]
    rect_nodes = geom_nodes[HyperrectangleKey()]

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
        anim=anim,
        interp=true, 
        dt=dt, 
        interp_steps=interp_steps, 
        kwargs...
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
function update_visualizer!(factory_vis::FactoryVisualizer,args...)
    update_visualizer!(factory_vis.scene_tree,factory_vis.vis_nodes,args...)
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

function visualizer_update_function!(factory_vis,env,newly_updated=Set{Int}();
        anim=nothing,
        render_stages=true,
        )
    @unpack vis, vis_nodes, staging_nodes = factory_vis
    if vis === nothing
        step_animation!(anim)
        return nothing
    end
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
                if matches_template(Union{RobotNode,TransportUnitID},agent)
                    build_step = ConstructionBots.get_parent_build_step(env.sched,node)
                    if !(build_step === nothing) && node_id(build_step) in env.active_build_steps
                        setvisible!(factory_vis.active_flags[node_id(agent)],true)
                    else
                        setvisible!(factory_vis.active_flags[node_id(agent)],false)
                    end
                end
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
        # show active flags
        # for agent in agents
        #     build_step = ConstructionBots.get_parent_build_step(env.sched,agent)
        #     if node_id(build_step) in env.active_build_steps
        #         setvisible!(factory_vis.active_flags[node_id(agent)],true)
        #     else
        #         setvisible!(factory_vis.active_flags[node_id(agent)],false)
        #     end
        # end
        # update_visualizer!(env.scene_tree,vis_nodes,agents)
        update_visualizer!(factory_vis,agents)
        render(vis)
    end
    step_animation!(anim)
    return nothing
end

# function construct_visualizer_update_function(vis,vis_nodes,staging_nodes;
function construct_visualizer_update_function(factory_vis;
        anim=nothing,
        render_stages=true,
    )
    _update_func(env,newly_updated=Set{Int}()) = visualizer_update_function!(
        factory_vis,
        env,
        newly_updated;
        anim=anim,
        render_stages
        )
    return _update_func
    # @unpack vis, vis_nodes, staging_nodes = factory_vis
    # if vis === nothing
    #     f(env,s=Set{Int}()) = step_animation!(anim)
    #     return f
    # end
    # update_visualizer_function(env,newly_updated=Set{Int}()) = begin
    #     atframe(anim,current_frame(anim)) do
    #         agents = Set{SceneNode}()
    #         for id in get_vtx_ids(ConstructionBots.rvo_global_id_map())
    #             agent = get_node(env.scene_tree, id)
    #             push!(agents, agent)
    #             for vp in collect_descendants(env.scene_tree,agent)
    #                 push!(agents,get_node(env.scene_tree,vp))
    #             end
    #         end
    #         if render_stages
    #             closed_steps = setdiff(keys(staging_nodes),env.active_build_steps)
    #             for id in closed_steps
    #                 setvisible!(staging_nodes[id],false)
    #             end
    #             for id in env.active_build_steps
    #                 setvisible!(staging_nodes[id],true)
    #             end
    #         end
    #         for id in env.active_build_steps
    #             setvisible!(staging_nodes[id],true)
    #         end
    #         for v in union(env.cache.active_set,newly_updated)
    #             node = get_node(env.sched,v)
    #             if matches_template(EntityGo,node)
    #                 agent = entity(node)
    #             elseif matches_template(Union{FormTransportUnit,DepositCargo},node)
    #                 agent = get_node(env.scene_tree,cargo_id(entity(node)))
    #             else
    #                 agent = nothing
    #             end
    #             if !(agent === nothing) && !(agent in agents)
    #                 push!(agents,agent)
    #                 for vp in collect_descendants(env.scene_tree,agent)
    #                     push!(agents,get_node(env.scene_tree,vp))
    #                 end
    #             end
    #         end
    #         # update_visualizer!(env.scene_tree,vis_nodes,agents)
    #         update_visualizer!(factory_vis,agents)
    #         render(vis)
    #     end
    #     step_animation!(anim)
    # end
    # # if !(anim === nothing)
    # #     anim_function(args...) = begin
    # #         atframe(anim.anim, current_frame(anim)) do
    # #             update_visualizer_function(args)
    # #         end
    # #     end
    # # end
    # return update_visualizer_function
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

function render_model_spec_with_pictures(model_spec;
        base_image_path="",
        bg_color="white",
        stroke_color="black",
        use_original_coords=true,
        picture_scale=2.0,
        scale=1,
        aspect_stretch=(1.0,1.0),
        label_pos=(0.0,0.0),
        label_fontsize=14pt,
        label_radius=1cm,
        label_bg_color=nothing,
        label_stroke_color=nothing,
        labels = Dict(),
        kwargs...
    )
    plt_spec = GraphUtils.contract_by_predicate(model_spec,n->matches_template(BuildingStep,n))
    plt = display_graph(plt_spec,scale=1) #,enforce_visited=true)
    # match pictures to assembly names
    counts = Dict()
    file_names = Dict()
    for n in node_iterator(plt_spec,topological_sort_by_dfs(plt_spec))
        parent_name = node_val(n).parent
        count = get!(counts,parent_name,1)
        name_string = split(node_val(n).parent,".")[1]
        filename = joinpath(base_image_path,string(name_string,@sprintf("%02i",count),".png"))
        file_names[node_id(n)] = filename
        counts[parent_name] = counts[parent_name] + 1
    end
    if use_original_coords
        coords = GraphPlottingBFS.get_layout_coords(model_spec)
        x = [coords[1][get_vtx(model_spec,id)] for id in get_vtx_ids(plt_spec)]
        y = [coords[2][get_vtx(model_spec,id)] for id in get_vtx_ids(plt_spec)]
    else
        x,y = GraphPlottingBFS.get_layout_coords(plt_spec)
    end

    _color_func = (v,c)->haskey(labels,get_vtx_id(plt_spec,v)) ? c : nothing
    
    plt = display_graph(plt_spec,(x,y),
        draw_node_function=(G,v)->Compose.compose(
            Compose.context(),
            (Compose.context(),
                (context(),
                    Compose.text(
                        label_pos...,
                        get(labels,get_vtx_id(plt_spec,v),""),
                        hcenter,
                        vcenter,
                        ),
                        Compose.fontsize(label_fontsize),
                    ),
                (context(),
                    Compose.circle(label_pos...,label_radius),
                        Compose.fill(_color_func(v,label_bg_color)),
                        Compose.stroke(_color_func(v,label_stroke_color)),
                        ),
                ),
            (Compose.context(),bitmap("image/png",read(file_names[get_vtx_id(G,v)]),
                -(picture_scale-1.0)/2,
                -(picture_scale-1.0)/2,
                picture_scale,
                picture_scale,
                )),
            (Compose.context(),circle(),fill(bg_color),Compose.stroke(stroke_color)),
            ),
        scale=scale,
        aspect_stretch=aspect_stretch,
    )
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
GraphPlottingBFS._title_string(::ConstructionBots.AssemblyStart)     = "sA"
GraphPlottingBFS._title_string(::ConstructionBots.AssemblyComplete)  = "cA"
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
    :(GraphPlottingBFS._subtitle_string),
    :(GraphPlottingBFS._subtitle_text_scale),
    :(GraphPlottingBFS._title_text_scale)
    )
    @eval $op(n::CustomNode,args...) = $op(node_val(n),args...)
    @eval $op(n::ScheduleNode,args...) = $op(n.node,args...)
end

GraphPlottingBFS._subtitle_text_scale(n::Union{ConstructionPredicate,SceneNode}) = 0.28
GraphPlottingBFS._title_text_scale(n::Union{ConstructionPredicate,SceneNode}) = 0.28
# GraphPlottingBFS._node_shape(n::CustomNode,args...) = GraphPlottingBFS._node_shape(node_val(n),args...)
# GraphPlottingBFS._node_color(n::CustomNode,args...) = GraphPlottingBFS._node_color(node_val(n),args...)
# GraphPlottingBFS._title_string(n::CustomNode,args...) = GraphPlottingBFS._title_string(node_val(n),args...)

GraphPlottingBFS._subtitle_string(n::SceneNode) = "$(get_id(node_id(n)))"
GraphPlottingBFS._subtitle_string(n::Union{EntityGo,EntityConfigPredicate,FormTransportUnit,DepositCargo}) = GraphPlottingBFS._subtitle_string(entity(n))
GraphPlottingBFS._subtitle_string(n::BuildPhasePredicate) = GraphPlottingBFS._subtitle_string(n.assembly)
GraphPlottingBFS._subtitle_string(n::ObjectNode) = "o$(get_id(node_id(n)))"
GraphPlottingBFS._subtitle_string(n::RobotNode) = "r$(get_id(node_id(n)))"
GraphPlottingBFS._subtitle_string(n::AssemblyNode) = "a$(get_id(node_id(n)))"
GraphPlottingBFS._subtitle_string(n::TransportUnitNode) = cargo_type(n) == AssemblyNode ? "a$(get_id(node_id(n)))" : "o$(get_id(node_id(n)))"


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