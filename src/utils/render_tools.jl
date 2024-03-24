struct AnimationWrapper
    anim::MeshCat.Animation
    step::Counter
end

AnimationWrapper(step::Int) = AnimationWrapper(MeshCat.Animation(), Counter(step))
current_frame(a::AnimationWrapper) = get_counter_status(a.step)
current_frame(::Nothing) = 0
step_animation!(::Nothing, step = 1) = nothing
step_animation!(a::AnimationWrapper, step = 1) =
    set_counter_status!(a.step, current_frame(a) + step)
set_current_frame!(a::AnimationWrapper, val) = set_counter_status!(a.step, val)
MeshCat.atframe(f, anim::AnimationWrapper, frame::Integer) = atframe(f, anim.anim, frame)
MeshCat.atframe(f, anim::Nothing, frame::Integer) = f()

"""
    construct_color_map(model_spec,id_map)

Given the color code in the `SubFileRef`s of model_spec, create a dict mapping
node ids to colors. Should only be applicable to models (not assemblies).
"""
function construct_color_map(model_spec, id_map)
    color_map = Dict{Union{String,AbstractID},AlphaColor}()
    ldraw_color_dict = LDrawParser.get_color_dict()
    for n in get_nodes(model_spec)
        if isa(node_val(n), SubFileRef)
            color_key = node_val(n).color
            if haskey(id_map, node_id(n))
                id = id_map[node_id(n)]
                if haskey(ldraw_color_dict, color_key) && isa(id, ObjectID)
                    color_map[id] = ldraw_color_dict[color_key]
                    color_map[id_map[id]] = color_map[id]
                end
            end
        end
    end
    return color_map
end

@with_kw mutable struct FactoryVisualizer
    vis = nothing
    scene_tree = SceneTree()
    vis_nodes = Dict{AbstractID,Any}() # Dict from AbstractID -> vis[string(id)]
    geom_nodes = Dict{GeometryKey,Dict{AbstractID,Any}}() # Dict containing the overapproximation layers (HypersphereKey,etc.)
    active_flags = Dict{AbstractID,Any}() # contains vis nodes for flagging agents as active or not
    staging_nodes = Dict{AbstractID,Any}()
end

"""
    populate_visualizer!(scene_tree,vis,id_map)

Populate the geometry (and transforms) of a MeshCat visualizer.
"""
function populate_visualizer!(
    scene_tree,
    vis;
    color_map = Dict(),
    material_type = nothing,
    wireframe = false,
    kwargs...,
)
    vis_root = vis["root"]
    vis_nodes = Dict{AbstractID,Any}()
    base_geom_nodes = Dict{AbstractID,Any}()
    for v in topological_sort_by_dfs(scene_tree)
        node = get_node(scene_tree, v)
        id = node_id(node)
        vis_nodes[id] = vis_root[string(id)]
        vis_node = vis_nodes[id]
        geom_vis_node = vis_node["base_geom"]
        base_geom_nodes[id] = geom_vis_node
        # Geometry
        geom = get_base_geom(node)
        if !(geom === nothing)
            if isa(geom, GeometryPrimitive)
                M = geom
            else
                filtered_geom = filter(m -> !isa(m, GeometryBasics.Line), geom)
                M = GeometryBasics.Mesh(coordinates(filtered_geom), faces(filtered_geom))
            end
            if !(material_type === nothing)
                mat = material_type(; wireframe = wireframe, kwargs...)
                mat.color = get(color_map, id, mat.color)
                setobject!(geom_vis_node, M, mat)
            else
                setobject!(geom_vis_node, M)
            end
        end
        settransform!(vis_node, global_transform(node))
    end
    factory_vis =
        FactoryVisualizer(vis = vis, scene_tree = scene_tree, vis_nodes = vis_nodes)
    factory_vis.geom_nodes[BaseGeomKey()] = base_geom_nodes
    factory_vis
end

function add_indicator_nodes!(
    factory_vis;
    cylinder_depth = 0.0025,
    cylinder_radius_pad = 0.02,
    active_color = RGB(0.0, 1.0, 0.0),
    kwargs...,
)
    @unpack vis, vis_nodes, scene_tree = factory_vis
    active_flags = Dict{AbstractID,Any}()
    for n in get_nodes(scene_tree)
        if matches_template(Union{TransportUnitNode,RobotNode}, n)
            vis_node = vis_nodes[node_id(n)]
            geom = get_base_geom(n, HypersphereKey())
            mod_center_point = Vector(get_center(geom))
            mod_center_point[3] = 0.0
            cylinder = GeometryBasics.Cylinder(
                Point(mod_center_point...),
                Point((mod_center_point .- [0.0, 0.0, cylinder_depth])...),
                get_radius(geom) + cylinder_radius_pad,
            )
            setobject!(
                vis_node["active"],
                cylinder,
                MeshLambertMaterial(color = active_color, kwargs...),
            )
            active_flags[node_id(n)] = vis_node["active"]
        end
    end
    factory_vis.active_flags = active_flags
    factory_vis
end

convert_to_renderable(geom) = geom
convert_to_renderable(geom::LazySets.Ball2) = GeometryBasics.Sphere(geom)
convert_to_renderable(geom::Hyperrectangle) = GeometryBasics.HyperRectangle(geom)

function show_geometry_layer!(
    factory_vis,
    key = HypersphereKey();
    color = RGBA{Float32}(0, 1, 0, 0.3),
    wireframe = true,
    material = MeshLambertMaterial(color = color, wireframe = wireframe),
)
    @unpack vis_nodes, scene_tree = factory_vis
    geom_nodes = Dict{AbstractID,Any}()
    for (id, vis_node) in vis_nodes
        node = get_node(scene_tree, id)
        geom = get_base_geom(node, key)
        node_name = string(key)
        setobject!(vis_node[node_name], convert_to_renderable(geom), material)
        geom_nodes[id] = vis_node[node_name]
    end
    factory_vis.geom_nodes[key] = geom_nodes
    factory_vis
end

"""
    rotate_camera!(vis,anim;

Rotate camera at fixed angular rate while keeping focus on a given point
"""
function rotate_camera!(
    vis,
    anim;
    rc = 10,
    dθ = 0.005,
    hc = 2,
    θ_start = 0,
    k_start = 0,
    k_final = current_frame(anim),
    radial_decay_factor = 0.0,
    origin = [0.0, 0.0, 0.0],
)
    θ = θ_start
    for k = k_start:k_final
        rc -= k * radial_decay_factor # spiral
        # Y-coord is up for camera
        camera_tform = CoordinateTransformations.Translation(rc * cos(θ), hc, -rc * sin(θ))
        θ = wrap_to_pi(θ + dθ)
        atframe(anim, k) do
            setprop!(
                vis["/Cameras/default/rotated/<object>"],
                "position",
                camera_tform.translation,
            )
            settransform!(
                vis["/Cameras/default"],
                CoordinateTransformations.Translation(origin...),
            )
        end
    end
    anim
end

function render_staging_areas!(
    vis,
    scene_tree,
    sched,
    staging_circles,
    root_key = "staging_circles";
    color = RGBA{Float32}(1, 0, 1, 0.1),
    wireframe = false,
    material = MeshLambertMaterial(color = color, wireframe = wireframe),
    include_build_steps = true,
    dz = 0.0,
)
    staging_nodes = Dict{AbstractID,Any}()
    staging_vis = vis[root_key]
    for (id, geom) in staging_circles
        node = get_node(scene_tree, id)
        sphere = LazySets.Ball2([geom.center..., 0.0], geom.radius)
        ctr = Point(project_to_2d(sphere.center)..., 0.0)
        cylinder = Cylinder(ctr, Point((ctr .- [0.0, 0.0, 0.01])...), sphere.radius)
        setobject!(staging_vis[string(id)], cylinder, material)
        staging_nodes[id] = staging_vis[string(id)]
        cargo = get_node(scene_tree, id)
        if isa(cargo, AssemblyNode)
            cargo_node = get_node(sched, AssemblyComplete(cargo))
        else
            cargo_node = get_node!(sched, ObjectStart(cargo, TransformNode()))
        end
        t = project_to_3d(
            project_to_2d(global_transform(start_config(cargo_node)).translation),
        )
        tform = CoordinateTransformations.Translation(t) ∘ identity_linear_map()
        settransform!(staging_nodes[id], tform)
        # settransform!(staging_nodes[id],global_transform(start_config(cargo_node)))
    end
    for n in node_iterator(sched, topological_sort_by_dfs(sched))
        if matches_template(OpenBuildStep, n)
            id = node_id(n)
            sphere = get_cached_geom(node_val(n).staging_circle)
            ctr = Point(project_to_2d(sphere.center)..., -0.02)
            cylinder = Cylinder(ctr, Point((ctr .- [0.0, 0.0, 0.01])...), sphere.radius)
            setobject!(staging_vis[string(id)], cylinder, material)
            staging_nodes[id] = staging_vis[string(id)]
        end
    end
    staging_nodes
end

"""
    visualize_staging_plan(vis,sched,scene_tree)

Render arrows pointing from start->goal for LiftIntoPlace nodes.
"""
function visualize_staging_plan(vis, sched, scene_tree; objects = false)
    vis_arrows = vis["arrows"]
    vis_triads = vis["triads"]
    bounding_vis = vis["bounding_circles"]
    delete!(vis_arrows)
    delete!(vis_triads)
    delete!(bounding_vis)
    for n in get_nodes(sched)
        if matches_template(LiftIntoPlace, n)
            # LiftIntoPlace path
            p1 = Point(global_transform(start_config(n)).translation...)
            p2 = Point(global_transform(goal_config(n)).translation...)
            lift_vis = ArrowVisualizer(vis_arrows[string(node_id(n))])
            setobject!(lift_vis, MeshPhongMaterial(color = RGBA{Float32}(0, 1, 0, 1.0)))
            settransform!(lift_vis, p1, p2)
            # Transport path
            cargo = entity(n)
            if isa(cargo, AssemblyNode)
                c = get_node(sched, AssemblyComplete(cargo))
                color = RGBA{Float32}(1, 0, 0, 1.0)
            elseif objects == true
                # continue
                c = get_node(sched, ObjectStart(cargo))
                color = RGBA{Float32}(0, 0, 1, 1.0)
            else
                continue
            end
            p3 = Point(global_transform(start_config(c)).translation...)
            transport_vis = ArrowVisualizer(vis_arrows[string(node_id(c))])
            setobject!(transport_vis, MeshPhongMaterial(color = color))
            settransform!(transport_vis, p3, p1)
        elseif matches_template(AssemblyComplete, n)
            triad_vis = vis_triads[string(node_id(n))]
            setobject!(triad_vis, Triad(0.25))
            settransform!(triad_vis, global_transform(goal_config(n)))
        end
    end
    for (id, geom) in bounding_circles
        if isa(id, AssemblyID)
            node = get_node(scene_tree, id)
        else
            node = get_node(scene_tree, node_val(get_node(sched, id)).assembly)
        end
        sphere = LazySets.Ball2([geom.center..., 0.0], geom.radius)
        b_vis = bounding_vis[string(id)]
        setobject!(
            b_vis,
            convert_to_renderable(sphere),
            MeshPhongMaterial(color = RGBA{Float32}(1, 0, 1, 0.1)),
        )
        cargo_node = get_node(sched, AssemblyComplete(node))
        settransform!(b_vis, global_transform(start_config(cargo_node)))
    end
    return vis_triads, vis_arrows, bounding_vis
end

function animate_reverse_staging_plan!(
    vis,
    vis_nodes,
    scene_tree,
    sched,
    nodes = get_nodes(scene_tree);
    v_max = 1.0,
    ω_max = 1.0,
    ϵ_v = 1e-4,
    ϵ_ω = 1e-4,
    dt = 0.1,
    interp = false,
    interp_steps = 10,
    objects = false,
    anim = nothing,
)
    graph = deepcopy(get_graph(scene_tree))
    # Initialize goal sequences for each ObjectNode and AssemblyNode
    goals = Dict{AbstractID,Vector{CoordinateTransformations.AffineMap}}()
    goal_idxs = Dict{AbstractID,Int}()
    interp_idxs = Dict{AbstractID,Int}()
    for node in nodes
        if matches_template(AssemblyNode, node)
            start_node = get_node(sched, AssemblyStart(node))
        elseif matches_template(ObjectNode, node)
            start_node = get_node(sched, ObjectStart(node))
        else
            continue
        end
        if !has_vertex(sched, node_id(LiftIntoPlace(node)))
            continue
        end
        lift_node = get_node(sched, LiftIntoPlace(node))
        goal_list = goals[node_id(node)] = Vector{CoordinateTransformations.AffineMap}()
        goal_idxs[node_id(node)] = 1
        interp_idxs[node_id(node)] = interp_steps
        push!(goal_list, global_transform(start_config(start_node)))
    end
    # Animate
    active_set = get_all_root_nodes(graph)
    closed_set = Set{Int}()
    _close_node(graph, v, active_set, closed_set) = begin
        push!(closed_set, v)
        union!(active_set, outneighbors(graph, v))
    end
    while !isempty(active_set)
        setdiff!(active_set, closed_set)
        for v in collect(active_set)
            node = get_node(scene_tree, v)
            if !haskey(goals, node_id(node))
                _close_node(graph, v, active_set, closed_set)
                continue
            end
            goal_idx = goal_idxs[node_id(node)]
            if goal_idx > length(goals[node_id(node)])
                _close_node(graph, v, active_set, closed_set)
                continue
            end
            goal = goals[node_id(node)][goal_idx]
            tf_error = relative_transform(global_transform(node), goal)
            twist = optimal_twist(tf_error, v_max, ω_max, dt)
            if norm(twist.vel) <= ϵ_v && norm(twist.ω) <= ϵ_ω ||
               interp_idxs[node_id(node)] == 0
                goal_idxs[node_id(node)] += 1
                interp_idxs[node_id(node)] = interp_steps
            end
            if interp
                isteps = interp_idxs[node_id(node)]
                @assert isteps > 0
                tform = interpolate_transforms(global_transform(node), goal, 1.0 / isteps)
                interp_idxs[node_id(node)] -= 1
            else
                tform = global_transform(node) ∘ integrate_twist(twist, dt)
            end
            set_desired_global_transform!(get_transform_node(node), tform)
        end
        to_update = collect_descendants(graph, active_set, true)
        animate_update_visualizer!(
            scene_tree,
            vis_nodes,
            [get_node(scene_tree, v) for v in to_update];
            anim = anim,
        )
        render(vis)
    end
end

function render_transport_unit_2d(
    scene_tree,
    tu,
    cvx_hull = [];
    scale = 4cm,
    hull_thickness = 0.01,
    line_thickness = 0.0025,
    hull_color = RGB(0.0, 1.0, 0.0),
    config_pt_radius = 2 * hull_thickness,
    config_pt_color = RGB(1.0, 0.5, 0.2),
    robot_color = RGB(0.1, 0.5, 0.9),
    geom_fill_color = RGBA(0.9, 0.9, 0.9, 0.3),
    geom_stroke_color = RGBA(0.0, 0.0, 0.0, 1.0),
    robot_radius = default_robot_radius(),
    rect2d = project_to_2d(get_base_geom(tu, HyperrectangleKey())),
    xpad = (0, 0),
    ypad = (0, 0),
    bg_color = nothing,
    overlay_for_single_robot = false,
)
    if length(robot_team(tu)) == 1 && overlay_for_single_robot
        robot_overlay = (
            context(),
            [
                Compose.circle(project_to_2d(tform.translation)..., ROBOT_RADIUS) for
                (id, tform) in robot_team(tu)
            ]...,
            Compose.fill(RGBA(robot_color, 0.8)),
        )
    else
        robot_overlay = (context(),)
    end
    set_default_graphic_size(rect2d.radius[1] * scale, rect2d.radius[2] * scale)
    Compose.compose(
        context(units = unit_box_from_rect(rect2d, xpad = xpad, ypad = ypad)),
        (
            context(),
            robot_overlay,
            plot_base_geom_2d(
                get_node(scene_tree, cargo_id(tu)),
                scene_tree;
                fill_color = geom_fill_color,
                stroke_color = geom_stroke_color,
            ),
            Compose.linewidth(line_thickness * max(h, w)),
            (
                context(),
                Compose.polygon([(p[1], p[2]) for p in cvx_hull]),
                Compose.stroke(hull_color),
                Compose.linewidth(hull_thickness * max(h, w)),
                Compose.fill(nothing),
            ),
            (
                context(),
                [
                    Compose.circle(
                        project_to_2d(tform.translation)...,
                        config_pt_radius * max(h, w),
                    ) for (id, tform) in robot_team(tu)
                ]...,
                Compose.fill(config_pt_color),
            ),
        ),
        (
            context(),
            [
                Compose.circle(project_to_2d(tform.translation)..., robot_radius) for
                (id, tform) in robot_team(tu)
            ]...,
            Compose.fill(robot_color),
        ),
        (context(), Compose.rectangle(), Compose.fill(bg_color)),
    )
end

function plot_base_geom_2d(
    node,
    scene_tree;
    fill_color = RGBA(1.0, 0.0, 0.0, 0.5),
    stroke_color = RGBA(0.0, 0.0, 0.0, 1.0),
    _show_lines = false,
    _show_triangles = false,
    _show_quadrilaterals = true,
    _use_unit_box = false,
    tform = identity_linear_map(),
    geom_key = BaseGeomKey(),
)
    polygon_pts = []
    for geom_element in recurse_child_geometry(node, scene_tree, geom_key)
        if isa(geom_key, BaseGeomKey)
            for geom in geom_element
                if isa(geom, GeometryBasics.Line) && _show_lines
                    push!(polygon_pts, Vector([(p[1], p[2]) for p in tform(geom).points]))
                elseif isa(geom, Triangle) && _show_triangles
                    push!(polygon_pts, Vector([(p[1], p[2]) for p in tform(geom).points]))
                elseif isa(geom, GeometryBasics.Quadrilateral) && _show_quadrilaterals
                    push!(polygon_pts, Vector([(p[1], p[2]) for p in tform(geom).points]))
                end
            end
        elseif isa(geom_key, HyperrectangleKey)
            r = tform(geom_element)
            push!(
                polygon_pts,
                [
                    (r.center[1] - r.radius[1], r.center[2] - r.radius[2]),
                    (r.center[1] - r.radius[1], r.center[2] + r.radius[2]),
                    (r.center[1] + r.radius[1], r.center[2] + r.radius[2]),
                    (r.center[1] + r.radius[1], r.center[2] - r.radius[2]),
                ],
            )
        end
    end
    if _use_unit_box
        r = get_base_geom(node, HyperrectangleKey())
        unit_box = unit_box_from_rect(r)
        ctx = context(units = unit_box)
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

function unit_box_from_rect(r; pad = 0.0, xpad = (pad, pad), ypad = (pad, pad))
    unit_box = UnitBox(
        r.center[1] - r.radius[1] - xpad[1],
        r.center[2] - r.radius[2] - ypad[1],
        2 * r.radius[1] + sum(xpad),
        2 * r.radius[2] + sum(ypad),
    )
end

"""
    animate_preprocessing_steps!

Step through the different phases of preprocessing
"""
function animate_preprocessing_steps!(
    factory_vis,
    sched;
    anim = nothing,
    dt = 0.0,
    interp_steps = 80,
    kwargs...,
)
    @unpack vis, vis_nodes, scene_tree, geom_nodes = factory_vis
    base_geom_nodes = geom_nodes[BaseGeomKey()]
    rect_nodes = geom_nodes[HyperrectangleKey()]

    atframe(anim, current_frame(anim)) do
        # Hide robots
        for n in get_nodes(scene_tree)
            if isa(n, Union{TransportUnitNode,RobotNode})
                setvisible!(vis_nodes[node_id(n)], false)
            end
        end
        setvisible!(base_geom_nodes, true)
        setvisible!(rect_nodes, false)
        jump_to_final_configuration!(scene_tree; set_edges = true)
        update_visualizer!(scene_tree, vis_nodes)
    end
    step_animation!(anim)
    # First, show the components of the final assembly turning into the hyperectangles
    for (ii, n) in enumerate(get_nodes(scene_tree))
        if isa(n, Union{AssemblyNode,ObjectNode})
            atframe(anim, current_frame(anim)) do
                setvisible!(rect_nodes[node_id(n)], true)
                setvisible!(base_geom_nodes[node_id(n)], false)
                update_visualizer!(scene_tree, vis_nodes, [n])
            end
            if mod(ii, 2) == 0
                step_animation!(anim)
            end
        end
    end
    # Show objects moving to the staging/building locations
    animate_reverse_staging_plan!(
        vis,
        vis_nodes,
        scene_tree,
        sched,
        filter(n -> isa(n, AssemblyNode), get_nodes(scene_tree));
        anim = anim,
        interp = true,
        dt = dt,
        interp_steps = interp_steps,
        kwargs...,
    )
    # Go from hyperectangles back to the full geometries (legos)
    for n in get_nodes(scene_tree)
        if isa(n, ObjectNode)
            atframe(anim, current_frame(anim)) do
                setvisible!(base_geom_nodes[node_id(n)], true)
                setvisible!(rect_nodes[node_id(n)], false)
                update_visualizer!(scene_tree, vis_nodes, [n])
            end
        end
    end
    set_current_frame!(anim, current_frame(anim) + 10)
    # Show the legos going to the starting locations
    animate_reverse_staging_plan!(
        vis,
        vis_nodes,
        scene_tree,
        sched,
        filter(n -> isa(n, ObjectNode), get_nodes(scene_tree));
        dt = dt,
        interp = true,
        interp_steps = interp_steps,
        anim = anim,
        kwargs...,
    )
    atframe(anim, current_frame(anim)) do
        setvisible!(rect_nodes, false)
        setvisible!(base_geom_nodes, true)
    end
    step_animation!(anim)
    # Make robots visible again
    for n in get_nodes(scene_tree)
        if isa(n, Union{TransportUnitNode,RobotNode})
            atframe(anim, current_frame(anim)) do
                setvisible!(vis_nodes[node_id(n)], true)
                update_visualizer!(scene_tree, vis_nodes, [n])
            end
        end
    end
    step_animation!(anim)
    atframe(anim, current_frame(anim)) do
        set_scene_tree_to_initial_condition!(scene_tree, sched; remove_all_edges = true)
        update_visualizer!(scene_tree, vis_nodes)
    end
    set_current_frame!(anim, current_frame(anim) + 10)
    vis
end

function MeshCat.setvisible!(vis_nodes::Dict{AbstractID,Any}, val)
    for (k, v) in vis_nodes
        setvisible!(v, val)
    end
end

"""
    update_visualizer!(scene_tree, vis_nodes)

Update the MeshCat transform tree.
"""
function update_visualizer!(vis_nodes, nodes)
    for n in nodes
        settransform!(vis_nodes[node_id(n)], global_transform(n))
    end
    return vis_nodes
end

function update_visualizer!(scene_tree::SceneTree, vis_nodes, nodes)
    for n in nodes
        settransform!(vis_nodes[node_id(n)], global_transform(n))
    end
    return vis_nodes
end

function update_visualizer!(scene_tree::SceneTree, vis_nodes; nodes = get_nodes(scene_tree))
    update_visualizer!(scene_tree, vis_nodes, nodes)
end

function update_visualizer!(factory_vis::FactoryVisualizer, args...)
    update_visualizer!(factory_vis.scene_tree, factory_vis.vis_nodes, args...)
end

function animate_update_visualizer!(args...; anim = nothing, step = 1)
    if anim === nothing
        return update_visualizer!(args...)
    else
        atframe(anim.anim, current_frame(anim)) do
            return update_visualizer!(args...)
        end
        step_animation!(anim, step)
    end
end

function visualizer_update_function!(factory_vis, env, newly_updated = Set{Int}();)
    @unpack vis, vis_nodes, staging_nodes = factory_vis

    closed_steps_nodes = []
    active_build_nodes = []
    fac_active_flags_nodes = []

    scene_nodes = Set{SceneNode}()
    for id in get_vtx_ids(ConstructionBots.rvo_global_id_map())
        agent = get_node(env.scene_tree, id)
        push!(scene_nodes, agent)
        for vp in collect_descendants(env.scene_tree, agent)
            push!(scene_nodes, get_node(env.scene_tree, vp))
        end
    end
    closed_steps = setdiff(keys(staging_nodes), env.active_build_steps)
    for id in closed_steps
        push!(closed_steps_nodes, staging_nodes[id])
    end
    for id in env.active_build_steps
        push!(active_build_nodes, staging_nodes[id])
    end
    for v in union(env.cache.active_set, newly_updated)
        node = get_node(env.sched, v)
        if matches_template(EntityGo, node)
            agent = entity(node)
            object = agent
        elseif matches_template(Union{FormTransportUnit,DepositCargo}, node)
            agent = entity(node)
            object = get_node(env.scene_tree, cargo_id(entity(node)))
        else
            agent = nothing
            object = nothing
        end
        if matches_template(Union{RobotNode,TransportUnitNode}, agent)
            if ConstructionBots.parent_build_step_is_active(node, env)
                if ConstructionBots.cargo_ready_for_pickup(node, env)
                    push!(fac_active_flags_nodes, node_id(agent))
                end
            end
        end
        if !(object === nothing) && !(object in scene_nodes)
            push!(scene_nodes, object)
            for vp in collect_descendants(env.scene_tree, object)
                push!(scene_nodes, get_node(env.scene_tree, vp))
            end
        end
    end
    return scene_nodes, closed_steps_nodes, active_build_nodes, fac_active_flags_nodes
end

function call_update!(scene_tree, vis_nodes, nodes, dt)
    update_visualizer!(scene_tree, vis_nodes, nodes)
    render(vis)
end

function render_model_spec_with_pictures(
    model_spec;
    base_image_path = "",
    bg_color = "white",
    stroke_color = "black",
    use_original_coords = true,
    picture_scale = 2.0,
    scale = 1,
    aspect_stretch = (1.0, 1.0),
    label_pos = (0.0, 0.0),
    label_fontsize = 14pt,
    label_radius = 1cm,
    label_bg_color = nothing,
    label_stroke_color = nothing,
    labels = Dict(),
    kwargs...,
)
    plt_spec = contract_by_predicate(model_spec, n -> matches_template(BuildingStep, n))
    plt = display_graph(plt_spec, scale = 1)  #,enforce_visited=true)
    # Match pictures to assembly names
    counts = Dict()
    file_names = Dict()
    for n in node_iterator(plt_spec, topological_sort_by_dfs(plt_spec))
        parent_name = node_val(n).parent
        count = get!(counts, parent_name, 1)
        name_string = split(node_val(n).parent, ".")[1]
        filename =
            joinpath(base_image_path, string(name_string, @sprintf("%02i", count), ".png"))
        file_names[node_id(n)] = filename
        counts[parent_name] = counts[parent_name] + 1
    end
    if use_original_coords
        coords = get_layout_coords(model_spec)
        x = [coords[1][get_vtx(model_spec, id)] for id in get_vtx_ids(plt_spec)]
        y = [coords[2][get_vtx(model_spec, id)] for id in get_vtx_ids(plt_spec)]
    else
        x, y = get_layout_coords(plt_spec)
    end
    _color_func = (v, c) -> haskey(labels, get_vtx_id(plt_spec, v)) ? c : nothing
    plt = display_graph(
        plt_spec,
        (x, y),
        draw_node_function = (G, v) -> Compose.compose(
            Compose.context(),
            (
                Compose.context(),
                (
                    context(),
                    Compose.text(
                        label_pos...,
                        get(labels, get_vtx_id(plt_spec, v), ""),
                        hcenter,
                        vcenter,
                    ),
                    Compose.fontsize(label_fontsize),
                ),
                (
                    context(),
                    Compose.circle(label_pos..., label_radius),
                    Compose.fill(_color_func(v, label_bg_color)),
                    Compose.stroke(_color_func(v, label_stroke_color)),
                ),
            ),
            (
                Compose.context(),
                bitmap(
                    "image/png",
                    read(file_names[get_vtx_id(G, v)]),
                    -(picture_scale - 1.0) / 2,
                    -(picture_scale - 1.0) / 2,
                    picture_scale,
                    picture_scale,
                ),
            ),
            (Compose.context(), circle(), fill(bg_color), Compose.stroke(stroke_color)),
        ),
        scale = scale,
        aspect_stretch = aspect_stretch,
    )
end

get_id(s::String) = s
# Render schedule nodes
_title_string(n::S) where {S<:SceneNode} = split(string(typeof(n)), ".")[end][1]
_title_string(n::RobotNode) = "R"
_title_string(n::ObjectNode) = "O"
_title_string(n::AssemblyNode) = "A"
_title_string(n::TransportUnitNode) = "T"
_title_string(::BuildingStep) = "B"
_title_string(::SubFileRef) = "S"
_title_string(::SubModelPlan) = "M"
_title_string(n::ConstructionBots.EntityConfigPredicate) = _title_string(n.entity)
_title_string(::ConstructionBots.RobotStart) = "R"
_title_string(n::ConstructionBots.ObjectStart) = "O"
_title_string(::ConstructionBots.AssemblyStart) = "sA"
_title_string(::ConstructionBots.AssemblyComplete) = "cA"
_title_string(::ConstructionBots.OpenBuildStep) = "oB"
_title_string(::ConstructionBots.CloseBuildStep) = "cB"
_title_string(::ConstructionBots.RobotGo) = "G"
_title_string(::ConstructionBots.FormTransportUnit) = "F"
_title_string(::ConstructionBots.DepositCargo) = "D"
_title_string(::ConstructionBots.TransportUnitGo) = "T"
_title_string(::ConstructionBots.LiftIntoPlace) = "L"
_title_string(::ConstructionBots.ProjectComplete) = "P"
for op in (
    :(_node_shape),
    :(_node_color),
    :(_title_string),
    :(_subtitle_string),
    :(_subtitle_text_scale),
    :(_title_text_scale),
)
    @eval $op(n::CustomNode, args...) = $op(node_val(n), args...)
    @eval $op(n::ScheduleNode, args...) = $op(n.node, args...)
end

_subtitle_text_scale(n::Union{ConstructionPredicate,SceneNode}) = 0.28
_title_text_scale(n::Union{ConstructionPredicate,SceneNode}) = 0.28
_subtitle_string(n::SceneNode) = "$(get_id(node_id(n)))"
_subtitle_string(n::Union{EntityGo,EntityConfigPredicate,FormTransportUnit,DepositCargo}) =
    _subtitle_string(entity(n))
_subtitle_string(n::BuildPhasePredicate) = _subtitle_string(n.assembly)
_subtitle_string(n::ObjectNode) = "o$(get_id(node_id(n)))"
_subtitle_string(n::RobotNode) = "r$(get_id(node_id(n)))"
_subtitle_string(n::AssemblyNode) = "a$(get_id(node_id(n)))"
_subtitle_string(n::TransportUnitNode) =
    cargo_type(n) == AssemblyNode ? "a$(get_id(node_id(n)))" : "o$(get_id(node_id(n)))"

SPACE_GRAY = RGB(0.2, 0.2, 0.2)
BRIGHT_RED = RGB(0.6, 0.0, 0.2)
LIGHT_BROWN = RGB(0.6, 0.3, 0.2)
LIME_GREEN = RGB(0.2, 0.6, 0.2)

BRIGHT_BLUE = RGB(0.0, 0.4, 1.0)
_node_color(::RobotNode) = SPACE_GRAY
_node_color(::ObjectNode) = SPACE_GRAY
_node_color(::AssemblyNode) = BRIGHT_BLUE
_node_color(::TransportUnitNode) = LIME_GREEN
_node_color(::BuildingStep) = LIGHT_BROWN
_node_color(::SubFileRef) = BRIGHT_RED
_node_color(::SubModelPlan) = SPACE_GRAY
_node_color(::ConstructionBots.EntityConfigPredicate) = _node_color(n.entity)
_node_color(::ConstructionBots.RobotStart) = SPACE_GRAY
_node_color(::ConstructionBots.ObjectStart) = SPACE_GRAY
_node_color(::ConstructionBots.AssemblyStart) = SPACE_GRAY
_node_color(::ConstructionBots.AssemblyComplete) = SPACE_GRAY
_node_color(::ConstructionBots.OpenBuildStep) = LIGHT_BROWN
_node_color(::ConstructionBots.CloseBuildStep) = LIGHT_BROWN
_node_color(::ConstructionBots.ProjectComplete) = SPACE_GRAY
_node_color(::ConstructionBots.RobotGo) = LIME_GREEN
_node_color(::ConstructionBots.FormTransportUnit) = LIME_GREEN
_node_color(::ConstructionBots.TransportUnitGo) = LIME_GREEN
_node_color(::ConstructionBots.DepositCargo) = LIME_GREEN
_node_color(::ConstructionBots.LiftIntoPlace) = BRIGHT_BLUE

function draw_node(g::AbstractCustomNGraph, v, args...; kwargs...)
    draw_node(get_node(g, v), args...; kwargs...)
end

function plot_staging_area(
    sched,
    scene_tree,
    staging_circles;
    save_file_name = "staging_area.pdf",
    save_image = true,
    nominal_width = 30cm,
    node_list = node_iterator(sched, topological_sort_by_dfs(sched)),
    show_intermediate_stages = false,
)
    _final_stage_stroke_color = "red"
    _final_stage_bg_color = nothing

    _stage_stroke_color = "yellow"
    _stage_bg_color = RGBA(1.0, 1.0, 0.0, 0.5)

    _bounding_stroke_color = "blue"
    _bounding_bg_color = nothing

    _dropoff_stroke_color = "green"
    _dropoff_bg_color = nothing

    base_geom_layer = Compose.compose(
        context(),
        Compose.linewidth(0.02pt),
        plot_assemblies(
            sched,
            scene_tree,
            fill_color = RGBA(0.5, 0.5, 0.5, 0.9),
            stroke_color = nothing,
        ),
    )
    bg_color = nothing
    staging_circs = []
    final_staging_circs = []
    bounding_circs = []
    dropoff_circs = []
    for n in node_list
        if matches_template(AssemblyStart, n)
        elseif matches_template(CloseBuildStep, n)
            push!(staging_circs, node_id(n) => get_cached_geom(node_val(n).staging_circle))
            push!(
                bounding_circs,
                node_id(n) => get_cached_geom(node_val(n).bounding_circle),
            )
            is_terminal_build_step = true
            for v in outneighbors(sched, n)
                if matches_template(OpenBuildStep, get_node(sched, v))
                    is_terminal_build_step = false
                end
            end
            is_terminal_build_step ? nothing : continue
            push!(
                final_staging_circs,
                node_id(n) => get_cached_geom(node_val(n).staging_circle),
            )
        elseif matches_template(DepositCargo, n)
            tu = entity(n)
            a = get_node(scene_tree, cargo_id(tu))
            tf = global_transform(goal_config(n))
            push!(dropoff_circs, node_id(n) => tf(get_base_geom(tu, HypersphereKey())))
        end
    end
    bbox = staging_plan_bbox(staging_circs)
    bg_ctx = (context(), Compose.rectangle(), Compose.fill(bg_color))
    Compose.set_default_graphic_size(
        nominal_width,
        (bbox.widths[2] / bbox.widths[1]) * nominal_width,
    )
    circles_ctx = (
        context(),
        (
            context(),
            [
                Compose.circle(c.center[1], c.center[2], c.radius) for
                (k, c) in final_staging_circs
            ]...,
            Compose.stroke(_final_stage_stroke_color),
            Compose.fill(_final_stage_bg_color),
        ),
        (
            context(),
            [
                Compose.circle(c.center[1], c.center[2], c.radius) for
                (k, c) in staging_circs if show_intermediate_stages
            ]...,
            Compose.stroke(_stage_stroke_color),
            Compose.fill(_stage_bg_color),
        ),
        (
            context(),
            [
                Compose.circle(c.center[1], c.center[2], c.radius) for
                (k, c) in dropoff_circs
            ]...,
            Compose.stroke(_dropoff_stroke_color),
            Compose.fill(_dropoff_bg_color),
        ),
        (
            context(),
            [
                Compose.circle(c.center[1], c.center[2], c.radius) for
                (k, c) in bounding_circs
            ]...,
            Compose.stroke(_bounding_stroke_color),
            Compose.fill(_bounding_bg_color),
        ),
    )
    plt = Compose.compose(
        context(units = UnitBox(bbox.origin..., bbox.widths...)),
        base_geom_layer,
        circles_ctx,
        bg_ctx,
    )
    circs = transform_final_assembly_staging_circles(sched, scene_tree, staging_circles)
    forms = [Compose.circle(v.center..., v.radius) for v in values(circs)]
    stage_plt = Compose.compose(
        context(),
        plt,
        (
            context(units = UnitBox(bbox.origin..., bbox.widths...)),
            forms...,
            Compose.fill(nothing),
            Compose.stroke(RGBA(0.5, 0.5, 0.5, 1.0)),
        ),
        (context(), Compose.rectangle(), Compose.fill(RGB(0.25, 0.25, 0.25))),
    )
    if save_image
        draw(PDF(save_file_name), stage_plt)
    end
    display(stage_plt)
end

function staging_plan_bbox(staging_circs)
    xlo = minimum(c.center[1] - c.radius for (k, c) in staging_circs)
    ylo = minimum(c.center[2] - c.radius for (k, c) in staging_circs)
    xhi = maximum(c.center[1] + c.radius for (k, c) in staging_circs)
    yhi = maximum(c.center[2] + c.radius for (k, c) in staging_circs)
    GeometryBasics.Rect2D(xlo, ylo, xhi - xlo, yhi - ylo)
end

staging_plan_bbox(sched::AbstractGraph) =
    staging_plan_bbox(extract_build_step_staging_circles(sched))

function extract_build_step_staging_circles(sched)
    circs = Dict()
    for n in get_nodes(sched)
        if matches_template(CloseBuildStep, n)
            circs[node_id(n)] = get_cached_geom(node_val(n).staging_circle)
        end
    end
    circs
end

function transform_final_assembly_staging_circles(sched, scene_tree, staging_circles)
    circs = Dict()
    for (k, v) in staging_circles
        tform = global_transform(
            goal_config(get_node(sched, AssemblyComplete(get_node(scene_tree, k)))),
        )
        # circ = LazySets.translate(v,tform.translation[1:2])
        circ = LazySets.Ball2(v.center + tform.translation[1:2], v.radius)
        circs[k] = circ
    end
    circs
end

function plot_assemblies(sched, scene_tree; base_ctx = context(), kwargs...)
    ctxs = []
    for n in get_nodes(sched)
        if matches_template(AssemblyComplete, n)
            tform = global_transform(goal_config(n))
            push!(ctxs, plot_base_geom_2d(entity(n), scene_tree; tform = tform, kwargs...))
        end
    end
    Compose.compose(base_ctx, ctxs...)
end
