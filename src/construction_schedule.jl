export
    start_config,
    goal_config,
    cargo_start_config,
    cargo_goal_config,
    cargo_deployed_config,
    cargo_loaded_config,
    entity,
    ConstructionPredicate,
    EntityConfigPredicate,
    RobotStart,
    ObjectStart,
    AssemblyComplete,
    AssemblyStart,
    EntityGo,
    RobotGo,
    TransportUnitGo,
    LiftIntoPlace,
    FormTransportUnit,
    DepositCargo,
    BuildPhasePredicate,
    OpenBuildStep,
    CloseBuildStep,
    ProjectComplete


"""
    abstract type ConstructionPredicate

Abstract type for ConstructionSchedule nodes.
"""
abstract type ConstructionPredicate end
start_config(n) = n.start_config
goal_config(n) = n.goal_config
entity(n) = n.entity
add_node!(g::AbstractCustomNGraph, n::ConstructionPredicate) = add_node!(g, n, node_id(n))
get_vtx(g::AbstractCustomNGraph, n::ConstructionPredicate) = get_vtx(g, node_id(n))

for op in (:start_config, :goal_config, :entity,
    :cargo_start_config, :cargo_goal_config,
    :cargo_deployed_config, :cargo_loaded_config)
    @eval begin
        $op(n::CustomNode) = $op(node_val(n))
        $op(n::ScheduleNode) = $op(n.node)
    end
end

abstract type EntityConfigPredicate{E} <: ConstructionPredicate end
start_config(n::EntityConfigPredicate) = n.config
goal_config(n::EntityConfigPredicate) = n.config

struct RobotStart <: EntityConfigPredicate{RobotNode}
    entity::RobotNode
    config::TransformNode
end
struct ObjectStart <: EntityConfigPredicate{ObjectNode}
    entity::ObjectNode
    config::TransformNode
end
struct AssemblyComplete <: EntityConfigPredicate{AssemblyNode}
    entity::AssemblyNode
    config::TransformNode
    inner_staging_circle::GeomNode # inner staging circle
    outer_staging_circle::GeomNode # outer staging circle
end
function AssemblyComplete(n::SceneNode, t::TransformNode)
    inner_staging_circle = GeomNode(LazySets.Ball2(zeros(SVector{3,Float64}), 0.0))
    outer_staging_circle = GeomNode(LazySets.Ball2(zeros(SVector{3,Float64}), 0.0))
    node = AssemblyComplete(n, t, inner_staging_circle, outer_staging_circle)
    set_parent!(inner_staging_circle, t)
    set_parent!(outer_staging_circle, t)
    node
end
struct AssemblyStart <: EntityConfigPredicate{AssemblyNode}
    entity::AssemblyNode
    config::TransformNode
end
for T in (:RobotStart, :ObjectStart, :AssemblyComplete, :AssemblyStart)
    @eval begin
        $T(n::SceneNode) = $T(n, TransformNode())
        $T(n::ConstructionPredicate) = $T(entity(n), goal_config(n)) # shared config
    end
end

"""
    abstract type EntityGo{E}

Encodes going from state start_config(n) to state goal_config(n).
"""
abstract type EntityGo{E} <: ConstructionPredicate end

struct RobotGo{R} <: EntityGo{RobotNode}
    entity::R
    start_config::TransformNode
    goal_config::TransformNode
    id::ActionID # required because there may be multiple RobotGo nodes for one robot
end

struct TransportUnitGo <: EntityGo{TransportUnitNode}
    entity::TransportUnitNode
    start_config::TransformNode
    goal_config::TransformNode
end

struct LiftIntoPlace{C} <: EntityGo{C}
    entity::C # AssemblyNode or ObjectNode
    start_config::TransformNode
    # path plan?
    goal_config::TransformNode
end

"""
    abstract type BuildPhasePredicate <: ConstructionPredicate

References a building phase. Interface:
- `get_assembly(::BuildPhasePredicate)` => ::AssemblyNode - the assembly to be
modified by this build phase
- `assembly_components(::BuildPhasePredicate)` => TransformDict{Union{ObjectID,AssemblyID}}
- the components to be added to the assembly during this phase
"""
abstract type BuildPhasePredicate <: ConstructionPredicate end

get_assembly(n::BuildPhasePredicate) = n.assembly
assembly_components(n::BuildPhasePredicate) = n.components
assembly_components(n::ConstructionPredicate) = assembly_components(entity(n))
get_assembly(n::CustomNode) = get_assembly(node_val(n))
assembly_components(n::CustomNode) = assembly_components(node_val(n))

"""
    extract_building_phase(n::BuildingStep,tree::SceneTree,model_spec,id_map)

extract the assembly and set of subcomponents from a BuildingStep.
"""
function extract_building_phase(n, tree::SceneTree, model_spec, id_map)
    @assert matches_template(BuildingStep, n)
    assembly_id = id_map[node_val(n).parent]
    @assert isa(assembly_id, AssemblyID)
    assembly = get_node(tree, assembly_id)
    parts = typeof(assembly_components(assembly))()
    for child_id in get_build_step_components(model_spec, id_map, n)
        @assert has_component(assembly, child_id) "$child_id is not a child of assembly $assembly"
        parts[child_id] = child_transform(assembly, child_id)
    end
    # for vp in inneighbors(model_spec,n)
    #     child_id = get_referenced_component(model_spec,id_map,get_vtx_id(model_spec,vp))
    #     # child_id = get(id_map,get_vtx_id(model_spec,vp),nothing)
    #     if !(child_id === nothing)
    #         @assert has_component(assembly, child_id) "$child_id is not a child of assembly $assembly"
    #         parts[child_id] = child_transform(assembly,child_id)
    #     end
    # end
    return assembly, parts
end

# struct BuildPhase
#     components::TransformDict{Union{AssemblyID,ObjectID}}
#     staging_circle::GeomNode # staging circle - surrounds the staging area--requires tf relative to assembly
#     bounding_circle::GeomNode # bounding circle - surrounds the current assembly--requires tf relative to assembly
# end

"""
    OpenBuildStep <: ConstructionPredicate

Marks the beginning of a new building phase.
Fields:
- assembly::AssemblyNode - the assembly to be modified by this build phase
- components::Dict{Union{AssemblyID,ObjectID}} - the components to be added to
the assembly during this phase
"""
struct OpenBuildStep <: BuildPhasePredicate
    assembly::AssemblyNode
    components::TransformDict{Union{AssemblyID,ObjectID}}
    staging_circle::GeomNode # staging circle - surrounds the staging area--requires tf relative to assembly
    bounding_circle::GeomNode # bounding circle - surrounds the current assembly--requires tf relative to assembly
    id::Int # id of this build step
end
struct CloseBuildStep <: BuildPhasePredicate
    assembly::AssemblyNode
    components::TransformDict{Union{AssemblyID,ObjectID}}
    staging_circle::GeomNode
    bounding_circle::GeomNode
    id::Int # id of this build step
end
for T in (:OpenBuildStep, :CloseBuildStep)
    @eval begin
        $T(a::AssemblyNode, c::Dict, g1::GeomNode, g2::GeomNode) = $T(a, c, g1, g2, get_id(get_unique_id(TemplatedID{$T})))
        $T(a::AssemblyNode, c::Dict) = $T(a, c,
            GeomNode(LazySets.Ball2(zeros(SVector{3,Float64}), 0.0)),
            GeomNode(LazySets.Ball2(zeros(SVector{3,Float64}), 0.0))
        )
        $T(n::CustomNode, args...) = $T(extract_building_phase(n, args...)...)
        $T(n::BuildPhasePredicate) = $T(n.assembly, n.components,
            n.staging_circle, n.bounding_circle, n.id)
    end
end
node_id(n::T) where {T<:BuildPhasePredicate} = TemplatedID{T}(n.id)
# add_node!(g::AbstractCustomNGraph, n::P) where {P<:BuildPhasePredicate} = add_node!(g,n,get_unique_id(TemplatedID{P}))

"""
    FormTransportUnit <: ConstructionPredicate

TransportUnit must remain motionless as the cargo moves from its start
configuration to its `FormTransportUnit.cargo_goal_config` configuration.
"""
struct FormTransportUnit <: ConstructionPredicate
    entity::TransportUnitNode
    config::TransformNode
    cargo_start_config::TransformNode
    cargo_goal_config::TransformNode # offset from config by child_transform(entity,cargo_id(entity))
end

"""
    DepositCargo <: ConstructionPredicate

TransportUnit must remain motionless as the cargo moves from its transport
configuration to its `LiftIntoPlace.start_config` configuration.
"""
struct DepositCargo <: ConstructionPredicate
    entity::TransportUnitNode
    config::TransformNode
    cargo_start_config::TransformNode # offset from config by child_transform(entity,cargo_id(entity))
    cargo_goal_config::TransformNode
end
start_config(n::Union{DepositCargo,FormTransportUnit}) = n.config
goal_config(n::Union{DepositCargo,FormTransportUnit}) = n.config
cargo_start_config(n::Union{DepositCargo,FormTransportUnit}) = n.cargo_start_config
cargo_goal_config(n::Union{DepositCargo,FormTransportUnit}) = n.cargo_goal_config
cargo_loaded_config(n::DepositCargo) = cargo_start_config(n)
cargo_loaded_config(n::FormTransportUnit) = cargo_goal_config(n)
cargo_deployed_config(n::DepositCargo) = cargo_goal_config(n)
cargo_deployed_config(n::FormTransportUnit) = cargo_start_config(n)

for T in (:DepositCargo, :FormTransportUnit)
    @eval begin
        function $T(n::TransportUnitNode)
            t = $T(n, TransformNode(), TransformNode(), TransformNode())
            # cargo_goal -> cargo_start -> entity_config
            set_parent!(cargo_loaded_config(t), cargo_deployed_config(t))
            set_parent!(start_config(t), cargo_loaded_config(t))
            # set child transform cargo_start -> entity_config
            set_local_transform!(start_config(t), inv(child_transform(n, cargo_id(n))))
            return t
        end
    end
end

for T in (:RobotGo, :TransportUnitGo) #,:LiftIntoPlace)
    @eval begin
        function $T(n::SceneNode)
            t = $T(n, TransformNode(), TransformNode())
            set_parent!(goal_config(t), start_config(t))
            return t
        end
    end
end
RobotGo(n::RobotNode, start, goal) = RobotGo(n, start, goal, get_unique_id(ActionID))
function LiftIntoPlace(n::Union{ObjectNode,AssemblyNode})
    l = LiftIntoPlace(n, TransformNode(), TransformNode())
    set_parent!(start_config(l), goal_config(l)) # point to parent
    return l
end

robot_team(n::ConstructionPredicate) = robot_team(entity(n))
cargo_node_type(n::TransportUnitNode) = cargo_type(n) == AssemblyNode ? AssemblyComplete : ObjectStart
cargo_node_type(n::ConstructionPredicate) = cargo_node_type(entity(n))

struct ProjectComplete <: ConstructionPredicate
    project_id::Int
    config::TransformNode
end
ProjectComplete(id) = ProjectComplete(id, TransformNode())
ProjectComplete() = ProjectComplete(get_id(get_unique_id(TemplatedID{ProjectComplete})))
node_id(n::ProjectComplete) = TemplatedID{ProjectComplete}(n.project_id)

required_predecessors(::ConstructionPredicate) = Dict()
required_successors(::ConstructionPredicate) = Dict()
eligible_predecessors(n::ConstructionPredicate) = required_predecessors(n)
eligible_successors(n::ConstructionPredicate) = required_successors(n)

required_predecessors(::RobotStart) = Dict()
required_successors(::RobotStart) = Dict(RobotGo => 1)

required_predecessors(::RobotGo) = Dict(Union{RobotStart,DepositCargo,RobotGo} => 1)
required_successors(::RobotGo) = Dict()
eligible_successors(::RobotGo) = Dict(Union{RobotGo,FormTransportUnit} => 1)

required_predecessors(n::FormTransportUnit) = Dict(RobotGo => length(robot_team(n)), cargo_node_type(n) => 1)
required_successors(::FormTransportUnit) = Dict(TransportUnitGo => 1)

required_predecessors(::TransportUnitGo) = Dict(FormTransportUnit => 1)
required_successors(::TransportUnitGo) = Dict(DepositCargo => 1)

required_predecessors(::DepositCargo) = Dict(TransportUnitGo => 1, OpenBuildStep => 1)
required_successors(n::DepositCargo) = Dict(LiftIntoPlace => 1, RobotGo => length(robot_team(n)))

required_predecessors(::LiftIntoPlace) = Dict(DepositCargo => 1)
required_successors(::LiftIntoPlace) = Dict(CloseBuildStep => 1)

required_predecessors(n::CloseBuildStep) = Dict(LiftIntoPlace => num_components(n))
required_successors(::CloseBuildStep) = Dict(Union{OpenBuildStep,AssemblyComplete} => 1)

required_predecessors(::OpenBuildStep) = Dict(Union{AssemblyStart,CloseBuildStep} => 1)
required_successors(n::OpenBuildStep) = Dict(DepositCargo => num_components(n))

required_predecessors(::ObjectStart) = Dict()
required_successors(::ObjectStart) = Dict(FormTransportUnit => 1)

# required_predecessors(  n::AssemblyComplete) = Dict(Union{ObjectStart,CloseBuildStep}=>1)
required_predecessors(::AssemblyComplete) = Dict(CloseBuildStep => 1)
required_successors(::AssemblyComplete) = Dict(Union{FormTransportUnit,ProjectComplete} => 1)

required_predecessors(::AssemblyStart) = Dict()
required_successors(::AssemblyStart) = Dict(OpenBuildStep => 1)

required_predecessors(::ProjectComplete) = Dict(AssemblyComplete => 1)
required_successors(::ProjectComplete) = Dict()


for T in (
    :ObjectStart,
    :RobotStart,
    # :RobotGo,
    :AssemblyStart,
    :AssemblyComplete,
    :FormTransportUnit,
    :TransportUnitGo,
    :DepositCargo,
    :LiftIntoPlace,
)
    @eval begin
        function node_id(n::$T)
            TemplatedID{Tuple{$T,typeof(node_id(entity(n)))}}(get_id(node_id(entity(n))))
        end
    end
end
node_id(n::RobotGo) = n.id

# for T in (
#     :ObjectStart,
#     :RobotStart,
#     :RobotGo,
#     :AssemblyStart,
#     :AssemblyComplete,
# )
#     @eval begin
#         node_id(n::$T) = TemplatedID{$T}(get_id(node_id(entity(n))))
#     end
# end

"""
    get_previous_build_step(model_spec,v)

Return the node representing the previous build step in the model spec.
"""
function get_previous_build_step(model_spec, v;
    skip_first=matches_template(SubModelPlan, get_node(model_spec, v)),
)
    vp = depth_first_search(model_spec, get_vtx(model_spec, v),
        u -> matches_template(BuildingStep, get_node(model_spec, u)),
        u -> (get_vtx(model_spec, v) == u || !matches_template(SubModelPlan, get_node(model_spec, u))),
        inneighbors;
        skip_first=skip_first,
    )
    if has_vertex(model_spec, vp)
        return get_node(model_spec, vp)
    end
    return nothing
end

export construct_partial_construction_schedule

"""
    populate_schedule_build_step!(sched,cb,step_node,model_spec,scene_tree,id_map)

Creates an OpenBuildStep ... CloseBuildStep structure and adds an edge
CloseBuildStep => parent.
Args:
- sched - the schedule
- cb::CustomNode{CloseBuildStep,AbstractID}
- model_spec
- scene_tree
- id_map
"""
function populate_schedule_build_step!(sched, parent::AssemblyComplete, cb, step_node, model_spec, scene_tree, id_map;
    connect_to_sub_assemblies=true,
)
    # OpenBuildStep
    ob = add_node!(sched, OpenBuildStep(node_val(cb)))
    for child_id in get_build_step_components(model_spec, id_map, step_node)
        cargo = get_node(scene_tree, child_id)
        transport_unit = get_node(scene_tree, node_id(TransportUnitNode(cargo)))
        @assert isa(cargo, Union{AssemblyNode,ObjectNode})
        # LiftIntoPlace
        l = add_node!(sched, LiftIntoPlace(cargo))
        set_parent!(goal_config(l), start_config(parent))
        set_local_transform!(goal_config(l), child_transform(entity(parent), node_id(cargo)))
        # ensure that LiftIntoPlace starts in the carry configuration
        set_desired_global_transform!(start_config(l),
            CoordinateTransformations.Translation(global_transform(start_config(l)).translation)
        )
        @info "Staging config: setting GOAL config of $(node_id(l)) to $(goal_config(l))"
        add_edge!(sched, l, cb) # LiftIntoPlace => CloseBuildStep
        # DepositCargo
        d = add_node!(sched, DepositCargo(transport_unit))
        set_parent!(cargo_deployed_config(d), start_config(l))
        add_edge!(sched, d, l) # DepositCargo => LiftIntoPlace
        add_edge!(sched, ob, d) # OpenBuildStep => DepositCargo
        # TransportUnitGo
        tgo = add_node!(sched, TransportUnitGo(transport_unit))
        set_parent!(goal_config(tgo), start_config(d))
        add_edge!(sched, tgo, d) # TransportUnitGo => DepositCargo
        # FormTransportUnit
        f = add_node!(sched, FormTransportUnit(transport_unit))
        set_parent!(start_config(tgo), goal_config(f))
        add_edge!(sched, f, tgo) # FormTransportUnit => TransportUnitGo
        # ObjectStart/AssemblyComplete
        if isa(cargo, AssemblyNode) # && connect_to_sub_assemblies
            cargo_node = get_node(sched, AssemblyComplete(cargo))
        else
            cargo_node = add_node!(sched, ObjectStart(cargo, TransformNode()))
        end
        if connect_to_sub_assemblies
            set_parent!(goal_config(node_val(cargo_node)), start_config(parent))
        end
        add_edge!(sched, cargo_node, f) # ObjectStart/AssemblyComplete => FormTransportUnit
        set_parent!(cargo_deployed_config(f), start_config(cargo_node))
    end
    return ob
end

"""
    populate_schedule_sub_graph!(
        sched,
        parent::AssemblyComplete,
        model_spec,
        scene_tree,
        id_map)

Add all building steps to parent, working backward from parents
"""
function populate_schedule_sub_graph!(sched, parent::AssemblyComplete, model_spec, scene_tree, id_map)
    parent_assembly = entity(parent)
    # sa = add_node!(sched,AssemblyStart(parent_assembly))
    sa = add_node!(sched, AssemblyStart(parent)) # shares config with AssemblyComplete (AssemblyComplete.config === AssemblyStart.config)
    spec_node = get_node(model_spec, id_map[node_id(parent_assembly)])
    step_node = get_previous_build_step(model_spec, spec_node; skip_first=true)
    immediate_parent = parent
    while !(step_node === nothing)
        # CloseBuildStep
        cb = add_node!(sched, CloseBuildStep(step_node, scene_tree, model_spec, id_map))
        add_edge!(sched, cb, immediate_parent) # CloseBuildStep => AssemblyComplete / OpenBuildStep
        set_parent!(node_val(cb).staging_circle, start_config(parent))
        set_parent!(node_val(cb).bounding_circle, start_config(parent))
        ob = populate_schedule_build_step!(sched, parent, cb, step_node, model_spec, scene_tree, id_map)
        immediate_parent = ob
        step_node = get_previous_build_step(model_spec, step_node; skip_first=true)
    end
    add_edge!(sched, sa, immediate_parent) # AssemblyStart => OpenBuildStep
    sched
end

"""
    construct_partial_construction_schedule(sched,args...)

Constructs a schedule that does not yet reflect the "staging plan" and does not
yet contain any RobotStart or RobotGo nodes.
"""
function construct_partial_construction_schedule(
    mpd_model,
    model_spec,
    scene_tree,
    id_map=build_id_map(mpd_model, model_spec)
)
    sched = NGraph{DiGraph,ConstructionPredicate,AbstractID}()
    parent_map = backup_descendants(model_spec, n -> matches_template(SubModelPlan, n))
    # Add assemblies first
    for node in node_iterator(model_spec, topological_sort_by_dfs(model_spec))
        if matches_template(SubModelPlan, node)
            assembly = get_node(scene_tree, id_map[node_id(node)])
            # AssemblyComplete
            a = add_node!(sched, AssemblyComplete(assembly, TransformNode())) ###############
            if is_root_node(scene_tree, assembly)
                # ProjectComplete
                p = add_node!(sched, ProjectComplete())
                add_edge!(sched, a, p)
            end
            # add build steps
            populate_schedule_sub_graph!(sched, node_val(a), model_spec, scene_tree, id_map)
        end
    end
    # Add robots
    set_robot_start_configs!(sched, scene_tree)
    return sched
end

export validate_schedule_transform_tree

function assert_transform_tree_ancestor(a, b)
    @assert has_ancestor(a, b) "a should have ancestor b, for a = $(a), b = $(b)"
end

function transformations_approx_equiv(t1, t2)
    a = all(isapprox.(t1.translation, t2.translation))
    b = all(isapprox.(t1.linear, t2.linear))
    a && b
end

"""
    validate_schedule_transform_tree(sched)

Checks if sched and its embedded transform tree are valid.
- The graph itself should be valid
- All chains AssemblyComplete ... LiftIntoPlace -> DepositCargo should be
    connected in the embedded transform tree

"""
function validate_schedule_transform_tree(sched; post_staging=false)
    try
        @assert validate_graph(sched)
        for n in get_nodes(sched)
            if matches_template(AssemblyComplete, n)
                @assert validate_tree(goal_config(n)) "Subtree invalid for $(n)"
            end
        end
        for n in get_nodes(sched)
            if matches_template(OpenBuildStep, n)
                open_build_step = node_val(n)
                assembly = open_build_step.assembly
                assembly_complete = get_node(sched, AssemblyComplete(assembly))
                for v in outneighbors(sched, n)
                    deposit_node = get_node(sched, v)
                    @assert matches_template(DepositCargo, deposit_node)
                    assert_transform_tree_ancestor(goal_config(deposit_node), start_config(assembly_complete))
                    for vp in outneighbors(sched, v)
                        lift_node = get_node(sched, vp)
                        if matches_template(LiftIntoPlace, lift_node)
                            @assert has_child(
                                goal_config(assembly_complete), goal_config(lift_node))
                            if post_staging
                                # Show that goal_config(LiftIntoPlace) matches the
                                # goal config of cargo relative to assembly
                                @assert transformations_approx_equiv(
                                    local_transform(goal_config(lift_node)),
                                    child_transform(assembly, node_id(entity(lift_node)))
                                )
                                # Show that start_config(LiftIntoPlace) matches
                                # cargo_deployed_config(DepositCargo)
                                @assert transformations_approx_equiv(
                                    global_transform(start_config(lift_node)),
                                    global_transform(cargo_deployed_config(deposit_node)),
                                )
                            end
                        end
                    end
                end
            elseif matches_template(Union{DepositCargo,FormTransportUnit}, n)
                transport_unit = entity(n)
                @assert has_child(cargo_loaded_config(n), start_config(n))
                @assert transformations_approx_equiv(
                    local_transform(start_config(n)),
                    inv(child_transform(transport_unit, cargo_id(transport_unit)))
                )
                if post_staging
                    @assert isapprox(global_transform(goal_config(n)).translation[3], 0.0; rtol=1e-6, atol=1e-6) "$n, $(global_transform(goal_config(n)).translation)"
                end
            elseif matches_template(ObjectStart, n)
            elseif matches_template(LiftIntoPlace, n)
                @assert has_child(goal_config(n), start_config(n))
                if post_staging
                    l = n
                    cargo = entity(n)
                    f = get_node(sched, FormTransportUnit(TransportUnitNode(cargo)))
                    tu = entity(f)
                    o = cargo
                    d = get_node(sched, DepositCargo(tu))
                    l = get_node(sched, LiftIntoPlace(o))
                    @assert isapprox(global_transform(start_config(f)).translation[3], 0.0; rtol=1e-6, atol=1e-6) "Z translation should be 0 for $(summary(node_id(f)))"
                    @assert isapprox(global_transform(start_config(d)).translation[3], 0.0; rtol=1e-6, atol=1e-6) "Z translation should be 0 for $(summary(node_id(d)))"
                    transformations_approx_equiv(
                        global_transform(start_config(n)),
                        global_transform(cargo_deployed_config(f))
                    )
                    transformations_approx_equiv(
                        global_transform(cargo_loaded_config(f)),
                        global_transform(goal_config(f)) ∘ child_transform(tu, node_id(o))
                    )
                    transformations_approx_equiv(
                        global_transform(cargo_loaded_config(d)),
                        global_transform(goal_config(d)) ∘ child_transform(tu, node_id(o))
                    )
                    transformations_approx_equiv(
                        global_transform(cargo_deployed_config(d)),
                        global_transform(start_config(l))
                    )
                end
            end
        end
    catch e
        if isa(e, AssertionError)
            bt = catch_backtrace()
            showerror(stderr, e, bt)
            return false
        else
            rethrow(e)
        end
    end
    return true
end


"""
    generate_staging_plan(scene_tree,params)

Given a `SceneTree` representing the final configuration of an assembly,
construct a plan for the start config, staging config, and final config of
each subassembly and individual object. The idea is to ensure that no node of
the "plan" graph overlaps with any other. Requires circular bounding geometry
for each component of the assembly.
    Start at terminal assembly
    work downward through building steps
    For each building step, arrange the incoming parts to balance these
    objectives:
    - minimize "LiftIntoPlace" distance
    - maximize distance between components to be placed.
    It may be necessary to not completely isolate build steps (i.e., two
    consecutive build steps may overlap in the arrival times of subcomponents).
"""
function generate_staging_plan!(scene_tree, sched;
    buffer_radius=0.0,
    build_step_buffer_radius=0.0,
)
    if !all(map(n -> has_vertex(n.geom_hierarchy, HypersphereKey()), get_nodes(scene_tree)))
        compute_approximate_geometries!(scene_tree,
            HypersphereKey(); ϵ=0.0)
    end
    # For each build step, determine the radius of the largest transform unit
    # that may be active before the build step closes. This radius will be used
    # to inform the size of the buffer zones between staging areas

    # store growing bounding circle of each assembly
    bounding_circles = Dict{AbstractID,LazySets.Ball2}()
    staging_circles = Dict{AbstractID,LazySets.Ball2}()
    for node in node_iterator(sched, topological_sort_by_dfs(sched))
        if matches_template(AssemblyStart, node)
        elseif matches_template(OpenBuildStep, node)
            # work updward through build steps
            # Set staging config of part as start_config(LiftIntoPlace(part))
            process_schedule_build_step!(
                node,
                sched,
                scene_tree,
                bounding_circles,
                staging_circles,
                ;
                build_step_buffer_radius=build_step_buffer_radius,
            )
        end
    end
    # Update assembly start points so that none of the staging regions overlap
    select_assembly_start_configs_layered!(sched, scene_tree, staging_circles;
        buffer_radius=buffer_radius,)
    # TODO store a TransformNode in ProjectComplete() (or in the schedule itself,
    # once there is a dedicated ConstructionSchedule type) so that an entire
    # schedule can be moved anywhere. All would-be root TransormNodes will have
    # this root node as their parent, regardless of the edge structure of the
    # schedule graph
    return staging_circles, bounding_circles
end


"""
    select_assembly_start_configs!(sched, scene_tree, staging_radii;
        buffer_radius=0.0)

Select the start configs (i.e., the build location) for each assembly. The
location is selected by minimizing distance to the assembly's staging location
while ensuring that no child's staging area overlaps with its parent's staging
area. Uses the same ring optimization approach as for selectig staging
locations.
"""
function select_assembly_start_configs_layered!(sched, scene_tree, staging_circles;
    buffer_radius=0.0,
)
    # Update assembly start points so that none of the staging regions overlap
    for start_node in node_iterator(sched, topological_sort_by_dfs(sched))
        if !matches_template(AssemblyComplete, start_node)
            continue
        end
        assembly_complete = node_val(start_node)
        node = entity(start_node)
        if matches_template(AssemblyNode, node)
            # Apply ring solver with child assemblies (not objects)
            assembly = node
            part_ids = sort(filter(k -> isa(k, AssemblyID),
                collect(keys(assembly_components(node)))))
            if isempty(part_ids)
                continue
            end

            orig_ball = staging_circles[node_id(assembly)]
            new_ball = nothing

            @info "Selecting staging location for $(summary(node_id(assembly)))"

            @assert haskey(staging_circles, node_id(assembly))


            steps_for_parts = find_step_numbers(start_node, part_ids, sched, scene_tree)
            steps_dict = Dict(zip(part_ids, steps_for_parts))
            @assert length(steps_for_parts) == length(part_ids)

            part_ids_left = part_ids

            while !isempty(part_ids_left)
                # We continue this process until we don't have any more parts to place. At each
                # iteration, we determine the maximum number of components that can fit around
                # the current staging radius. If there are parts that remain, we increase the
                # staging radius from the previous "layer" and keep going.

                staging_circle = staging_circles[node_id(assembly)]
                staging_radius = staging_circle.radius + buffer_radius

                radii_left = [staging_circles[id].radius + buffer_radius for id in part_ids_left]
                steps_left = [steps_dict[id] for id in part_ids_left]
                unique_step_lengths = sort!(unique(steps_left))

                cnt = 1
                can_fit_steps = 0
                while cnt <= length(unique_step_lengths)
                    current_step_consideration = unique_step_lengths[cnt]
                    bool_mask = steps_left .<= current_step_consideration
                    radii_k = radii_left[bool_mask]

                    # Determine if the staging radius can fit all components
                    half_widths = asin.(radii_k ./ (radii_k .+ staging_radius))
                    can_fit = sum(half_widths) * 2 <= 2.0 * Float64(π)

                    if can_fit
                        can_fit_steps = current_step_consideration
                        cnt += 1
                    else
                        break
                    end
                end

                part_ids_for_opt = []
                radii_for_opt = []
                if can_fit_steps == 0
                    # Current staging radius cannot fit all of the components of the first step
                    # Let's select the maximum number of components in the first step that can
                    # fit. Selecting from the smallest radii
                    current_step_consideration = minimum(unique_step_lengths)
                    bool_mask = steps_left .<= current_step_consideration
                    part_ids_at_or_before_step = part_ids_left[bool_mask]
                    radii = radii_left[bool_mask]
                    sorted_idxs = sortperm(radii)
                    sorted_radii = radii[sorted_idxs]
                    half_widths = asin.(sorted_radii ./ (sorted_radii .+ staging_radius))
                    last_idx = findlast(cumsum(half_widths) * 2 .<= 2.0 * Float64(π))

                    idxs_for_opt = sorted_idxs[1:last_idx] # With step mask

                    part_ids_for_opt = part_ids_at_or_before_step[idxs_for_opt]
                    radii_for_opt = radii[idxs_for_opt]

                elseif can_fit_steps == maximum(unique_step_lengths)
                    # Current staging radius can fit all of the assemblies
                    part_ids_for_opt = part_ids_left
                    radii_for_opt = radii_left

                else
                    # We can fit all components upto can_fit_step, but not all of the remaining components
                    # We will select the maximum number of components in the next step that can fit in
                    # addition to all of the components from the can_fit_step
                    bool_mask = steps_left .<= can_fit_steps
                    part_ids_for_opt = part_ids_left[bool_mask]
                    radii_for_opt = radii_left[bool_mask]

                    arc_dist_used = sum(asin.(radii_for_opt ./ (radii_for_opt .+ staging_radius)))

                    part_ids_left = setdiff(part_ids_left, part_ids_for_opt)
                    steps_left = [steps_dict[id] for id in part_ids_left]
                    radii_left = [staging_circles[id].radius + buffer_radius for id in part_ids_left]

                    bool_mask = steps_left .<= unique_step_lengths[cnt]

                    part_ids_at_or_before_step = part_ids_left[bool_mask]
                    radii = radii_left[bool_mask]
                    sorted_idxs = sortperm(radii)
                    sorted_radii = radii[sorted_idxs]
                    half_widths = asin.(sorted_radii ./ (sorted_radii .+ staging_radius))
                    last_idx = findlast((arc_dist_used .+ cumsum(half_widths)) * 2 .<= 2.0 * Float64(π))

                    last_idx = isnothing(last_idx) ? 0 : last_idx
                    idxs_for_opt = sorted_idxs[1:last_idx] # With step mask

                    append!(part_ids_for_opt, part_ids_at_or_before_step[idxs_for_opt])
                    append!(radii_for_opt, radii[idxs_for_opt])
                end

                part_ids_left = setdiff(part_ids_left, part_ids_for_opt)
                steps_left = [steps_dict[id] for id in part_ids_left]
                radii_left = [staging_circles[id].radius + buffer_radius for id in part_ids_left]

                parts_for_opt = (get_node(scene_tree, part_id) for part_id in part_ids_for_opt)
                radii_for_opt = [staging_circles[id].radius + buffer_radius for id in part_ids_for_opt]

                ring_radius = -1
                θ_star = nothing
                # repeat to ensure correct alignment
                while staging_radius - ring_radius > 1e-6
                    θ_des = Vector{Float64}()
                    ring_radius = staging_radius
                    for (part_id, part) in zip(part_ids_for_opt, parts_for_opt)
                        # retrieve staging config from LiftIntoPlace node
                        lift_node = get_node(sched, LiftIntoPlace(part))
                        # give coords of the dropoff point (we want `part` to be built as close as possible to here.)
                        # crucial: must be relative to the assembly origin
                        tr = relative_transform(
                            global_transform(goal_config(start_node)),
                            global_transform(start_config(lift_node))).translation
                        tform = CoordinateTransformations.Translation(tr...)
                        geom = staging_circles[part_id]
                        # the vector from the circle center to the goal location
                        d_ = project_to_2d(tform.translation) - staging_circle.center
                        # scale d_ to the appropriate radius, then shift tip vector from part center to geom.center
                        d = (d_ / norm(d_)) * ring_radius + geom.center
                        push!(θ_des, atan(d[2], d[1]))
                    end
                    # optimize placement and increase staging_radius if necessary
                    θ_star, staging_radius = solve_ring_placement_problem(θ_des, radii_for_opt, ring_radius)
                end

                # Compute staging config transforms (relative to parent assembly)
                tformed_geoms = Vector{LazySets.Ball2}()
                for (i, (θ, r, part_id, part)) in enumerate(zip(θ_star, radii_for_opt, part_ids_for_opt, parts_for_opt))
                    part_start_node = get_node(sched, AssemblyComplete(part))
                    geom = staging_circles[part_id]
                    R = staging_radius + r
                    # shift by vector from geom.center to part origin
                    t = CoordinateTransformations.Translation(
                        R * cos(θ) + staging_circle.center[1] - geom.center[1],
                        R * sin(θ) + staging_circle.center[2] - geom.center[2],
                        0.0)
                    # Get global orientation parent frame ʷRₚ
                    tform = t ∘ identity_linear_map() # AffineMap transform
                    # set transform of start node
                    set_local_transform_in_global_frame!(start_config(part_start_node), tform)
                    tform2D = CoordinateTransformations.Translation(t.translation[1:2]...)
                    push!(tformed_geoms, tform2D(geom))
                end
                old_ball = staging_circles[node_id(assembly)]
                new_ball = overapproximate(
                    vcat(tformed_geoms, staging_circles[node_id(assembly)]),
                    LazySets.Ball2{Float64,SVector{2,Float64}}
                )
                @assert new_ball.radius + 1e-5 > old_ball.radius + norm(new_ball.center .- old_ball.center) "$(summary(node_id(assembly))) old ball $(old_ball), new_ball $(new_ball)"
                staging_circles[node_id(assembly)] = new_ball
            end

            # add directly as staging circle of AssemblyComplete node
            assembly_complete.inner_staging_circle.base_geom = project_to_3d(orig_ball)
            assembly_complete.outer_staging_circle.base_geom = project_to_3d(new_ball)
            @info "Updating staging_circle for $(summary(node_id(assembly))) to $(staging_circles[node_id(assembly)])"
        end
    end
    sched
end


"""
    select_assembly_start_configs!(sched, scene_tree, staging_radii;
        buffer_radius=0.0)

Select the start configs (i.e., the build location) for each assembly. The
location is selected by minimizing distance to the assembly's staging location
while ensuring that no child's staging area overlaps with its parent's staging
area. Uses the same ring optimization approach as for selectig staging
locations.
"""
function select_assembly_start_configs!(sched, scene_tree, staging_circles;
    buffer_radius=0.0,
)
    # Update assembly start points so that none of the staging regions overlap
    for start_node in node_iterator(sched, topological_sort_by_dfs(sched))
        if !matches_template(AssemblyComplete, start_node)
            continue
        end
        assembly_complete = node_val(start_node)
        node = entity(start_node)
        if matches_template(AssemblyNode, node)
            # Apply ring solver with child assemblies (not objects)
            assembly = node
            part_ids = sort(filter(k -> isa(k, AssemblyID),
                collect(keys(assembly_components(node)))))
            if isempty(part_ids)
                continue
            end
            @info "Selecting staging location for $(summary(node_id(assembly)))"
            # start_node = get_node(sched,AssemblyComplete(assembly))
            # staging_radii
            @assert haskey(staging_circles, node_id(assembly))
            staging_circle = staging_circles[node_id(assembly)]
            staging_radius = staging_circle.radius + buffer_radius
            ring_radius = -1
            parts = (get_node(scene_tree, part_id) for part_id in part_ids)
            # radii = [staging_circles[id].radius for id in part_ids]
            radii = [staging_circles[id].radius + buffer_radius / 2 for id in part_ids]
            # θ_des = Vector{Float64}()
            θ_star = nothing
            # repeat to ensure correct alignment
            while staging_radius - ring_radius > 1e-6
                θ_des = Vector{Float64}()
                ring_radius = staging_radius
                for (part_id, part) in zip(part_ids, parts)
                    # retrieve staging config from LiftIntoPlace node
                    lift_node = get_node(sched, LiftIntoPlace(part))
                    # give coords of the dropoff point (we want `part` to be built as close as possible to here.)
                    # crucial: must be relative to the assembly origin
                    tr = relative_transform(
                        global_transform(goal_config(start_node)),
                        global_transform(start_config(lift_node))).translation
                    tform = CoordinateTransformations.Translation(tr...)
                    geom = staging_circles[part_id]
                    # the vector from the circle center to the goal location
                    d_ = project_to_2d(tform.translation) - staging_circle.center
                    # scale d_ to the appropriate radius, then shift tip vector from part center to geom.center
                    d = (d_ / norm(d_)) * ring_radius + geom.center
                    push!(θ_des, atan(d[2], d[1]))
                end
                # optimize placement and increase staging_radius if necessary
                θ_star, staging_radius = solve_ring_placement_problem(
                    θ_des, radii, ring_radius)
            end
            # Compute staging config transforms (relative to parent assembly)
            tformed_geoms = Vector{LazySets.Ball2}()
            for (i, (θ, r, part_id, part)) in enumerate(zip(θ_star, radii, part_ids, parts))
                part_start_node = get_node(sched, AssemblyComplete(part))
                geom = staging_circles[part_id]
                R = staging_radius + r
                # shift by vector from geom.center to part origin
                t = CoordinateTransformations.Translation(
                    R * cos(θ) + staging_circle.center[1] - geom.center[1],
                    R * sin(θ) + staging_circle.center[2] - geom.center[2],
                    0.0)
                # Get global orientation parent frame ʷRₚ
                tform = t ∘ identity_linear_map() # AffineMap transform
                # set transform of start node
                # @info "Starting config: setting START config of $(node_id(start_node)) to $(tform)"
                set_local_transform_in_global_frame!(start_config(part_start_node), tform)
                tform2D = CoordinateTransformations.Translation(t.translation[1:2]...)
                push!(tformed_geoms, tform2D(geom))
            end
            old_ball = staging_circles[node_id(assembly)]
            new_ball = overapproximate(
                vcat(tformed_geoms, staging_circles[node_id(assembly)]),
                LazySets.Ball2{Float64,SVector{2,Float64}}
            )
            # @assert new_ball.radius > old_ball.radius+norm(new_ball.center .- old_ball.center)
            @assert new_ball.radius + 1e-5 > old_ball.radius + norm(new_ball.center .- old_ball.center) "$(summary(node_id(assembly))) old ball $(old_ball), new_ball $(new_ball)"
            staging_circles[node_id(assembly)] = new_ball
            # add directly as staging circle of AssemblyComplete node
            assembly_complete.inner_staging_circle.base_geom = project_to_3d(old_ball)
            assembly_complete.outer_staging_circle.base_geom = project_to_3d(new_ball)
            @info "Updating staging_circle for $(summary(node_id(assembly))) to $(staging_circles[node_id(assembly)])"
        end
    end
    sched
end

"""
    process_schedule_build_step!(node,sched,scene_tree,bounding_circles,staging_circles)

Select the staging configuration for all subcomponents to be added to `assembly`
during `build_step`, where `build_step::OpenBuildStep = node_val(node)`, and
`assembly = build_step.assembly.`
Also updates `bounding_radii[node_id(assembly)]` to reflect the increasing size
of assembly as more parts are added to it.
Updates:
- `staging_circles`
- the relevant `LiftIntoPlace` nodes (start_config and goal_config transforms)
Keyword Args:
- build_step_buffer_radius = 0.0 - amount by which to inflate each transport unit
when layout out the build step.
"""
function process_schedule_build_step!(node, sched, scene_tree, bounding_circles,
    staging_circles; build_step_buffer_radius=0.0)

    open_build_step = node_val(node)
    assembly = open_build_step.assembly
    start_node = get_node(sched, AssemblyComplete(assembly))

    # optimize staging locations
    part_ids = sort(collect(keys(assembly_components(open_build_step))))
    parts = (get_node(scene_tree, part_id) for part_id in part_ids)
    tforms = (child_transform(assembly, id) for id in part_ids) # working in assembly frame
    θ_des = Vector{Float64}()
    radii = Vector{Float64}()
    geoms = Vector{LazySets.Ball2}()

    # bounding circle at current stage
    bounding_circle = get!(bounding_circles, node_id(assembly), LazySets.Ball2(zeros(SVector{2,Float64}), 0.0))
    for (part_id, part, tform) in zip(part_ids, parts, tforms)
        tu = get_node(scene_tree, TransportUnitNode(part))
        tu_tform = tform ∘ inv(child_transform(tu, part_id))
        tu_geom = project_to_2d(tu_tform(get_base_geom(tu, HypersphereKey())))
        d = tu_geom.center - bounding_circle.center
        r = tu_geom.radius
        push!(geoms, tu_geom)
        # Store transformed geometry for staging placement optimization
        push!(θ_des, wrap_to_pi(atan(d[2], d[1])))
        push!(radii, r + build_step_buffer_radius)
    end

    if !isempty(geoms)
        # Compute new bounding circle which contains the assembly at the end of this build step
        new_bounding_circle = overapproximate(
            vcat(geoms, bounding_circle),
            LazySets.Ball2{Float64,SVector{2,Float64}}
        )
        bounding_circles[node_id(assembly)] = new_bounding_circle # for assembly
        bounding_circles[node_id(node)] = new_bounding_circle # for build step
        # incorporate new bounding circle into OpenBuildStep
        open_build_step.bounding_circle.base_geom = project_to_3d(new_bounding_circle)
    end
    if length(geoms) == 1 && bounding_circle.radius < 1e-6
        @info "Only 1 component to place at first build step--no need for ring optimization - $(summary(part_ids[1]))"
        θ_star, assembly_radius = θ_des, bounding_circle.radius
    else
        # optimize placement and increase assembly_radius if necessary
        θ_star, assembly_radius = solve_ring_placement_problem(
            θ_des,
            radii,
            bounding_circle.radius,
        )
        Δθ = θ_star .- θ_des
        if maximum(abs.(Δθ)) > 0.1
            @info "ring placement solution $(node_id(node))" assembly_radius radii θ_des θ_star Δθ
        end
    end

    # Compute staging config transforms (relative to parent assembly)
    tformed_geoms = Vector{LazySets.Ball2}()
    for (i, (θ, r, part_id, part, tform)) in enumerate(zip(θ_star, radii, part_ids, parts, tforms))
        tu = get_node(scene_tree, TransportUnitNode(part))
        _child_tform = child_transform(tu, part_id).translation
        lift_node = get_node(sched, LiftIntoPlace(get_node(scene_tree, part_id)))
        # Compute the offset transform (relative to the assembly center)
        t = identity_linear_map()
        if length(geoms) == 1 && assembly_radius < 1e-6
            @info "Only 1 component to place at first build step--R = 0"
            R = 0.0
            set_local_transform_in_global_frame!(start_config(lift_node), t)
        else
            R = assembly_radius + r
            ### Use set_local_transform_in_global_frame!
            t = CoordinateTransformations.Translation(
                R * cos(θ) + bounding_circle.center[1] + _child_tform[1],
                R * sin(θ) + bounding_circle.center[2] + _child_tform[2],
                0.0)
            # @show t
            # set start config of lift node - have to add the ∘ inv(tform) because lift_node's goal_config() is already at tform
            tr = CoordinateTransformations.Translation(tform.translation[1:2]..., 0.0)
            set_local_transform_in_global_frame!(start_config(lift_node), t ∘ inv(tr))
        end
        # Store transformed geometry
        geom = project_to_2d(t(get_base_geom(tu, HypersphereKey())))
        push!(tformed_geoms, geom)
    end

    # Compute next staging circle by overapproximating current bounding circle and new geometry
    if haskey(staging_circles, node_id(assembly))
        # carry over existing staging circle geometry
        push!(tformed_geoms, staging_circles[node_id(assembly)])
    end
    old_ball = get(staging_circles, node_id(assembly), LazySets.Ball2(zeros(SVector{2,Float64}), 0.0))
    new_ball = overapproximate(
        vcat(tformed_geoms, bounding_circles[node_id(assembly)]),
        LazySets.Ball2{Float64,SVector{2,Float64}}
    )
    @assert new_ball.radius + 1e-5 > old_ball.radius + norm(new_ball.center .- old_ball.center) "$(summary(node_id(assembly))) old ball $(old_ball), new_ball $(new_ball)"
    staging_circles[node_id(assembly)] = new_ball
    # incorporate new bounding circle into OpenBuildStep
    @info "Updating staging_circle for $(summary(node_id(assembly))) to $(staging_circles[node_id(assembly)]) based on $(summary(node_id(node)))"
    open_build_step.bounding_circle.base_geom = project_to_3d(bounding_circles[node_id(node)])
    open_build_step.staging_circle.base_geom = project_to_3d(staging_circles[node_id(assembly)])

    sched
end

"""
    solve_ring_placement_problem(θ_des,R,r,rmin)

Formulate and solve a JuMP Model that encodes a ring optimization problem:
    Min_θ sum((θ .- θ_des).^2)
    s.t.  no overlapping of circles placed along ring
If there are too many circles, the problem will be infeasible. Either allow R
to increase as a variable, or use a search to optimize R while repeating the
optimization.
return θ_star
"""
function solve_ring_placement_problem(θ_des, r, R, rmin=0.0;
    ϵ=1e-1, # buffer for increasing R when necessary
    weights=ones(length(θ_des))
)
    model = Model(default_geom_optimizer())
    set_optimizer_attributes(model, default_geom_optimizer_attributes()...)

    θ_des = map(wrap_to_pi, θ_des)

    n = length(θ_des)
    @assert length(r) == n "length(r) != n for n=$n, r = $r"
    # sort θ (the unsorted vector will be returned at the end)
    idxs = sortperm(θ_des)
    reverse_idxs = sortperm(idxs)
    θ_des = θ_des[idxs]
    r = r[idxs]
    # compute half widths in radians (required radial spacing between parts)
    half_widths = asin.(r ./ (r .+ R))
    # increase R and recompute half widths if ring is too small
    while sum(half_widths) * 2 >= 2.0 * Float64(π)
        @info "R = $R is too small----sum(half_widths)*2 = $(sum(half_widths)*2)"
        R = R * sum(half_widths) / (Float64(π)) + ϵ
        half_widths = asin.(r ./ (r .+ R))
        @info "Increased R to $R. sum(half_widths)*2 = $(sum(half_widths)*2)"
    end
    @variable(model, θ[1:n])
    # Constrain order and spacing of elements of θ
    for i in 1:n-1
        @constraint(model, θ[i] + half_widths[i] <= θ[i+1] - half_widths[i+1])
    end
    # wrap-around constraint
    @constraint(model, θ[n] + half_widths[n] <= θ[1] - half_widths[1] + 2.0 * Float64(π))
    @objective(model, Min, sum(weights .* (θ .- θ_des) .^ 2))

    optimize!(model)
    if !(primal_status(model) == MOI.FEASIBLE_POINT)
        @warn "Ring optimization failed!"
    end

    return value.(θ)[reverse_idxs], R
end

"""
    calibrate_transport_tasks!(sched)

Set the appropriate transforms for all transport tasks such that
- FormTransportUnit moves the object from its initial condition to its
    transport-unit-relative transform: `start_config(n::FormTransportUnit)` will
    be directly below the object's start config.
- DepositTransportUnit
"""
function calibrate_transport_tasks!(sched)
    for node in get_nodes(sched)
        if matches_template(AssemblyComplete, node)
            assembly = entity(node)
            for (id, tf) in assembly_components(assembly)
                if matches_template(ObjectID, id)
                    cargo = ObjectNode(id, GeomNode(nothing))
                else
                    cargo = AssemblyNode(id, GeomNode(nothing))
                end
                start_node, form_transport, go_node, deposit_node, lift_node = get_transport_node_sequence(sched, cargo)
                cargo = entity(lift_node)
                # line up lift_node to target assembly
                set_local_transform!(goal_config(lift_node), tf)
                # line up deposit and collect nodes with floor
                align_construction_predicates!(sched, start_node, form_transport)
                align_construction_predicates!(sched, lift_node, deposit_node)
            end
        end
    end
    return sched
end

"""
    align_construction_predicates!(sched,start::Union{ObjectStart,AssemblyComplete,LiftIntoPlace},t::Union{DepositCargo,FormTransportUnit})

Line up pick up and drop off so that the carrying config of the cargo is directly
under its deployed config.
"""
function align_construction_predicates!(sched, start::Union{ObjectStart,AssemblyComplete,LiftIntoPlace}, t::Union{DepositCargo,FormTransportUnit})
    transport_unit = entity(t)
    cargo = entity(start)
    @assert node_id(cargo) == cargo_id(transport_unit)
    @assert has_parent(cargo_deployed_config(t), start_config(start))
    @assert has_parent(cargo_loaded_config(t), cargo_deployed_config(t))

    # Park Transport Unit so that cargo can drop straight down
    tform = child_transform(transport_unit, node_id(cargo))
    delta_z = tform.translation[3] - global_transform(cargo_deployed_config(t)).translation[3]
    # TODO would be nice to have constrained transforms (e.g., fix to X-Y plane)
    set_local_transform!(
        cargo_loaded_config(t),
        CoordinateTransformations.Translation(0.0, 0.0, delta_z) ∘ identity_linear_map()
    ) # relative to cargo_deployed_config(t)
end
align_construction_predicates!(sched, a::CustomNode, b::CustomNode) = align_construction_predicates!(sched, node_val(a), node_val(b))

"""
    get_transport_node_sequence(sched,cargo::Union{AsemblyNode,ObjectNode})

Retrieves the following node sequence from sched for transporting `cargo`.
    ObjectStart / AssemblyComplete
    FormTransportUnit
    TransportUnitGo
    DepositCargo
    LiftIntoPlace
example
```julia
start, form_transport, go, deposit, lift = get_transport_node_sequence(sched,cargo)
```
"""
function get_transport_node_sequence(sched, cargo::Union{AssemblyNode,ObjectNode})
    if matches_template(ObjectNode, cargo)
        start_node = get_node(sched, ObjectStart(cargo))
    else
        start_node = get_node(sched, AssemblyComplete(cargo))
    end
    form_transport = get_node(sched, FormTransportUnit(TransportUnitNode(cargo)))
    transport_unit = entity(form_transport)
    go_node = get_node(sched, TransportUnitGo(transport_unit))
    deposit_node = get_node(sched, DepositCargo(transport_unit))
    lift_node = get_node(sched, LiftIntoPlace(cargo))
    return start_node, form_transport, go_node, deposit_node, lift_node
end

"""
    transport_sequence_sanity_check(sched,cargo)

Print out the sequence of "stops" for the transport of `cargo`.
"""
function transport_sequence_sanity_check(sched, cargo)
    start_node, form_transport, go_node, deposit_node, lift_node = get_transport_node_sequence(sched, cargo)
    @show global_transform(start_config(start_node))
    @show global_transform(cargo_deployed_config(form_transport))
    @show global_transform(cargo_loaded_config(form_transport))
    @show global_transform(start_config(form_transport))
    @show global_transform(start_config(go_node))
    @show global_transform(goal_config(go_node))
    @show global_transform(start_config(deposit_node))
    @show global_transform(cargo_loaded_config(deposit_node))
    @show global_transform(cargo_deployed_config(deposit_node))
    @show global_transform(start_config(lift_node))
    @show global_transform(goal_config(lift_node))
    return
end

"""
    get_max_object_transport_unit_radius(scene_tree,key=HypersphereKey())
"""
function get_max_object_transport_unit_radius(scene_tree, key=HypersphereKey())
    r = 0.0
    for n in get_nodes(scene_tree)
        if matches_template(TransportUnitNode, n)
            if matches_template(ObjectNode, cargo_type(n))
                r = max(r, get_base_geom(n, key).radius)
            end
        end
    end
    return r
end

export set_scene_tree_to_initial_condition!

get_start_node(n::SceneNode, sched) = get_node(sched, get_start_node(n))
get_start_node(n::RobotNode) = RobotStart(n)
get_start_node(n::ObjectNode) = ObjectStart(n)
get_start_node(n::AssemblyNode) = AssemblyComplete(n)
get_start_node(n::TransportUnitNode) = FormTransportUnit(n)

function get_parent_build_step(sched, n::DepositCargo)
    for v in inneighbors(sched, n)
        np = get_node(sched, v)
        if matches_template(OpenBuildStep, np)
            return np
        end
    end
    return nothing
end
get_parent_build_step(sched, n::Union{FormTransportUnit,TransportUnitGo}) = get_parent_build_step(sched, get_node(sched, DepositCargo(entity(n))))
function get_parent_build_step(sched, n::Union{RobotGo,RobotStart})
    if outdegree(sched, n) > 0
        return get_parent_build_step(sched, first(outneighbors(sched, n)))
    end
    return nothing
end
get_parent_build_step(sched, n::ScheduleNode) = get_parent_build_step(sched, n.node)
get_parent_build_step(sched, v::Int) = get_parent_build_step(sched, get_node(sched, v))
get_first_build_step(sched, n::AssemblyStart) = get_node(sched, first(outneighbors(sched, n)))
get_first_build_step(sched, n::ScheduleNode) = get_first_build_step(sched, n.node)
get_first_build_step(sched, n::CustomNode) = get_first_build_step(sched, node_val(n))


"""
    set_scene_tree_to_initial_condition!(scene_tree,sched)

Once the staging plan has been computed, this function moves all entities to
their starting locations.
"""
function set_scene_tree_to_initial_condition!(scene_tree, sched;
    remove_all_edges=false,
)
    if remove_all_edges
        for e in collect(edges(scene_tree))
            force_remove_edge!(scene_tree, edge_source(e), edge_target(e))
        end
    end
    for scene_node in node_iterator(scene_tree, topological_sort_by_dfs(scene_tree))
        if has_vertex(sched, get_start_node(scene_node))
            n = get_start_node(scene_node, sched)
            goal = global_transform(start_config(n))
        else
            goal = identity_linear_map()
        end
        set_desired_global_transform!(scene_node, goal)
    end
    # for n in get_nodes(sched)
    #     if matches_template(Union{RobotStart,ObjectStart,AssemblyStart,FormTransportUnit},n)
    #         scene_node = get_node(scene_tree,node_id(entity(n)))
    #         # @assert has_parent(scene_node,scene_node)
    #         # set_local_transform!(scene_node,global_transform(start_config(n)))
    #         set_desired_global_transform!(scene_node,global_transform(start_config(n)))
    #     end
    # end
    scene_tree
end

"""
    select_initial_object_grid_locations!(sched,scene_tree,vtxs)

Cycle through `vtxs`, placing an object at each vertex until all objects have
been placed.
"""
function select_initial_object_grid_locations!(sched, vtxs)
    nodes = Vector{ObjectNode}()
    # collect nodes by walking through the schedule, so that the objects will be
    # sorted by precedence
    for node in filtered_topological_sort(sched, LiftIntoPlace)
        cargo = entity(node)
        if matches_template(ObjectNode, cargo)
            push!(nodes, cargo)
        end
    end
    tforms = map(
        v -> CoordinateTransformations.Translation(v[1], v[2], v[3]) ∘ identity_linear_map(),
        vtxs
    )
    for (node, tform) in zip(nodes, Base.Iterators.cycle(tforms))
        start_node = get_node(sched, ObjectStart(node))
        set_desired_global_transform!(start_config(start_node), tform)
    end
    sched
end

"""
    add_robots_to_scene!(scene_tree,vtxs,geoms=(default_robot_geom() for v in vtxs))

For each vtx in `vtxs`, place a robot at that location and add it to `scene_tree`
"""
function add_robots_to_scene!(scene_tree, vtxs, geoms=(default_robot_geom() for v in vtxs))
    for (vtx, geom) in zip(vtxs, Base.Iterators.cycle(geoms))
        tform = CoordinateTransformations.Translation(vtx[1], vtx[2], 0.0) ∘ identity_linear_map()
        robot_node = add_node!(scene_tree,
            RobotNode(get_unique_id(RobotID), GeomNode(geom)))
        set_local_transform!(robot_node, tform)
    end
    scene_tree
end

"""
    set_robot_start_configs!(sched,scene_tree)

For each `n::RobotNode` in `SceneTree`, add a corresponding `RobotStart` and
`RobotGo` node and set the start transform to `local_transform(n)`.
"""
function set_robot_start_configs!(sched, scene_tree)
    for node in get_nodes(scene_tree)
        if matches_template(RobotNode, node)
            if !has_vertex(sched, RobotStart(node))
                start_node = add_node!(sched, RobotStart(node))
            else
                start_node = get_node(sched, RobotStart(node))
            end
            set_local_transform!(start_config(start_node), global_transform(node))
            if !has_vertex(sched, RobotGo(node))
                go_node = add_node!(sched, RobotGo(node))
            else
                go_node = get_node(sched, RobotGo(node))
            end
            add_edge!(sched, start_node, go_node)
            set_parent!(start_config(go_node), goal_config(start_node))
        end
    end
    sched
end

"""
    construct_vtx_array(;
        origin=SVector(0.0,0.0),
        spacing=(1.0,1.0),
        ranges=(-10:10,-10:10),
        obstacles=nothing,
        bounds=nothing,
        )

Construct a regular array of vertices over `ranges`, beginning at `origin` and
spaced by `spacing`, removing all vertices that fall within `obstacles` or
outside of `bounds`.
"""
function construct_vtx_array(;
    origin=SVector(0.0, 0.0, 0.0),
    spacing=(1.0, 1.0, 0.0),
    ranges=(-10:10, -10:10, 0:0),
    obstacles=nothing,
    bounds=nothing
)
    pts = Vector{SVector{3,Float64}}()
    for idxs in Base.Iterators.product(ranges...)
        pt = origin + SVector(map(i -> spacing[i] * idxs[i], 1:length(spacing))...)
        pt2d = project_to_2d(pt)
        legal = true
        if !(bounds === nothing)
            for ob in bounds
                if !(pt2d in ob)
                    legal = false
                    break
                end
            end
        end
        if !(obstacles === nothing)
            for ob in obstacles
                if pt2d in ob
                    legal = false
                    break
                end
            end
        end
        if legal
            push!(pts, pt)
        end
    end
    return pts
end


"""
    init_transport_units(scene_tree, robot_shape)
"""
function init_transport_units!(scene_tree; kwargs...)
    cvx_hulls = compute_hierarchical_2d_convex_hulls(scene_tree)
    for (id, pts) in cvx_hulls
        isempty(pts) ? continue : nothing
        cargo = get_node(scene_tree, id)
        transport_unit = configure_transport_unit(cargo, pts; kwargs...)
        if has_vertex(scene_tree, node_id(transport_unit))
            @warn "scene tree already has node $(node_id(transport_unit))"
        else
            add_node!(scene_tree, transport_unit)
        end
    end
    scene_tree, cvx_hulls
end

"""
    configure_transport_unit(cargo, pts;

Define the transport unit for cargo. `pts` are the eligible support points
"""
function configure_transport_unit(cargo, pts;
    robot_radius=default_robot_radius(),
    robot_height=default_robot_height()
)
    # initialize transport node with cargo id
    base_rect = get_base_geom(cargo, HyperrectangleKey())
    zmin = base_rect.center[3] .- base_rect.radius[3]
    cargo_tform = CoordinateTransformations.Translation(0.0, 0.0, robot_height - zmin) ∘ identity_linear_map()
    transport_unit = TransportUnitNode(node_id(cargo) => cargo_tform)
    support_pts = select_support_locations(VPolygon(pts), robot_radius)
    for pt in support_pts
        tform = CoordinateTransformations.Translation(pt[1], pt[2], 0.0,) ∘ identity_linear_map()
        add_robot!(transport_unit, tform)
    end
    transport_unit
end

"""
    add_temporary_invalid_robots!(scene_tree,transport_unit;
        geom=default_robot_geom(), with_edges=false)

Add invalid robots corresponding to the robots that would be part of
`transport_unit`.
"""
function add_temporary_invalid_robots!(scene_tree, transport_unit;
    geom=default_robot_geom(),
    with_edges=false,
)
    for (id, tform) in robot_team(transport_unit)
        if !has_vertex(scene_tree, id)
            robot_node = add_node!(scene_tree, RobotNode(id, GeomNode(geom)))
        else
            robot_node = get_node(scene_tree, id)
        end
        if with_edges
            set_child!(scene_tree, transport_unit, robot_node)
        end
    end
    return scene_tree
end

"""
    add_temporary_invalid_robots!(scene_tree;kwargs...)

Call `add_temporary_invalid_robots!(scene_tree, n;kwargs...)` for all
`n::TransportUnitNode`.
"""
function add_temporary_invalid_robots!(scene_tree; kwargs...)
    for n in get_nodes(scene_tree)
        if matches_template(TransportUnitNode, n)
            add_temporary_invalid_robots!(scene_tree, n; kwargs...)
        end
    end
end

"""
    remove_temporary_invalid_robots!(scene_tree)
    remove_temporary_invalid_robots!(scene_tree,transport_unit)

Remove all invalid robots (limited optionally to those associated with
`transport_unit::TransportUnitNode`) from scene_tree.
"""
function remove_temporary_invalid_robots!(scene_tree, transport_unit)
    for (id, _) in robot_team(transport_unit)
        if !valid_id(id)
            rem_node!(scene_tree, id)
        end
    end
    return scene_tree
end
function remove_temporary_invalid_robots!(scene_tree)
    ids = filter(id -> isa(id, BotID) && !valid_id(id), get_vtx_ids(scene_tree))
    for id in ids
        rem_node!(scene_tree, id)
    end
    scene_tree
end

"""
    select_optimal_carrying_configuration(pts::AbstractVector)

Return an AffineMap representing a rotation that minimizes the z-dimension of
the transformed points.
"""
function select_optimal_carrying_configuration(pts::AbstractVector)
    mat = hcat(convert.(Vector, pts)...)
    U, = svd(mat)
    if !isapprox(det(U), 1.0)
        @warn "det(U) = $(det(U)). Flipping last column"
        U = diagm([1.0, 1.0, -1.0]) * U
        @assert isapprox(det(U), 1.0)
    end
    return CoordinateTransformations.LinearMap(U) ∘ identity_linear_map()
end
function select_optimal_carrying_configuration(n::ObjectNode)
    select_optimal_carrying_configuration(coordinates(get_base_geom(n)))
end

"""
    find_step_numbers(start_node, part_ids, sched, scene_tree; max_depth=100)

Finds the number of steps down the tree the corresponding lift nodes are from the
start node. Returns a vector of Int where the i-th element is the steps from the start node
that the life node corresponding to the i-th part_id is.
"""
function find_step_numbers(start_node, part_ids, sched, scene_tree; max_depth=1000)
    steps_found = Vector{Int}(undef, length(part_ids))
    found = []
    lift_nodes_to_find = []
    for part_id in part_ids
        lift_node = get_node(sched, LiftIntoPlace(get_node(scene_tree, part_id)))
        push!(lift_nodes_to_find, lift_node.id)
    end
    ns = [start_node]
    cnt = 0
    continue_looking = true
    while continue_looking
        cnt += 1
        new_ns = []
        for n in ns
            ons = inneighbors(sched, n)
            for ni in sched.nodes[ons]
                for (ii, lift_node_id) in enumerate(lift_nodes_to_find)
                    if ii in found
                        continue
                    end
                    if ni.id == lift_node_id
                        push!(found, ii)
                        steps_found[ii] = cnt
                    end
                end
                if length(found) == length(lift_nodes_to_find)
                    continue_looking = false
                    break
                end
            end
            append!(new_ns, sched.nodes[ons])
        end
        ns = unique(new_ns)
        if length(ns) < 1 || cnt > max_depth
            continue_looking = false
            idxs = setdiff(1:length(lift_nodes_to_find), found)
            error("Could not find node $(lift_nodes_to_find[idxs]) in $(start_node.id). Searched to depth $(cnt)")
        end
    end
    return steps_found
end
