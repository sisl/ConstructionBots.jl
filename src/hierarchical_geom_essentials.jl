
global DEFAULT_GEOM_OPTIMIZER = nothing
global DEFAULT_GEOM_OPTIMIZER_ATTRIBUTES =
    Dict{Union{String,MOI.AbstractOptimizerAttribute},Any}()

"""
    default_geom_optimizer()

Returns the black box optimizer to use when formulating JuMP models.
"""
default_geom_optimizer() = DEFAULT_GEOM_OPTIMIZER

"""
    set_default_geom_optimizer!(optimizer)

Set the black box optimizer to use when formulating JuMP models.
"""
function set_default_geom_optimizer!(optimizer)
    global DEFAULT_GEOM_OPTIMIZER = optimizer
end

"""
    default_geom_optimizer_attributes()

Return a dictionary of default optimizer attributes.
"""
default_geom_optimizer_attributes() = DEFAULT_GEOM_OPTIMIZER_ATTRIBUTES

"""
    set_default_geom_optimizer_attributes!(vals)

Set default optimizer attributes.
e.g. `set_default_geom_optimizer_attributes!(Dict("PreSolve"=>-1))`
"""
function set_default_geom_optimizer_attributes!(pair::Pair, pairs...)
    push!(DEFAULT_GEOM_OPTIMIZER_ATTRIBUTES, pair)
    set_default_geom_optimizer_attributes!(pairs...)
end

set_default_geom_optimizer_attributes!(d::Dict) =
    set_default_geom_optimizer_attributes!(d...)
set_default_geom_optimizer_attributes!() = nothing

"""
    clear_default_geom_optimizer_attributes!()

Clear the default optimizer attributes.
"""
function clear_default_geom_optimizer_attributes!()
    empty!(DEFAULT_GEOM_OPTIMIZER_ATTRIBUTES)
end

const BaseGeometry = Union{LazySets.Ball2,Hyperrectangle,AbstractPolytope}
const BallType = LazySets.Ball2
get_center(s::LazySets.Ball2) = LazySets.center(s)
get_radius(s::LazySets.Ball2) = s.radius
get_center(s::GeometryBasics.HyperSphere) = s.center
get_radius(s::GeometryBasics.HyperSphere) = s.r
GeometryBasics.Sphere(s::LazySets.Ball2) =
    GeometryBasics.Sphere(GeometryBasics.Point(s.center...), s.radius)
Base.convert(::Type{S}, s::LazySets.Ball2) where {S<:GeometryBasics.HyperSphere} =
    S(GeometryBasics.Point(s.center...), s.radius)
const RectType = Hyperrectangle
get_center(s::Hyperrectangle) = s.center
get_radius(s::Hyperrectangle) = s.radius
GeometryBasics.HyperRectangle(s::Hyperrectangle) = GeometryBasics.HyperRectangle(
    GeometryBasics.Vec((s.center .- s.radius)...),
    2 * GeometryBasics.Vec(s.radius...),
)
global DEFAULT_ROBOT_GEOM = GeometryBasics.Cylinder(
    GeometryBasics.Point{3,Float64}(0.0, 0.0, 0.0),
    GeometryBasics.Point{3,Float64}(0.0, 0.0, 0.25),
    0.5,
)

"""
    default_robot_geom()

Get the default robot geometry.
"""
default_robot_geom() = DEFAULT_ROBOT_GEOM

"""
    set_default_robot_geom!(geom)

Set the default robot geometry.
"""
function set_default_robot_geom!(geom)
    global DEFAULT_ROBOT_GEOM = geom
end

function default_robot_radius()
    geom = default_robot_geom()
    if isa(geom, GeometryBasics.Cylinder)
        return geom.r
    else
        return 0.25
    end
end

function default_robot_height()
    geom = default_robot_geom()
    if isa(geom, GeometryBasics.Cylinder)
        return norm(geom.origin - geom.extremity)
    else
        return 0.1
    end
end

const Z_PROJECTION_MAT = SMatrix{2,3,Float64}(1.0, 0.0, 0.0, 1.0, 0.0, 0.0)

"""
    project_to_2d(geom,t=CoordinateTransformations.LinearMap(Z_PROJECTION_MAT)) = t(geom)
"""
project_to_2d(geom, t = CoordinateTransformations.LinearMap(Z_PROJECTION_MAT)) = t(geom)

"""
    project_to_3d(geom,t=CoordinateTransformations.LinearMap(transpose(Z_PROJECTION_MAT))) = t(geom)
"""
project_to_3d(geom, t = CoordinateTransformations.LinearMap(transpose(Z_PROJECTION_MAT))) =
    t(geom)

"""
    transform(geom,t)

Transform geometry `geom` according to the transformation `t`.
"""
transform(v, t) = t(v)
(t::CoordinateTransformations.Translation)(g::BaseGeometry) =
    LazySets.translate(g, Vector(t.translation))

"""
    (t::CoordinateTransformations.LinearMap)(g::Hyperrectangle)

"rotating" a Hyperrectangle `g` results in a new Hyperrectangle that bounds the
transformed version `g`.
"""
(t::CoordinateTransformations.LinearMap)(g::Hyperrectangle) =
    overapproximate(t(convert(LazySets.VPolytope, g)), Hyperrectangle)
(t::CoordinateTransformations.AffineMap)(g::Hyperrectangle) =
    overapproximate(t(convert(LazySets.VPolytope, g)), Hyperrectangle)
(t::CoordinateTransformations.Translation)(g::Hyperrectangle) =
    Hyperrectangle(t(g.center), g.radius)
for T in (:(CoordinateTransformations.AffineMap),)
    @eval begin
        (t::$T)(v::V) where {N<:GeometryBasics.Ngon,V<:AbstractVector{N}} = V(map(t, v))
        (t::$T)(g::C) where {C<:GeometryBasics.Cylinder} =
            C(t(g.origin), t(g.extremity), g.r)
        (t::$T)(g::VPolytope) = VPolytope(map(t, vertices_list(g)))
        (t::$T)(g::HPolytope) = HPolytope(map(t, constraints_list(g)))
        (t::$T)(g::VPolygon) = VPolytope(map(t, vertices_list(g)))
        (t::$T)(g::HPolygon) = HPolytope(map(t, constraints_list(g)))
        # (t::$T)(g::BufferedPolygon) = BufferedPolygon(map(t,g.halfspaces),map(t,g.pts),g.min_face_length)
        # (t::$T)(g::BufferedPolygonPrism) = BufferedPolygonPrism(t(g.p),t(g.origin),t(g.extremity))
        (t::$T)(::Nothing) = nothing
        (t::$T)(g::LazySets.Ball2) = LazySets.Ball2(t(g.center), g.radius)
    end
end
for T in (:(CoordinateTransformations.LinearMap), :(CoordinateTransformations.Translation))
    @eval begin
        (t::$T)(v::V) where {N<:GeometryBasics.Ngon,V<:AbstractVector{N}} = V(map(t, v))
        (t::$T)(g::G) where {G<:GeometryBasics.Ngon} = G(map(t, g.points))
        (t::$T)(g::C) where {C<:GeometryBasics.Cylinder} =
            C(t(g.origin), t(g.extremity), g.r)
        (t::$T)(g::VPolytope) = VPolytope(map(t, vertices_list(g)))
        (t::$T)(g::HPolytope) = HPolytope(map(t, constraints_list(g)))
        (t::$T)(g::VPolygon) = VPolytope(map(t, vertices_list(g)))
        (t::$T)(g::HPolygon) = HPolytope(map(t, constraints_list(g)))
        # (t::$T)(g::BufferedPolygon) = BufferedPolygon(map(t,g.halfspaces),map(t,g.pts),g.min_face_length)
        # (t::$T)(g::BufferedPolygonPrism) = BufferedPolygonPrism(t(g.p),t(g.origin),t(g.extremity))
        (t::$T)(::Nothing) = nothing
        (t::$T)(g::LazySets.Ball2) = LazySets.Ball2(t(g.center), g.radius)
    end
end

identity_linear_map3() = CoordinateTransformations.compose(
    CoordinateTransformations.Translation(zero(SVector{3,Float64})),
    CoordinateTransformations.LinearMap(one(SMatrix{3,3,Float64})),
)
identity_linear_map2() = CoordinateTransformations.compose(
    CoordinateTransformations.Translation(zero(SVector{3,Float64})),
    CoordinateTransformations.LinearMap(one(SMatrix{3,3,Float64})),
)
identity_linear_map() = identity_linear_map3()
scaled_linear_map(scale) =
    CoordinateTransformations.LinearMap(scale * one(SMatrix{3,3,Float64})) ∘
    identity_linear_map()
Base.convert(::Type{Hyperrectangle{Float64,T,T}}, rect::Hyperrectangle) where {T} =
    Hyperrectangle(T(rect.center), T(rect.radius))

"""
    relative_transform(a::AffineMap,b::AffineMap)

compute the relative transform `t` between two frames, `a`, and `b` such that
    t = inv(a) ∘ b
    b = a ∘ t
    a = b ∘ inv(t)

    i.e. (a ∘ t)(p) == a(t(p)) == b(p)
"""
relative_transform(
    a::A,
    b::B,
) where {
    A<:CoordinateTransformations.Transformation,
    B<:CoordinateTransformations.Transformation,
} = inv(a) ∘ b

function Rotations.rotation_error(
    a::CoordinateTransformations.AffineMap,
    b::CoordinateTransformations.AffineMap,
    error_map = MRPMap(),
)
    rot_err = Rotations.rotation_error(
        Rotations.QuatRotation(a.linear),
        Rotations.QuatRotation(b.linear),
        error_map,
    )
end
for op in (:relative_transform, :(Rotations.rotation_error))
    @eval $op(tree::AbstractCustomTree, parent, child, args...) =
        $op(global_transform(tree, parent), global_transform(tree, child), args...)
    @eval $op(parent, child, args...) =
        $op(global_transform(parent), global_transform(child), args...)
end

"""
    interpolate_rotation(Ra,Rb,c=0.5)

compute a rotation matrix `ΔR` such that `Ra*ΔR` is c/1.0 of the way to `Rb`.
    R = R⁻¹ Rb
    θ = axis + magnitude representation
    ΔR = exp(cross_product_operator(θ)*c)
"""
function interpolate_rotation(Ra, Rb, c = 0.5)
    R = inv(Ra) * Rb
    r = Rotations.RotationVec(R)  # rotation vector
    Rotations.RotationVec(r.sx * c, r.sy * c, r.sz * c)
end
function interpolate_transforms(Ta, Tb, c = 0.5)
    R = interpolate_rotation(Ta.linear, Tb.linear, c)
    t = (1 - c) * Ta.translation + c * Tb.translation
    CoordinateTransformations.Translation(t) ∘ CoordinateTransformations.LinearMap(R)
end

@with_kw struct TransformNodeID <: AbstractID
    id::Int = -1
end

"""
    TransformNode

Has a parent field that points to another TransformNode.
"""
mutable struct TransformNode <: CachedTreeNode{TransformNodeID}
    id::TransformNodeID
    local_transform::CoordinateTransformations.Transformation  # transform from the parent frame to the child frame
    global_transform::CachedElement{CoordinateTransformations.Transformation}
    parent::TransformNode  # parent node
    children::Dict{AbstractID,CachedTreeNode}
    function TransformNode(a::CoordinateTransformations.Transformation, b::CachedElement)
        t = new()
        t.id = get_unique_id(TransformNodeID)
        t.local_transform = a
        t.global_transform = b
        t.parent = t
        t.children = Dict{AbstractID,CachedTreeNode}()
        return t
    end
end
function TransformNode()
    TransformNode(identity_linear_map(), CachedElement(identity_linear_map()))
end

Base.show(io::IO, ::MIME"text/plain", m::TransformNode) = print(
    io,
    "TransformNode(",
    get_id(node_id(m)),
    ")\n",
    "  local:  ",
    local_transform(m),
    "\n",
    "  global: ",
    global_transform(m),
    "\n",
    "  parent: ",
    string(node_id(m.parent)),
    "\n",
    "  children: ",
    map(k -> string("\n    ", string(k)), collect(keys(get_children(m))))...,
    "\n",
)
Base.show(io::IO, m::TransformNode) =
    print(io, "TransformNode(", get_id(node_id(m)), ") - ", m.global_transform)

# Cached Tree interface
cached_element(n::TransformNode) = n.global_transform
tf_up_to_date(n::TransformNode) = cached_node_up_to_date(n)
set_tf_up_to_date!(n::TransformNode, val = true) = set_cached_node_up_to_date!(n, val)
local_transform(n::TransformNode) = n.local_transform
set_global_transform!(n::TransformNode, t, args...) = update_element!(n, t, args...)
global_transform(n::TransformNode) = get_cached_value!(n)

function propagate_forward!(parent::TransformNode, child::TransformNode)
    if parent === child
        set_global_transform!(child, local_transform(child))
    else
        set_global_transform!(child, global_transform(parent) ∘ local_transform(child))
    end
    global_transform(child)
end

function set_local_transform!(n::TransformNode, t, update = false)
    n.local_transform = t ∘ identity_linear_map()  # guarantee AffineMap?
    set_tf_up_to_date!(n, false)
    if update
        global_transform(n)  # just get the global transform
    end
    return n.local_transform
end

function set_local_transform_in_global_frame!(n::TransformNode, t, args...)
    rot_mat = CoordinateTransformations.LinearMap(global_transform(get_parent(n)).linear)
    set_local_transform!(n, inv(rot_mat) ∘ t)
end

export set_desired_global_transform!

"""
    set_desired_global_transform!(n::TransformNode,t,args...)

Set a transform node's global local transform such that its global transform
(respecting it's current parent) will be `t`.
"""
function set_desired_global_transform!(n::TransformNode, t, args...)
    if has_parent(n, n)
        set_local_transform!(n, t)
    else
        parent = get_parent(n)
        tform = relative_transform(global_transform(parent), t)
        set_local_transform!(n, tform)
    end
end
const transform_node_accessor_interface =
    [:tf_up_to_date, :local_transform, :global_transform]
const transform_node_mutator_interface = [
    :set_tf_up_to_date!,
    :set_local_transform!,
    :set_global_transform!,
    :set_local_transform_in_global_frame!,
    :set_desired_global_transform!,
]

export set_desired_global_transform_without_affecting_children!

"""
    set_desired_global_transform_without_affecting_children!()
"""
function set_desired_global_transform_without_affecting_children!(
    n::TransformNode,
    t,
    args...,
)
    tf_dict = Dict{TransformNodeID,CoordinateTransformations.AffineMap}()
    child_dict = get_children(n)
    for (id, c) in child_dict
        tf_dict[id] = global_transform(c)
    end
    set_desired_global_transform!(n, t, args...)
    for (id, tf) in tf_dict
        set_desired_global_transform!(child_dict[id], tf)
    end
    global_transform(n)
end

"""
    rem_parent!(child::TransformNode)

Ensure that the detached child does not "jump" in the global frame when detached
from the parent.
"""
function rem_parent!(child::TransformNode)
    tf = global_transform(child)
    delete!(get_children(get_parent(child)), node_id(child))
    set_local_transform!(child, tf)
    child.parent = child
end

for op in transform_node_accessor_interface
    @eval $op(tree::AbstractCustomTree, v) = $op(get_node(tree, v))
    @eval $op(tree::AbstractCustomTree, n::TransformNode) = $op(n)
end
for op in transform_node_mutator_interface
    @eval $op(tree::AbstractCustomTree, v, args...) = $op(get_node(tree, v), args...)
    @eval $op(tree::AbstractCustomTree, n::TransformNode, args...) = $op(n, args...)
end

"""
    set_child!(tree,parent,child,new_local_transform)

Replace existing edge `old_parent` → `child` with new edge `parent` → `child`.
Set the `child.local_transform` to `new_local_transform`.
"""
function set_child!(
    tree::AbstractCustomTree,
    parent,
    child,
    t = relative_transform(tree, parent, child),
    edge = nothing,
)
    Graphs.rem_edge!(tree, get_parent(tree, child), child)
    if Graphs.add_edge!(tree, parent, child, edge)
        set_parent!(get_node(tree, child), get_node(tree, parent))  # for TransformNode
        @assert !Graphs.is_cyclic(tree) "adding edge $(parent) → $(child) made tree cyclic"
        set_local_transform!(tree, child, t)
        return true
    end
    return false
end
const transform_tree_mutator_interface = [:set_local_transform!, :set_global_transform!]

function add_child!(tree::AbstractCustomTree, parent, child, child_id)
    n = add_node!(tree, child, child_id)
    if set_child!(tree, get_node(tree, parent), child_id)
        return n
    else
        rem_node!(tree, child_id)
        return nothing
    end
end

@with_kw struct GeomID <: AbstractID
    id::Int = -1
end

mutable struct GeomNode{G} <: CachedTreeNode{GeomID}
    id::GeomID
    base_geom::G
    parent::TransformNode
    cached_geom::CachedElement{G}
end

function GeomNode(geom)
    GeomNode(get_unique_id(GeomID), geom, TransformNode(), CachedElement(geom))
end

function GeomNode(geom, tf)
    GeomNode(get_unique_id(GeomID), geom, tf, CachedElement(geom))
end
get_children(n::GeomNode) = Dict{AbstractID,CachedTreeNode}()
cached_element(n::GeomNode) = n.cached_geom
set_cached_geom!(n::GeomNode, geom) = update_element!(n, geom)
get_base_geom(n::GeomNode) = n.base_geom
get_transform_node(n::GeomNode) = n.parent
for op in transform_node_accessor_interface
    @eval $op(g::GeomNode, args...) = $op(get_transform_node(g))
end
for op in transform_node_mutator_interface
    @eval $op(g::GeomNode, args...) = $op(get_transform_node(g), args...)
end
set_parent!(a::GeomNode, b::TransformNode) = set_parent!(get_transform_node(a), b)
set_parent!(a::GeomNode, b::GeomNode) = set_parent!(a, get_transform_node(b))

function propagate_forward!(t::TransformNode, n::GeomNode)
    transformed_geom = transform(get_base_geom(n), global_transform(n))
    set_cached_geom!(n, transformed_geom)
end

"""
    get_cached_geom(n::GeomNode)

If `n.cached_geom` is out of date, transform it according to
`global_transform(n)`. Updates both the global transform and geometry if
necessary.
"""
get_cached_geom(n::GeomNode) = get_cached_value!(n)

"""
    Base.copy(n::GeomNode)

Shares `n.base_geom`, deepcopies `n.transform_node`, and copies `n.cached_geom`
(see documenation for `Base.copy(::CachedElement)`).
"""
Base.copy(n::GeomNode) = GeomNode(
    get_unique_id(GeomID),
    n.base_geom,
    copy(get_transform_node(n)),
    copy(n.cached_geom),
)

abstract type GeometryKey end
"""
    BaseGeomKey <: GeometryKey

Points to any kind of geometry.
"""
struct BaseGeomKey <: GeometryKey end
"""
    PolyhedronKey <: GeometryKey

Points to a polyhedron.
"""
struct PolyhedronKey <: GeometryKey end
struct PolygonKey <: GeometryKey end
struct OctagonalPrismKey <: GeometryKey end
"""
    ZonotopeKey<: GeometryKey

Points to a zonotope. Might not be useful for approximating shapes that are very
asymmetrical.
"""
struct ZonotopeKey <: GeometryKey end
struct HyperrectangleKey <: GeometryKey end
struct HypersphereKey <: GeometryKey end
struct CylinderKey <: GeometryKey end
struct CircleKey <: GeometryKey end
struct ConvexHullKey <: GeometryKey end

construct_child_approximation(::PolyhedronKey, geom, args...) =
    LazySets.overapproximate(geom, equatorial_overapprox_model(), args...)
construct_child_approximation(::HypersphereKey, geom, args...) =
    LazySets.overapproximate(geom, LazySets.Ball2{Float64,SVector{3,Float64}}, args...)
construct_child_approximation(::HyperrectangleKey, geom, args...) =
    LazySets.overapproximate(
        geom,
        Hyperrectangle{Float64,SVector{3,Float64},SVector{3,Float64}},
        args...,
    )
construct_child_approximation(::PolygonKey, geom, args...) =
    LazySets.overapproximate(geom, ngon_overapprox_model(8), args...)
construct_child_approximation(::CircleKey, geom, args...) =
    LazySets.overapproximate(geom, LazySets.Ball2{Float64,SVector{2,Float64}}, args...)
construct_child_approximation(::CylinderKey, geom, args...) =
    LazySets.overapproximate(geom, GeometryBasics.Cylinder, args...)
construct_child_approximation(::OctagonalPrismKey, geom, args...) =
    LazySets.overapproximate(
        geom,
        BufferedPolygonPrism(
            regular_buffered_polygon(8, 1.0; buffer = 0.05 * default_robot_radius()),
        ),
        args...,
    )

"""
    GeometryHierarchy

A hierarchical representation of geometry
Fields:
* graph - encodes the hierarchy of geometries
* nodes - geometry nodes
"""
@with_kw struct GeometryHierarchy <: AbstractCustomNTree{GeomNode,GeometryKey}
    graph::Graphs.DiGraph = Graphs.DiGraph()
    nodes::Vector{GeomNode} = Vector{GeomNode}()
    vtx_map::Dict{GeometryKey,Int} = Dict{GeometryKey,Int}()
    vtx_ids::Vector{GeometryKey} = Vector{GeometryKey}()  # maps vertex uid to actual graph node
end

function geom_hierarchy(geom::GeomNode)
    h = GeometryHierarchy()
    add_node!(h, geom, BaseGeomKey())
    return h
end

for op in (:get_base_geom, :get_cached_geom)
    @eval begin
        function $op(n::GeometryHierarchy, k = BaseGeomKey())
            if Graphs.has_vertex(n, k)
                return $op(get_node(n, k))
            else
                return nothing
            end
        end
    end
end

"""
    add_child_approximation!(g::GeometryHierarchy, child_key, parent_key,
        base_geom=get_base_geom(g,parent_id), args...)

Overapproximate `g`'s type `parent_key` geometry with geometry of type
`child_key`, and add this new approximation to `g` under key `child_key` with an
edge from `parent_key` to `child_key`.
"""
function add_child_approximation!(
    g::GeometryHierarchy,
    child_id,
    parent_id,
    base_geom = get_base_geom(g, parent_id),
    args...,
)
    @assert Graphs.has_vertex(g, parent_id) "g does not have parent_id = $parent_id"
    @assert !Graphs.has_vertex(g, child_id) "g already has child_id = $child_id"
    node = get_node(g, parent_id)
    geom = construct_child_approximation(child_id, base_geom, args...)
    add_node!(
        g,
        GeomNode(geom, node.parent),  # share parent
        child_id,
        # TODO: Add parent
    )
    Graphs.add_edge!(g, parent_id, child_id)
    return g
end

"""
    abstract type SceneNode

A node of the `SceneTree`. Each concrete subtype of `SceneNode`
contains all of the following information:
- All required children (whether for a temporary or permanent edge)
- Required Parent (For a permanent edge only)
- Geometry of the entity represented by the node
- Unique ID accessible via `node_id(node)`
- Current transform--both global (relative to base frame) and local (relative to
current parent)
- Required transform relative to parent (if applicable)

"""
abstract type SceneNode end
abstract type SceneAssemblyNode <: SceneNode end
# SceneNode interface
node_id(n::SceneNode) = n.id
const geom_node_accessor_interface = [
    transform_node_accessor_interface...,
    :get_base_geom,
    :get_cached_geom,
    :get_transform_node,
]
const geom_node_mutator_interface = [transform_node_mutator_interface...]
for op in geom_node_accessor_interface
    @eval $op(n::SceneNode) = $op(n.geom)
    @eval $op(n::CustomNode) = $op(node_val(n))
end
for op in geom_node_mutator_interface
    @eval $op(n::SceneNode, args...) = $op(n.geom, args...)
    @eval $op(n::CustomNode, args...) = $op(node_val(n), args...)
end
set_parent!(a::SceneNode, b::SceneNode) = set_parent!(a.geom, b.geom)
set_parent!(a::CustomNode, b::CustomNode) = set_parent!(node_val(a), node_val(b))
rem_parent!(a::GeomNode) = rem_parent!(get_transform_node(a))
rem_parent!(a::SceneNode) = rem_parent!(a.geom)
rem_parent!(a::CustomNode) = rem_parent!(node_val(a))
for op in (:(has_parent), :(has_child), :(has_descendant))
    @eval begin
        $op(a::SceneNode, b::SceneNode) = $op(get_transform_node(a), get_transform_node(b))
    end
end
get_cached_geom(n::SceneNode, k::GeometryKey) = get_cached_geom(n.geom_hierarchy, k)
get_base_geom(n::SceneNode, k::GeometryKey) = get_base_geom(n.geom_hierarchy, k)

function add_child_approximation!(n::SceneNode, args...)
    add_child_approximation!(n.geom_hierarchy, args...)
end

"""
    required_transforms_to_children(n::SceneNode)

Returns a dictionary mapping child id to required relative transform.
"""
function required_transforms_to_children(n::SceneNode)
    return TransformDict{AbstractID}()
end

"""
    required_transform_to_parent(n::SceneNode,parent_id)

Returns the required transform relative to the parent.
"""
function required_transform_to_parent end

"""
    Base.copy(n::N) where {N<:SceneNode}

Shares `n.base_geom`, deepcopies `n.transform_node`, and copies `n.cached_geom`
(see documenation for `Base.copy(::CachedElement)`).
"""
Base.copy(n::N) where {N<:SceneNode} = N(n, copy(n.geom))

struct RobotNode{R} <: SceneNode
    id::BotID{R}
    geom::GeomNode
    geom_hierarchy::GeometryHierarchy
end

RobotNode(id::BotID, geom) = RobotNode(id, geom, geom_hierarchy(geom))
RobotNode(n::RobotNode, geom) = RobotNode(n.id, geom)
RobotNode(id::BotID, n::RobotNode) = RobotNode(id, n.geom, n.geom_hierarchy)

struct ObjectNode <: SceneNode
    id::ObjectID
    geom::GeomNode
    geom_hierarchy::GeometryHierarchy
end

ObjectNode(id::ObjectID, geom) = ObjectNode(id, geom, geom_hierarchy(geom))
ObjectNode(n::ObjectNode, geom) = ObjectNode(n.id, geom)
has_component(n::SceneNode, id) = false
# Necessary for copying
RobotNode{R}(n::RobotNode, args...) where {R} = RobotNode(n.id, args...)

const TransformDict{T} = Dict{T,CoordinateTransformations.Transformation}

"""
    abstract type SceneNode end

An Abstract type, of which all nodes in a SceneTree are concrete subtypes.
"""
@with_kw struct AssemblyID <: AbstractID
    id::Int = -1
end

struct AssemblyNode <: SceneNode
    id::AssemblyID
    geom::GeomNode
    components::TransformDict{Union{ObjectID,AssemblyID}}
    geom_hierarchy::GeometryHierarchy
end

AssemblyNode(n::AssemblyNode, geom) =
    AssemblyNode(n.id, geom, n.components, geom_hierarchy(geom))
AssemblyNode(id, geom) = AssemblyNode(
    id,
    geom,
    TransformDict{Union{ObjectID,AssemblyID}}(),
    geom_hierarchy(geom),
)
assembly_components(n::AssemblyNode) = n.components
add_component!(n::AssemblyNode, p) = push!(n.components, p)
has_component(n::AssemblyNode, id) = haskey(assembly_components(n), id)
child_transform(n::AssemblyNode, id) = assembly_components(n)[id]
num_components(n) = length(assembly_components(n))
required_transforms_to_children(n::AssemblyNode) = assembly_components(n)

struct TransportUnitNode{C<:Union{ObjectID,AssemblyID},T} <: SceneNode
    geom::GeomNode
    cargo::Pair{C,T}
    robots::TransformDict{BotID}  # must be filled with unique invalid ids
    geom_hierarchy::GeometryHierarchy
end

TransportUnitNode(n::TransportUnitNode, geom) =
    TransportUnitNode(geom, n.cargo, n.robots, geom_hierarchy(geom))
TransportUnitNode(geom, cargo) =
    TransportUnitNode(geom, cargo, TransformDict{BotID}(), geom_hierarchy(geom))
TransportUnitNode(geom, cargo_id::Union{AssemblyID,ObjectID}) =
    TransportUnitNode(geom, cargo_id => identity_linear_map())
TransportUnitNode(cargo::Pair) = TransportUnitNode(GeomNode(nothing), cargo)
TransportUnitNode(cargo_id::Union{AssemblyID,ObjectID}) =
    TransportUnitNode(cargo_id => identity_linear_map())
TransportUnitNode(cargo::Union{AssemblyNode,ObjectNode}) = TransportUnitNode(node_id(cargo))
robot_team(n::TransportUnitNode) = n.robots
cargo_id(n::TransportUnitNode) = n.cargo.first
cargo_type(n::TransportUnitNode) = isa(cargo_id(n), AssemblyID) ? AssemblyNode : ObjectNode
Base.copy(n::TransportUnitNode) = TransportUnitNode(n, copy(n.geom))

function required_transforms_to_children(n::TransportUnitNode)
    merge(robot_team(n), Dict(n.cargo))
end

const TransportUnitID = TemplatedID{Tuple{T,C}} where {C,T<:TransportUnitNode}

node_id(n::TransportUnitNode{C,T}) where {C,T} =
    TemplatedID{Tuple{TransportUnitNode,C}}(get_id(cargo_id(n)))
Base.convert(::Pair{A,B}, pair) where {A,B} = convert(A, pair.first) => convert(B, p.second)

has_component(n::TransportUnitNode, id::Union{ObjectID,AssemblyID}) = id == cargo_id(n)
has_component(n::TransportUnitNode, id::BotID) = haskey(robot_team(n), id)
child_transform(n::TransportUnitNode, id::Union{ObjectID,AssemblyID}) =
    has_component(n, id) ? n.cargo.second :
    throw(ErrorException("TransportUnitNode $n does not have component $id"))
child_transform(n::TransportUnitNode, id::BotID) = robot_team(n)[id]
add_robot!(n::TransportUnitNode, p) = push!(robot_team(n), p)
remove_robot!(n::TransportUnitNode, id) = delete!(robot_team(n), id)
add_robot!(n::TransportUnitNode, r, t) = add_robot!(n, r => t)
add_robot!(n::TransportUnitNode, t::CoordinateTransformations.AffineMap) =
    add_robot!(n, get_unique_invalid_id(RobotID) => t)

function is_in_formation(n::TransportUnitNode, scene_tree)
    all([Graphs.has_edge(scene_tree, n, id) for (id, _) in robot_team(n)])
end

function capture_robots!(agent::TransportUnitNode, scene_tree)
    formed = true
    for (robot_id, _) in robot_team(agent)
        if !Graphs.has_edge(scene_tree, agent, robot_id)
            if !capture_child!(scene_tree, agent, robot_id)
                formed = false
            end
        end
    end
    return formed
end

"""
    swap_robot_id!(transport_unit,old_id,new_id)

Robot `new_id` takes robot `old_id`'s plave in `robot_team(transport_unit)`.
"""
function swap_robot_id!(transport_unit, old_id, new_id)
    team = robot_team(transport_unit)
    tform = child_transform(transport_unit, old_id)
    remove_robot!(transport_unit, old_id)
    add_robot!(transport_unit, new_id => tform)
end

export recurse_child_geometry

"""
    recurse_child_geometry(node::SceneNode,tree,key=BaseGeomKey(),depth=typemax(Int))

Return an iterator over all geometry elements matching `key` that belong to
`node` or to any of `node`'s descendants down to depth `depth`.
"""
function recurse_child_geometry(
    node::SceneNode,
    tree,
    key = BaseGeomKey(),
    depth = typemax(Int),
)
    if depth < 0
        return nothing
    end
    geom = [get_base_geom(node, key)]
    if !(geom[1] === nothing)
        return geom
    end
    return nothing
end

function recurse_child_geometry(
    dict::TransformDict,
    tree,
    key = BaseGeomKey(),
    depth = typemax(Int),
)
    if depth < 0
        return nothing
    end
    geoms = []
    for (child_id, tform) in dict
        if Graphs.has_vertex(tree, child_id)
            child_geom = recurse_child_geometry(get_node(tree, child_id), tree, key, depth)
            if !(child_geom === nothing)
                push!(geoms, transform_iter(tform, child_geom))
            end
        end
    end
    if isempty(geoms)
        return nothing
    end
    Base.Iterators.flatten(geoms)
end

function recurse_child_geometry(
    node::Union{TransportUnitNode,AssemblyNode},
    tree,
    key = BaseGeomKey(),
    depth = typemax(Int),
)
    if depth < 0
        return nothing
    end
    geom = [get_base_geom(node, key)]
    child_geom =
        recurse_child_geometry(required_transforms_to_children(node), tree, key, depth - 1)
    if (geom[1] === nothing)
        return child_geom
    elseif (child_geom === nothing)
        return geom
    else
        return Base.Iterators.flatten((geom, child_geom))
    end
end

"""
    abstract type SceneTreeEdge

Concrete subtypes are `PermanentEdge` or `TemporaryEdge`. The idea is that
the former is not allowed to be removed, whereas the latter stays in place.
"""
abstract type SceneTreeEdge end
struct PermanentEdge <: SceneTreeEdge end
struct TemporaryEdge <: SceneTreeEdge end
for U in (:TransportUnitID, :TransportUnitNode)
    for V in (:RobotID, :RobotNode, :AssemblyID, :AssemblyNode, :ObjectID, :ObjectNode)
        @eval make_edge(g, u::$U, v::$V) = TemporaryEdge()
    end
end
for U in (:AssemblyID, :AssemblyNode)
    for V in (:ObjectID, :ObjectNode, :AssemblyID, :AssemblyNode)
        @eval make_edge(g, u::$U, v::$V) = PermanentEdge()
    end
end

"""
    SceneTree <: AbstractCustomNETree{SceneNode,SceneTreeEdge,AbstractID}

A tree data structure for describing the current configuration of a multi-entity
system, as well as the desired final configuration. The configuration of the
system is defined by the topology of the tree (i.e., which nodes are connected
by edges) and the configuration of each node (its transform relative to its
parent node).

There are four different concrete subtypes of `SceneNode`
- `ObjectNode` refers to a single inanimate rigid body
- `RobotNode` refers to a single rigid body robot
- `AssemblyNode` refers to a rigid collection of multiple objects and/or subassemblies
- `TransportUnitNode` refers to a team of one or more robot that fit together as
a transport team to move an assembly or object.

The edges of the SceneTree are either `PermanentEdge` or `TemporaryEdge`.

Temporary edges (removed after transport is complete)
- TransportUnitNode → RobotNode
- TransportUnitNode → ObjectNode
- TransportUnitNode → AssemblyNode

Permanent edges (cannot be broken after placement)
- AssemblyNode → AssemblyNode
- AssemblyNode → ObjectNode
"""
@with_kw struct SceneTree <: AbstractCustomNETree{SceneNode,SceneTreeEdge,AbstractID}
    graph::Graphs.DiGraph = Graphs.DiGraph()
    nodes::Vector{SceneNode} = Vector{SceneNode}()
    vtx_map::Dict{AbstractID,Int} = Dict{AbstractID,Int}()
    vtx_ids::Vector{AbstractID} = Vector{AbstractID}()
    inedges::Vector{Dict{Int,SceneTreeEdge}} = Vector{Dict{Int,SceneTreeEdge}}()
    outedges::Vector{Dict{Int,SceneTreeEdge}} = Vector{Dict{Int,SceneTreeEdge}}()
end

add_node!(tree::SceneTree, node::SceneNode) = add_node!(tree, node, node_id(node))
get_vtx(tree::SceneTree, n::SceneNode) = get_vtx(tree, node_id(n))

function Base.copy(tree::SceneTree)
    SceneTree(
        graph = deepcopy(tree.graph),
        nodes = map(copy, tree.nodes), # TODO: Check if this causes problems with TransformNode
        vtx_map = deepcopy(tree.vtx_map),
        vtx_ids = deepcopy(tree.vtx_ids),
    )
end

"""
    set_child!(tree::SceneTree,parent::AbstractID,child::AbstractID)

Only eligible children can be added to a candidate parent in a `SceneTree`.
Eligible edges are those for which the child node is listed in the components of
the parent. `ObjectNode`s and `RobotNode`s may not have any children.
"""
function set_child!(tree::SceneTree, parent::AbstractID, child::AbstractID)
    node = get_node(tree, parent)
    @assert has_component(node, child) "$(get_node(tree,child)) cannot be a child of $(node)"
    t = child_transform(node, child)
    child_node = get_node(tree, child)
    set_child!(tree, parent, get_vtx(tree, child), t, make_edge(tree, node, child_node))
end

set_child!(tree::SceneTree, parent::SceneNode, args...) =
    set_child!(tree, node_id(parent), args...)
set_child!(tree::SceneTree, parent::SceneNode, child::SceneNode) =
    set_child!(tree, node_id(parent), node_id(child))
set_child!(tree::SceneTree, parent::AbstractID, child::SceneNode, args...) =
    set_child!(tree, parent, node_id(child), args...)

function force_remove_edge!(tree::SceneTree, u, v)
    rem_parent!(get_node(tree, v))
    if Graphs.has_edge(tree, u, v)
        delete_edge!(tree, u, v)
    end
end

"""
    Graphs.rem_edge!(tree::SceneTree,u,v)

Edge may only be removed if it is not permanent.
"""
function Graphs.rem_edge!(tree::SceneTree, u, v)
    if !Graphs.has_edge(tree, u, v)
        return true
    end
    @assert !isa(get_edge(tree, u, v), PermanentEdge) "Edge $u → $v is permanent!"
    force_remove_edge!(tree, u, v)
end

"""
    disband!(tree::SceneTree,n::TransportUnitNode)

Disband a transport unit by removing the edge to its children.
"""
function disband!(tree::SceneTree, n::TransportUnitNode)
    for (r, tform) in robot_team(n)
        if !Graphs.has_edge(tree, n, r)
            @warn "Disbanding $n -- $r should be attached, but is not"
        end
        Graphs.rem_edge!(tree, n, r)
    end
    Graphs.rem_edge!(tree, n, cargo_id(n))
    return true
end

global CAPTURE_DISTANCE_TOLERANCE = 1e-2
capture_distance_tolerance() = CAPTURE_DISTANCE_TOLERANCE

function set_capture_distance_tolerance!(val)
    global CAPTURE_DISTANCE_TOLERANCE = val
end

global CAPTURE_ROTATION_TOLERANCE = 1e-2
capture_rotation_tolerance() = CAPTURE_ROTATION_TOLERANCE

function set_capture_rotation_tolerance!(val)
    global CAPTURE_ROTATION_TOLERANCE = val
end

export is_within_capture_distance

"""
    is_within_capture_distance(parent,child,ttol=capture_distance_tolerance(),rtol=capture_rotation_tolerance())

Returns true if the error between `relative_transform(parent,child)` and
`child_transform(parent,node_id(child))` is small enough for capture of `child`
by parent.
"""
function is_within_capture_distance(parent::SceneNode, child::SceneNode, args...)
    t = relative_transform(global_transform(parent), global_transform(child))
    t_des = child_transform(parent, node_id(child))
    is_within_capture_distance(t, t_des, args...)
end

function is_within_capture_distance(
    t,
    t_des,
    ttol = capture_distance_tolerance(),
    rtol = capture_rotation_tolerance(),
)
    et = norm(t.translation - t_des.translation)  # translation error
    er = Inf
    # Rotation error
    # - QuatVecMap: cheapest to compute, but goes singular at 180 degrees  (sign ambiguity)
    # - MRPMap: singular at 360 degrees (sign ambiguity)
    er_quatvecmap = norm(Rotations.rotation_error(t, t_des, Rotations.QuatVecMap()))
    er_mrpmap = norm(Rotations.rotation_error(t, t_des, Rotations.MRPMap()))

    quatvecmap_singular = isnan(er_quatvecmap) || isinf(er_quatvecmap)
    mrpmap_singular = isnan(er_mrpmap) || isinf(er_mrpmap)
    @assert !(quatvecmap_singular && mrpmap_singular) "Both QuatVecMap and MRPMap are singular. This should not happen."
    if quatvecmap_singular
        er = er_mrpmap
    elseif mrpmap_singular
        er = er_quatvecmap
    else
        er = min(er_quatvecmap, er_mrpmap)
    end
    if et < ttol && er < rtol
        return true
    end
    return false
end

"""
    capture_child!(tree::SceneTree,u,v,ttol,rtol)

If the transform error of `v` relative to the transform prescribed for it by `u`
is small, allow `u` to capture `v` (`v` becomes a child of `u`).
"""
function capture_child!(
    tree::SceneTree,
    u,
    v,
    ttol = capture_distance_tolerance(),
    rtol = capture_rotation_tolerance(),
)
    nu = get_node(tree, u)
    nv = get_node(tree, v)
    @assert has_component(nu, node_id(nv)) "$nu cannot capture $nv"
    if is_within_capture_distance(get_node(tree, u), get_node(tree, v), ttol, rtol)
        if !is_root_node(tree, v)
            p = get_node(tree, get_parent(tree, v))  # current parent
            @assert(
                isa(p, TransportUnitNode),
                "Trying to capture child $v from non-TransportUnit parent $p"
            )
            # NOTE: There may be a time gap if the cargo has to be lifted into
            # place by a separate "robot"
            disband!(tree, p)
        end
        return set_child!(tree, u, v)
    end
    return false
end

function construct_scene_dependency_graph(scene_tree)
    G = Graphs.DiGraph(Graphs.nv(scene_tree))
    for n in get_nodes(scene_tree)
        for (child_id, _) in required_transforms_to_children(n)
            Graphs.add_edge!(G, get_vtx(scene_tree, n), get_vtx(scene_tree, child_id))
        end
    end
    return G
end

"""
    compute_approximate_geometries!(scene_tree,key=HypersphereKey(),base_key=key;
        leaves_only=false,ϵ=0.0)

Walk up the tree, computing bounding spheres (or whatever other geometry).
Args:
- key: determines the type of overapproximation geometry to compute
- base_key: determines the type of child geometry on which to base a parent's
    geometry
Example:
    compute_approximate_geometries!(tree,HyperrectangleKey(),BaseGeomKey()) will
    compute a Hyperrectangle approximation of each object and assembly. The
    computation for each assembly's Hyperrectangle will be based on the
    BaseGeomKey geometry of all of that assembly's children.
"""
function compute_approximate_geometries!(
    scene_tree,
    key = HypersphereKey(),
    base_key = key;
    leaves_only = false,
    ϵ = 0.0,
)
    # Build dependency graph (to enable full topological sort)
    G = construct_scene_dependency_graph(scene_tree)
    ids = (get_vtx_id(scene_tree, v) for v in reverse(Graphs.topological_sort_by_dfs(G)))
    for node in node_iterator(scene_tree, ids)
        h = node.geom_hierarchy
        if !Graphs.has_vertex(h, key)
            if isa(node, Union{ObjectNode,RobotNode})
                for (k1, k2) in [(BaseGeomKey(), base_key), (base_key, key)]
                    if !Graphs.has_vertex(h, k2)
                        base_geom = recurse_child_geometry(node, scene_tree, k1, 1)
                        if !(base_geom === nothing)
                            add_child_approximation!(h, k2, k1, base_geom)
                        end
                    end
                end
            elseif leaves_only == false
                geoms = recurse_child_geometry(node, scene_tree, base_key, 1)
                if !(geoms === nothing)
                    add_child_approximation!(h, key, BaseGeomKey(), geoms)
                else
                    @warn "Unable to recurse child geometry" node
                end
            end
        end
    end
    return scene_tree
end

export remove_geometry!

remove_geometry!(n::GeometryHierarchy, key) = rem_node!(n, key)
remove_geometry!(n::SceneNode, key) = rem_node!(n.geom_hierarchy, key)

function remove_geometry!(scene_tree::SceneTree, key)
    for n in get_nodes(scene_tree)
        remove_geometry!(n, key)
    end
end

"""
    jump_to_final_configuration!(scene_tree;respect_edges=false)

Jump to the final configuration state wherein all `ObjectNode`s and
`AssemblyNode`s are in the configurations specified by their parent assemblies.
"""
function jump_to_final_configuration!(scene_tree; respect_edges = false, set_edges = false)
    for node in get_nodes(scene_tree)
        if matches_template(AssemblyNode, node)
            for (id, tform) in assembly_components(node)
                force_remove_edge!(scene_tree, node, id)
                set_child!(scene_tree, node, id)
                if !set_edges
                    force_remove_edge!(scene_tree, node, id)
                end
            end
        end
    end
    scene_tree
end

"""
    project_point_to_line(p,p1,p2)

project `p` onto line between `p1` and `p2`
"""
function project_point_to_line(p, p1, p2)
    base = p2 - p1
    leg = p - p1
    v = p1 + dot(leg, base) * base / norm(base)^2
end

"""
    project_onto_vector(vec,b)

project `vec` onto `b`.
"""
project_onto_vector(vec, b) = b * dot(vec, b) / dot(b, b)

"""
"""
projection_norm(a, b) = dot(a, b) / norm(b)

"""
    circle_intersects_line(circle,line)

Return true if `circle` intersect `line`.
"""
function circle_intersection_with_line(circle, line)
    p1, p2 = line.points[1], line.points[2]
    c = get_center(circle)
    pt = project_point_to_line(c, p1, p2)
    d = norm(p2 - p1)
    d1 = norm(p1 - pt)
    d2 = norm(p2 - pt)
    r = get_radius(circle)
    if isapprox(d, d1 + d2)
        return norm(c - pt) - r
    else
        return min(norm(c - p1), norm(c - p2)) - r
    end
end

circle_intersects_line(args...) = circle_intersection_with_line(args...) < 0
circle_intersection_with_line(circle, pt1, pt2) = circle_intersection_with_line(
    circle,
    GeometryBasics.Line(GeometryBasics.Point(pt1...), GeometryBasics.Point(pt2...)),
)

"""
    extract_points_and_radii(lazy_set)

Returns an iterator over points and radii.
"""
extract_points_and_radii(n::GeometryBasics.Ngon) =
    zip(GeometryBasics.coordinates(n), Base.Iterators.repeated(0.0))

function extract_points_and_radii(n::GeometryBasics.Cylinder, c = 16)
    # Approximate, because there's no other way to account for the cylinder's flat top and bottom
    v = normalize(n.extremity - n.origin)
    θ_range = 0.0:(2π/c):(2π*(c-1)/c)
    extract_points_and_radii(
        Base.Iterators.flatten([
            map(
                θ -> SVector(
                    n.origin +
                    normalize(cross(SVector{3,Float64}(cos(θ), sin(θ), 0.0), v)) * n.r,
                ),
                θ_range,
            ),
            map(
                θ -> SVector(
                    n.extremity +
                    normalize(cross(SVector{3,Float64}(cos(θ), sin(θ), 0.0), v)) * n.r,
                ),
                θ_range,
            ),
        ]),
    )
end

extract_points_and_radii(n::LazySets.Ball2) = [(n.center, n.radius)]
extract_points_and_radii(n::Union{Hyperrectangle,AbstractPolytope}) =
    zip(LazySets.vertices(n), Base.Iterators.repeated(0.0))
for T in (:AbstractVector, :(Base.Iterators.Flatten), :(Base.Generator))
    @eval extract_points_and_radii(n::$T) =
        Base.Iterators.flatten(map(extract_points_and_radii, n))
end
extract_points_and_radii(n::SVector) = [(n, 0.0)]
LazySets.dim(::Type{SVector{N,T}}) where {N,T} = N
LazySets.dim(::Type{LazySets.Ball2{T,V}}) where {T,V} = LazySets.dim(V)
Base.convert(::Type{LazySets.Ball2{T,V}}, b::LazySets.Ball2) where {T,V} =
    LazySets.Ball2(V(b.center), T(b.radius))

"""
    overapproximate_sphere(points_and_radii,N=3,ϵ=0.0)

The bounding sphere ought to be constructed instead by iteratively adding points
and updating the sphere.
"""
function overapproximate_sphere(points_and_radii, N = 3, ϵ = 0.0)
    model = Model(default_geom_optimizer())
    set_optimizer_attributes(model, default_geom_optimizer_attributes()...)
    @variable(model, v[1:N])
    @variable(model, d >= 0.0)
    @objective(model, Min, d)
    list_is_empty = true
    for (pt, r) in points_and_radii
        list_is_empty = false
        @constraint(
            model,
            [(d - r - ϵ), map(i -> (v[i] - pt[i]), 1:N)...] in SecondOrderCone()
        )
    end
    if list_is_empty
        return LazySets.Ball2(SVector(zeros(N)...), 0.0)
    end
    optimize!(model)
    ball = LazySets.Ball2(SVector{N,Float64}(value.(v)), max(0.0, value(d)))
    return ball
end

function overapproximate_cylinder(points_and_radii, ϵ = 0.0)
    N = 2
    b = overapproximate_sphere(points_and_radii, N, ϵ)
    zhi = -Inf
    zlo = Inf
    for (pt, r) in points_and_radii
        if pt[3] > zhi
            zhi = pt[3]
        end
        if pt[3] < zlo
            zlo = pt[3]
        end
    end
    # Get z values
    return GeometryBasics.Cylinder(
        GeometryBasics.Point(b.center[1], b.center[2], zlo),
        GeometryBasics.Point(b.center[1], b.center[2], zhi),
        b.radius,
    )
end

for T in (:AbstractPolytope, :LazySet, :AbstractVector, :(GeometryBasics.Ngon), :Any)
    @eval begin
        function LazySets.overapproximate(
            lazy_set::$T,
            ::Type{H},
            ϵ::Float64 = 0.0,
            N = LazySets.dim(H),
        ) where {H<:LazySets.Ball2}
            return convert(
                H,
                overapproximate_sphere(extract_points_and_radii(lazy_set), N, ϵ),
            )
        end
        function LazySets.overapproximate(
            lazy_set::$T,
            ::Type{H},
            ϵ::Float64 = 0.0,
        ) where {A,V<:AbstractVector,H<:Hyperrectangle{A,V,V}}
            high = -Inf * ones(V)
            low = Inf * ones(V)
            for (pt, r) in extract_points_and_radii(lazy_set)
                high = max.(high, pt .+ r)
                low = min.(low, pt .- r)
            end
            ctr = (high .+ low) / 2
            widths = (high .- low) / 2
            if any(widths .< 0)
                ctr = zeros(V)
                widths = zeros(V)
            end
            Hyperrectangle(ctr, widths .+ (ϵ / 2))
        end
        function LazySets.overapproximate(
            lazy_set::$T,
            m::Type{C},
            ϵ::Float64 = 0.0,
        ) where {C<:GeometryBasics.Cylinder}
            overapproximate_cylinder(extract_points_and_radii(lazy_set), ϵ)
        end
    end
end

function LazySets.overapproximate(
    lazy_set::LazySets.Ball2,
    model::Type{H},
    ϵ::Float64 = 0.0,
) where {A,V<:AbstractVector,H<:Hyperrectangle{A,V,V}}
    ctr = lazy_set.center
    r = lazy_set.radius + ϵ
    Hyperrectangle(V(ctr), r * ones(V))
end

function LazySets.overapproximate(
    lazy_set,
    sphere::H,
    ϵ::Float64 = 0.0,
) where {V,T,H<:LazySets.Ball2{T,V}}
    r = 0.0
    for (pt, rad) in extract_points_and_radii(lazy_set)
        r = max(norm(pt - get_center(sphere)) + rad + ϵ, r)
    end
    LazySets.Ball2(V(get_center(sphere)), T(r))
end

"""
    construct_support_placement_aggregator

Construct an objective function that scores a selection of indices into `pts`.
Balances a neighbor-neighbor metric with a all-all metric
Args
* pts - a vector of points
* n - the number of indices to be selected
* [f_neighbor = v->1.0*minimum(v)] - a function mapping a vector of distances
    to a scalar value.
* [f_inner = v->1.0*minimum(v)] - a function mapping a vector of distances
    to a scalar value.
"""
function construct_support_placement_aggregator(
    pts,
    n,
    f_neighbor = v -> 1.0 * minimum(v) + (0.5 / n) * sum(v),
    f_inner = v -> (0.1 / (n^2)) * minimum(v),
)
    D = [norm(v - vp) for (v, vp) in Base.Iterators.product(pts, pts)]
    d_neighbor =
        (idxs) -> f_neighbor(
            map(i -> wrap_get(D, (idxs[i], wrap_get(idxs, i + 1))), 1:length(idxs)),
        )
    d_inner =
        (idxs) ->
            f_inner([wrap_get(D, (i, j)) for (i, j) in Base.Iterators.product(idxs, idxs)])
    d = (idxs) -> d_neighbor(idxs) + d_inner(idxs)
end

"""
    spaced_neighbors(polygon,n::Int,aggregator=sum)

Return the indices of the `n` vertices of `polygon` whose neighbor distances
maximize the utility metric defined by `aggregator`. Uses local optimization,
so there is no guarantee of global optimality.
"""
function spaced_neighbors(
    polygon::LazySets.AbstractPolygon,
    n::Int,
    score_function = construct_support_placement_aggregator(vertices_list(polygon), n),
    ϵ = 1e-8,
)
    pts = vertices_list(polygon)
    @assert length(pts) >= n "length(pts) = $(length(pts)), but n = $n"
    if length(pts) == n
        return collect(1:n), 0.0
    end
    best_idxs = SVector{n,Int}(collect(1:n)...)
    d_hi = score_function(best_idxs)
    idx_list = [best_idxs]
    while true
        updated = false
        for deltas in Base.Iterators.product(map(i -> (-1, 0, 1), 1:n)...)
            idxs = sort(map(i -> wrap_idx(length(pts), i), best_idxs .+ deltas))
            if length(unique(idxs)) == n
                if score_function(idxs) > d_hi + ϵ
                    best_idxs = map(i -> wrap_idx(length(pts), i), idxs)
                    d_hi = score_function(idxs)
                    push!(idx_list, best_idxs)
                    updated = true
                end
            end
        end
        if !updated
            break
        end
    end
    return best_idxs, d_hi
end

perimeter(pts) = sum(map(v -> norm(v[1] - v[2]), zip(pts[1:end-1], pts[2:end])))  # sum(map(i->norm(wrap_get(pts,(i,i+1))),1:length(pts)))
perimeter(p::LazySets.AbstractPolygon) = perimeter(vertices_list(p))

"""
    select_support_locations(polygon, r)

Select support locations where options are the vertices of `polygon` and the
robot radius is `r`.
"""
function select_support_locations(
    polygon::LazySets.AbstractPolygon,
    r;
    score_function_constructor = construct_support_placement_aggregator,
)
    n = select_num_robots(polygon, r)
    if n == 1
        sphere = overapproximate(polygon, LazySets.Ball2{Float64,SVector{2,Float64}})
        support_pts = [LazySets.center(sphere)]
        return support_pts
    end
    pts = vertices_list(polygon)
    pts = reduce_to_valid_candidate_list(pts, r)
    reduced_polygon = VPolygon(pts)
    # The desired number of robots can't be more than the number of valid points

    # We also use the vertices list of the reduced polygon so we are only considering the
    # convex hull of the valid points``
    n = min(n, length(vertices_list(reduced_polygon)))  # cannot be greater than the number of valid points
    if n == 1
        sphere = overapproximate(polygon, LazySets.Ball2{Float64,SVector{2,Float64}})
        support_pts = [LazySets.center(sphere)]
        return support_pts
    end
    score_function = score_function_constructor(pts, n)
    best_idxs, _ = spaced_neighbors(reduced_polygon, n, score_function)
    if length(best_idxs) < n
        @warn "$n support points requested, but only $(length(best_idxs)) returned for polygon with $(length(pts)) vertices"
    end
    support_pts = pts[best_idxs]
    return support_pts
end

"""
    reduce_to_valid_candidate_list(pts, r; ϵ=0.5*r)

Reduce the list of candidate points to those that are at least `2r+ϵ` away from each other.
This selects points that are "far" away from each other.
"""
function reduce_to_valid_candidate_list(pts, r; ϵ = 0.5 * r)
    pts_c = deepcopy(pts)
    candidate_pts = Vector{Vector{Float64}}()
    pts_dist = norm.(pts_c, 2)  # the first candidate point is the point furthest from the origin
    first_candidate_idx = argmax(pts_dist)
    push!(candidate_pts, pts_c[first_candidate_idx])
    deleteat!(pts_c, first_candidate_idx)
    while !isempty(pts_c)
        dist_to_candidates = Matrix{Float64}(undef, length(candidate_pts), length(pts_c))
        lp_dist_to_candidates = Matrix{Float64}(undef, length(candidate_pts), length(pts_c))
        for (ii, c_pt) in enumerate(candidate_pts), (jj, pt) in enumerate(pts_c)
            dist_to_candidates[ii, jj] = norm(c_pt - pt, 2)
            lp_dist_to_candidates[ii, jj] = norm(c_pt - pt, 2)
        end
        # Determine feasible points
        min_dist_to_candidates = minimum(dist_to_candidates, dims = 1)
        bool_mask = vec(min_dist_to_candidates .> (2 * r + ϵ))
        pts_c = pts_c[bool_mask]

        if isempty(pts_c)
            break
        end
        # Select next candidate: feasible and furthest sum lp norm from candidates
        sum_lp_dist = vec(sum(lp_dist_to_candidates, dims = 1))
        sum_lp_dist = sum_lp_dist[bool_mask]
        next_candidate_idx = argmax(sum_lp_dist)
        push!(candidate_pts, pts_c[next_candidate_idx])
        deleteat!(pts_c, next_candidate_idx)
    end
    @assert !isempty(candidate_pts) "Starting point should always be an option"
    return candidate_pts
end

"""
    select_num_robots(polygon,r)

polygon: the polygon whose vertices are eligible locations for placing robots
r: the robot radius
"""
function select_num_robots(polygon, r)
    # Extract length and width from spectral value decomposition
    pts = vertices_list(polygon)
    _, S, _ = svd(hcat(convert.(Vector, pts)...))
    L = S[1]  # length
    W = S[2]  # width
    neighbor_dists = map(i -> norm(pts[i] - pts[i+1]), 1:length(pts)-1)
    push!(neighbor_dists, norm(pts[end] - pts[1]))
    N = length(pts) - length(findall(neighbor_dists .< 2 * r))
    p = perimeter(polygon)
    x = p / (π * r)
    if W >= 2 * r
        n = Int(floor(max(1, min(N, min(x, 2 * sqrt(x))))))
    else  # long and skinny
        n = Int(floor(max(1, min(x, 2))))
    end
    @info "number of robots in transport unit: $(n)"
    return n
end

"""
    compute_hierarchical_2d_convex_hulls(scene_tree)

Compute the 2d projected convex hull of each `ObjectNode` and each
`AssemblyNode` in scene_tree.
"""
function compute_hierarchical_2d_convex_hulls(scene_tree)
    cvx_hulls = Dict{AbstractID,Vector{GeometryBasics.Point{2,Float64}}}()
    for v in reverse(Graphs.topological_sort_by_dfs(scene_tree))
        node = get_node(scene_tree, v)
        if matches_template(ObjectNode, node)
            pts = map(project_to_2d, GeometryBasics.coordinates(get_base_geom(node)))
            cvx_hulls[node_id(node)] = convex_hull(pts)
        elseif matches_template(AssemblyNode, node)
            pts = nothing
            for (child_id, tform) in assembly_components(node)
                composite_tform = project_to_2d ∘ tform ∘ project_to_3d
                child_pts = map(composite_tform, cvx_hulls[child_id])
                if pts === nothing
                    pts = child_pts
                else
                    append!(pts, child_pts)
                end
            end
            cvx_hulls[node_id(node)] = convex_hull(pts)
        end
    end
    cvx_hulls
end
