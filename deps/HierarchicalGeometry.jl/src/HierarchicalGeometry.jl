module HierarchicalGeometry

using StaticArrays
using LinearAlgebra

using GraphUtils
using Parameters
using JuMP
using ECOS
using MathOptInterface
using Logging
using Reexport

import Rotations
import CoordinateTransformations
import LazySets
import Graphs
import GeometryBasics
import Polyhedra

@reexport using LazySets

# TODO convert RigidBodyDynamics.CartesianFrame3D to CoordinateTransformations.Transformation
# TODO add RigidBodyDynamics.Mechanism(s) to SceneTree
# TODO replace TransformNode with RigidBodyDynamics equivalent. Each SceneNode should have a RigidBody included in it.


include("JuMP_interface.jl")
set_default_optimizer!(ECOS.Optimizer)
set_default_optimizer_attributes!(MOI.Silent()=>true)

export
    transform,
    transform!,
    identity_linear_map,
    scaled_linear_map,
    distance_lower_bound,
    has_overlap

const BaseGeometry = Union{Ball2,Hyperrectangle,AbstractPolytope}
const BallType = Ball2
get_center(s::Ball2) = LazySets.center(s)
get_radius(s::Ball2) = s.radius
get_center(s::GeometryBasics.HyperSphere) = s.center
get_radius(s::GeometryBasics.HyperSphere) = s.r
GeometryBasics.Sphere(s::Ball2) = GeometryBasics.Sphere(GeometryBasics.Point(s.center...),s.radius)
Base.convert(::Type{S},s::Ball2) where {S<:GeometryBasics.HyperSphere} = S(GeometryBasics.Point(s.center...),s.radius)
const RectType = Hyperrectangle
get_center(s::Hyperrectangle) = s.center
get_radius(s::Hyperrectangle) = s.radius
GeometryBasics.HyperRectangle(s::Hyperrectangle) = GeometryBasics.HyperRectangle(GeometryBasics.Vec((s.center .- s.radius)...),2*GeometryBasics.Vec(s.radius...))

"""
    centroid(pts::Vector{V}) where {V<:AbstractVector}

Compute the centroid of the polygon whose vertices are defined (in ccw order) by
`pts`.
"""
function centroid(pts::Vector{V}) where {V<:AbstractVector}
    p1 = pts[1]
    total_area = 0.0
    c = zero(V)
    for (p2,p3) in zip(pts[2:end-1],pts[3:end])
        a = GeometryBasics.area([p1,p2,p3])
        total_area += a
        c = ((total_area-a)/total_area)*c .+ (a/total_area) * (p1 .+ p2 .+ p3) / 3
    end
    c
end

export
    default_robot_geom,
    default_robot_radius,
    default_robot_height,
    set_default_robot_geom!

global DEFAULT_ROBOT_GEOM = GeometryBasics.Cylinder(
    GeometryBasics.Point{3,Float64}(0.0,0.0,0.0),
    GeometryBasics.Point{3,Float64}(0.0,0.0,0.25),
    0.5
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
    if isa(geom,GeometryBasics.Cylinder)
        return geom.r
    else
        return 0.25
    end
end
function default_robot_height()
    geom = default_robot_geom()
    if isa(geom,GeometryBasics.Cylinder)
        return norm(geom.origin-geom.extremity)
    else
        return 0.1
    end
end


const Z_PROJECTION_MAT = SMatrix{2,3,Float64}(1.0,0.0,0.0,1.0,0.0,0.0)

"""
    project_to_2d(geom,t=CoordinateTransformations.LinearMap(Z_PROJECTION_MAT)) = t(geom)
"""
project_to_2d(geom,t=CoordinateTransformations.LinearMap(Z_PROJECTION_MAT)) = t(geom)

"""
    project_to_3d(geom,t=CoordinateTransformations.LinearMap(transpose(Z_PROJECTION_MAT))) = t(geom)
"""
project_to_3d(geom,t=CoordinateTransformations.LinearMap(transpose(Z_PROJECTION_MAT))) = t(geom)

function project_rotation_to_XY(rot_mat)
    z = rot_mat[:,3] # z column
    rb = rotation_between(SVector(0.0,0.0,1.0),z)
    rot_z = inv(rb) * rot_mat
end

include("overapproximation.jl")

"""
    transform(geom,t)

Transform geometry `geom` according to the transformation `t`.
TODO: It is important for all base geometries to be defined with respect to
their own origin.
"""
transform(v,t) = t(v)
(t::CoordinateTransformations.Translation)(g::BaseGeometry) = LazySets.translate(g,Vector(t.translation))

"""
    (t::CoordinateTransformations.LinearMap)(g::Hyperrectangle)

"rotating" a Hyperrectangle `g` results in a new Hyperrectangle that bounds the
transformed version `g`.
"""
(t::CoordinateTransformations.LinearMap)(g::Hyperrectangle) = overapproximate(t(convert(LazySets.VPolytope,g)),Hyperrectangle)
(t::CoordinateTransformations.AffineMap)(g::Hyperrectangle) = overapproximate(t(convert(LazySets.VPolytope,g)),Hyperrectangle)
(t::CoordinateTransformations.Translation)(g::Hyperrectangle) = Hyperrectangle(t(g.center),g.radius)

# (t::AffineMap)(g::G) where {G<:GeometryBasics.Ngon} defined in LDrawParser

for T in (
        :(CoordinateTransformations.AffineMap),
    )
    @eval begin
        (t::$T)(v::V) where {N<:GeometryBasics.Ngon,V<:AbstractVector{N}} = V(map(t,v))
        (t::$T)(g::C) where {C<:GeometryBasics.Cylinder} = C(t(g.origin),t(g.extremity),g.r)
        (t::$T)(g::VPolytope) = VPolytope(map(t, vertices_list(g)))
        (t::$T)(g::HPolytope) = HPolytope(map(t, constraints_list(g)))
        (t::$T)(g::VPolygon) = VPolytope(map(t, vertices_list(g)))
        (t::$T)(g::HPolygon) = HPolytope(map(t, constraints_list(g)))
        (t::$T)(g::BufferedPolygon) = BufferedPolygon(map(t,g.halfspaces),map(t,g.pts),g.min_face_length)
        (t::$T)(g::BufferedPolygonPrism) = BufferedPolygonPrism(t(g.p),t(g.origin),t(g.extremity))
        (t::$T)(::Nothing) = nothing
        (t::$T)(g::Ball2) = Ball2(t(g.center),g.radius)
    end
end

for T in (
        :(CoordinateTransformations.LinearMap),
        :(CoordinateTransformations.Translation),
    )
    @eval begin
        (t::$T)(v::V) where {N<:GeometryBasics.Ngon,V<:AbstractVector{N}} = V(map(t,v))
        (t::$T)(g::G) where {G<:GeometryBasics.Ngon} = G(map(t,g.points))
        (t::$T)(g::C) where {C<:GeometryBasics.Cylinder} = C(t(g.origin),t(g.extremity),g.r)
        (t::$T)(g::VPolytope) = VPolytope(map(t, vertices_list(g)))
        (t::$T)(g::HPolytope) = HPolytope(map(t, constraints_list(g)))
        (t::$T)(g::VPolygon) = VPolytope(map(t, vertices_list(g)))
        (t::$T)(g::HPolygon) = HPolytope(map(t, constraints_list(g)))
        (t::$T)(g::BufferedPolygon) = BufferedPolygon(map(t,g.halfspaces),map(t,g.pts),g.min_face_length)
        (t::$T)(g::BufferedPolygonPrism) = BufferedPolygonPrism(t(g.p),t(g.origin),t(g.extremity))
        (t::$T)(::Nothing) = nothing
        (t::$T)(g::Ball2) = Ball2(t(g.center),g.radius)
    end
end
(t::CoordinateTransformations.Translation)(h::LazySets.HalfSpace) = LazySets.HalfSpace(h.a,h.b+dot(t.translation,h.a))
(t::CoordinateTransformations.LinearMap)(h::LazySets.HalfSpace) = LazySets.HalfSpace(t(Vector(h.a)),h.b)
# for N in (:(GeometryBasics.Point{2,Float64}),:(SVector{2,Float64}),)
#     for M in (:(SMatrix{3,3,Float64}),:(Rotation{3,Float64}),)
#         @eval (t::CoordinateTransformations.LinearMap{$M})(p::$N) = $N(t.linear[1:2,1:2]*p)
#     end
#     @eval (t::CoordinateTransformations.Translation)(p::$N) = $N(t.translation[1:2]*p)
# end

identity_linear_map3() = CoordinateTransformations.compose(CoordinateTransformations.Translation(zero(SVector{3,Float64})),CoordinateTransformations.LinearMap(one(SMatrix{3,3,Float64})))
identity_linear_map2() = CoordinateTransformations.compose(CoordinateTransformations.Translation(zero(SVector{3,Float64})),CoordinateTransformations.LinearMap(one(SMatrix{3,3,Float64})))
identity_linear_map() = identity_linear_map3()
scaled_linear_map(scale) = CoordinateTransformations.LinearMap(scale*one(SMatrix{3,3,Float64})) ∘ identity_linear_map()
Base.convert(::Type{Hyperrectangle{Float64,T,T}},rect::Hyperrectangle) where {T} = Hyperrectangle(T(rect.center),T(rect.radius))

export relative_transform
"""
    relative_transform(a::AffineMap,b::AffineMap)

compute the relative transform `t` between two frames, `a`, and `b` such that
    t = inv(a) ∘ b
    b = a ∘ t
    a = b ∘ inv(t)

    i.e. (a ∘ t)(p) == a(t(p)) == b(p)
"""
relative_transform(a::A,b::B) where {A<:CoordinateTransformations.Transformation,B<:CoordinateTransformations.Transformation} = inv(a) ∘ b
# function relative_transform(a::CoordinateTransformations.AffineMap,b::CoordinateTransformations.AffineMap,error_map=MRPMap())
#     t = inv(a) ∘ b
#     # pos_err = CoordinateTransformations.Translation(b.translation - a.translation)
#     # rot_err = Rotations.rotation_error(a,b,error_map)
#     # q = Rotations.QuatRotation(rot_err)
#     # t = pos_err ∘ CoordinateTransformations.LinearMap(q) # IMPORTANT: rotation on right side
# end
function Rotations.rotation_error(
    a::CoordinateTransformations.AffineMap,
    b::CoordinateTransformations.AffineMap,
    error_map=MRPMap())
    rot_err = Rotations.rotation_error(
        Rotations.QuatRotation(a.linear),
        Rotations.QuatRotation(b.linear),error_map)
end
for op in (:relative_transform,:(Rotations.rotation_error))
    @eval $op(tree::AbstractCustomTree,parent,child,args...) = $op(
        global_transform(tree,parent),
        global_transform(tree,child),
        args...
    )
    @eval $op(parent,child,args...) = $op(
        global_transform(parent),
        global_transform(child),
        args...
    )
end

"""
    interpolate_rotation(Ra,Rb,c=0.5)

compute a rotation matrix `ΔR` such that `Ra*ΔR` is c/1.0 of the way to `Rb`.
    R = R⁻¹ Rb
    θ = axis + magnitude representation
    ΔR = exp(cross_product_operator(θ)*c)
"""
function interpolate_rotation(Ra,Rb,c=0.5)
    R = inv(Ra) * Rb
    r = Rotations.RotationVec(R) # rotation vector
    Rotations.RotationVec(r.sx*c,r.sy*c,r.sz*c)
end
function interpolate_transforms(Ta,Tb,c=0.5)
    R = interpolate_rotation(Ta.linear,Tb.linear,c)
    t = (1-c)*Ta.translation + c*Tb.translation
    CoordinateTransformations.Translation(t) ∘ CoordinateTransformations.LinearMap(R)
end

export
    distance_lower_bound,
    has_overlap

LazySets.ρ(a::AbstractArray,p::GeometryBasics.Point) = ρ(a,Singleton([p.data...]))
LazySets.ρ(a::AbstractArray,v::AbstractArray{V,1}) where {V<:GeometryBasics.Point} = minimum(map(x->ρ(a,x),v))
LazySets.ρ(a::AbstractArray,x::GeometryBasics.Ngon) = ρ(a,GeometryBasics.coordinates(x))
# Distance between sets
LazySets.distance(a::BallType,b::BallType,p::Real=2.0) = norm(get_center(a)-get_center(b),p) - (get_radius(a)+get_radius(b))
function LazySets.distance(a::Hyperrectangle,b::Hyperrectangle,p::Real=2.0)
    m = minkowski_sum(a,b)
    d = map(h->dot(h.a,get_center(a))-h.b, constraints_list(m))
    sort!(d;rev=true)
    idx = findlast(d .> 0.0)
    if idx === nothing
        return d[1]
    else
        return norm(d[1:idx])
    end
end
distance_lower_bound(a::BallType,b::BallType) = LazySets.distance(a,b)
distance_lower_bound(a::Hyperrectangle,b::Hyperrectangle) = LazySets.distance(a,b)
has_overlap(a::BaseGeometry,b::BaseGeometry) = !is_intersection_empty(a,b) # distance_lower_bound(a,b) <= 0.0

export GeometryCollection
struct GeometryCollection{G}
    elements::Vector{G}
end
for op in [:distance_lower_bound,:(LazySets.distance)]
    @eval $op(a::GeometryCollection,b) = minimum($op(x,b) for x in a.elements)
    @eval $op(a::GeometryCollection,b::GeometryCollection) = minimum($op(a,y) for y in b.elements)
end

"""
    check_collision(a,b)

Recursive collision checking
"""
function check_collision(a,b)
    if has_overlap(a,b)
        for p in assembly_components(b)
            if check_collision(p,a)
                return true
            end
        end
    end
    return false
end

const cached_element_accessor_interface = [
    :(GraphUtils.is_up_to_date),
    :(GraphUtils.time_stamp)
    ]
const cached_element_mutator_interface = [
    :(GraphUtils.update_element!),
    :(GraphUtils.set_time_stamp!),
    :(GraphUtils.set_element!),
    :(GraphUtils.set_up_to_date!)
    ]


export
    TransformNode,
    local_transform,
    global_transform,
    set_local_transform!,
    set_global_transform!

@with_kw struct TransformNodeID <: AbstractID
    id::Int = -1
end

"""
    TransformNode

Has a parent field that points to another TransformNode.
"""
mutable struct TransformNode <: CachedTreeNode{TransformNodeID}
    id::TransformNodeID
    local_transform::CoordinateTransformations.Transformation # transform from the parent frame to the child frame
    global_transform::CachedElement{CoordinateTransformations.Transformation}
    parent::TransformNode # parent node
    children::Dict{AbstractID,CachedTreeNode}
    function TransformNode(
            a::CoordinateTransformations.Transformation,
            b::CachedElement)
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
    TransformNode(identity_linear_map(),CachedElement(identity_linear_map()))
end
function TransformNode(
    a::CoordinateTransformations.Transformation,
    b::CoordinateTransformations.Transformation
    )
    TransformNode(a,CachedElement(b))
end
# TODO copying as currently implemented will cause major issues with
function Base.copy(n::TransformNode)
    node = TransformNode(
    deepcopy(n.local_transform),
    deepcopy(n.global_transform)
    )
    # Don't want to copy parent or children. Just need to reconnect later.
    return node
end

Base.show(io::IO, ::MIME"text/plain", m::TransformNode) = print(
    io, "TransformNode(",get_id(node_id(m)),")\n",
        "  local:  ",local_transform(m), "\n",
        "  global: ",global_transform(m), "\n",
        "  parent: ",string(node_id(m.parent)),"\n",
        "  children: ",map(k->string("\n    ",string(k)),collect(keys(get_children(m))))...,"\n",
        )
Base.show(io::IO, m::TransformNode) = print(io, "TransformNode(",get_id(node_id(m)),") - ",m.global_transform)

# Cached Tree interface
GraphUtils.cached_element(n::TransformNode) = n.global_transform
tf_up_to_date(n::TransformNode) = GraphUtils.cached_node_up_to_date(n)
set_tf_up_to_date!(n::TransformNode,val=true) = GraphUtils.set_cached_node_up_to_date!(n,val)
local_transform(n::TransformNode) = n.local_transform
set_global_transform!(n::TransformNode,t,args...) = GraphUtils.update_element!(n,t,args...)
global_transform(n::TransformNode) = GraphUtils.get_cached_value!(n)
function GraphUtils.propagate_forward!(parent::TransformNode,child::TransformNode)
    if parent === child
        set_global_transform!(child,local_transform(child))
    else
        set_global_transform!(child,global_transform(parent) ∘ local_transform(child))
    end
    global_transform(child)
end
function set_local_transform!(n::TransformNode,t,update=false)
    n.local_transform = t ∘ identity_linear_map() # guarantee AffineMap?
    set_tf_up_to_date!(n,false)
    if update
        global_transform(n) # just get the global transform
    end
    return n.local_transform
end
function set_local_transform_in_global_frame!(n::TransformNode,t,args...)
    rot_mat = CoordinateTransformations.LinearMap(global_transform(get_parent(n)).linear)
    set_local_transform!(n, inv(rot_mat) ∘ t )
end

export set_desired_global_transform!
"""
    set_desired_global_transform!(n::TransformNode,t,args...)

Set a transform node's global local transform such that its global transform
(respecting it's current parent) will be `t`.
"""
function set_desired_global_transform!(n::TransformNode,t,args...)
    if has_parent(n,n)
        set_local_transform!(n,t)
    else
        parent = get_parent(n)
        tform = relative_transform(global_transform(parent),t)
        set_local_transform!(n,tform)
    end
end
const transform_node_accessor_interface = [:tf_up_to_date,:local_transform,:global_transform]
const transform_node_mutator_interface = [:set_tf_up_to_date!,:set_local_transform!,:set_global_transform!,:set_local_transform_in_global_frame!,:set_desired_global_transform!]

export set_desired_global_transform_without_affecting_children!
"""
    set_desired_global_transform_without_affecting_children!()
"""
function set_desired_global_transform_without_affecting_children!(n::TransformNode,t,args...)
    tf_dict = Dict{TransformNodeID,CoordinateTransformations.AffineMap}()
    child_dict = get_children(n)
    for (id,c) in child_dict
        tf_dict[id] = global_transform(c)
    end
    set_desired_global_transform!(n,t,args...)
    for (id,tf) in tf_dict
        set_desired_global_transform!(child_dict[id],tf)
    end
    global_transform(n)
end

"""
    rem_parent!(child::TransformNode)

Ensure that the detached child does not "jump" in the global frame when detached
from the parent.
"""
function GraphUtils.rem_parent!(child::TransformNode)
    tf = global_transform(child)
    delete!(get_children(get_parent(child)), node_id(child))
    set_local_transform!(child,tf)
    child.parent = child
end

for op in transform_node_accessor_interface
    @eval $op(tree::AbstractCustomTree,v) = $op(get_node(tree,v))
    @eval $op(tree::AbstractCustomTree,n::TransformNode) = $op(n)
end
for op in transform_node_mutator_interface
    @eval $op(tree::AbstractCustomTree,v,args...) = $op(get_node(tree,v),args...)
    @eval $op(tree::AbstractCustomTree,n::TransformNode,args...) = $op(n,args...)
end

export
    set_child!,
    get_parent_transform,
    update_transform_tree!

"""
    set_child!(tree,parent,child,new_local_transform)

Replace existing edge `old_parent` → `child` with new edge `parent` → `child`.
Set the `child.local_transform` to `new_local_transform`.
"""
function set_child!(tree::AbstractCustomTree,parent,child,
        t=relative_transform(tree,parent,child),
        edge=nothing
    )
    Graphs.rem_edge!(tree,get_parent(tree,child),child)
    if Graphs.add_edge!(tree,parent,child,edge)
        set_parent!(get_node(tree,child),get_node(tree,parent)) # for TransformNode
        @assert !Graphs.is_cyclic(tree) "adding edge $(parent) → $(child) made tree cyclic"
        set_local_transform!(tree,child,t)
        return true
    end
    return false
end
const transform_tree_mutator_interface = [:set_local_transform!,:set_global_transform!]
function GraphUtils.add_child!(tree::AbstractCustomTree,parent,child,child_id)
    n = add_node!(tree,child,child_id)
    if set_child!(tree,get_node(tree,parent),child_id)
        return n
    else
        rem_node!(tree,child_id)
        return nothing
    end
end

export
    GeomNode,
    get_base_geom,
    get_cached_geom

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
    GeomNode(get_unique_id(GeomID),geom,TransformNode(),CachedElement(geom))
end
function GeomNode(geom,tf)
    GeomNode(get_unique_id(GeomID),geom,tf,CachedElement(geom))
end
GraphUtils.get_children(n::GeomNode) = Dict{AbstractID,CachedTreeNode}()
GraphUtils.cached_element(n::GeomNode) = n.cached_geom
set_cached_geom!(n::GeomNode,geom) = update_element!(n,geom)

get_base_geom(n::GeomNode) = n.base_geom
get_transform_node(n::GeomNode) = n.parent
for op in transform_node_accessor_interface
    @eval $op(g::GeomNode,args...) = $op(get_transform_node(g))
end
for op in transform_node_mutator_interface
    @eval $op(g::GeomNode,args...) = $op(get_transform_node(g),args...)
end
GraphUtils.set_parent!(a::GeomNode,b::TransformNode) = set_parent!(get_transform_node(a),b)
GraphUtils.set_parent!(a::GeomNode,b::GeomNode) = set_parent!(a,get_transform_node(b))
function GraphUtils.propagate_forward!(t::TransformNode,n::GeomNode)
    transformed_geom = transform(get_base_geom(n),global_transform(n))
    set_cached_geom!(n,transformed_geom)
end


"""
    get_cached_geom(n::GeomNode)

If `n.cached_geom` is out of date, transform it according to
`global_transform(n)`. Updates both the global transform and geometry if
necessary.
"""
get_cached_geom(n::GeomNode) = GraphUtils.get_cached_value!(n)
get_cached_geom(tree::AbstractCustomTree,v) = get_cached_geom(get_node(tree,v))
distance_lower_bound(a::GeomNode{G},b::GeomNode{G}) where {G<:Union{BallType,RectType}} = distance_lower_bound(get_cached_geom(a),get_cached_geom(b))
const geom_node_accessor_interface = [
    transform_node_accessor_interface...,
    :get_base_geom, :get_cached_geom, :get_transform_node
    ]
const geom_node_mutator_interface = [
    transform_node_mutator_interface...
]

"""
    Base.copy(n::GeomNode)

Shares `n.base_geom`, deepcopies `n.transform_node`, and copies `n.cached_geom`
(see documenation for `Base.copy(::CachedElement)`).
"""
Base.copy(n::GeomNode) = GeomNode(
    get_unique_id(GeomID),
    n.base_geom,
    copy(get_transform_node(n)),
    copy(n.cached_geom)
)


export
    GeometryHierarchy,
    BaseGeomKey,
    PolyhedronKey,
    PolygonKey,
    ZonotopeKey,
    HyperrectangleKey,
    HypersphereKey,
    CylinderKey,
    OctagonalPrismKey,
    CircleKey

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

construct_child_approximation(::PolyhedronKey,geom,args...)     = LazySets.overapproximate(geom,equatorial_overapprox_model(),args...)
construct_child_approximation(::HypersphereKey,geom,args...)    = LazySets.overapproximate(geom,Ball2{Float64,SVector{3,Float64}},args...)
construct_child_approximation(::HyperrectangleKey,geom,args...) = LazySets.overapproximate(geom,Hyperrectangle{Float64,SVector{3,Float64},SVector{3,Float64}},args...)
construct_child_approximation(::PolygonKey,geom,args...)        = LazySets.overapproximate(geom,ngon_overapprox_model(8),args...)
construct_child_approximation(::CircleKey,geom,args...)         = LazySets.overapproximate(geom,Ball2{Float64,SVector{2,Float64}},args...)
construct_child_approximation(::CylinderKey,geom,args...)       = LazySets.overapproximate(geom,GeometryBasics.Cylinder,args...)
construct_child_approximation(::OctagonalPrismKey,geom,args...) = LazySets.overapproximate(geom,BufferedPolygonPrism(regular_buffered_polygon(8,1.0;buffer=0.05*default_robot_radius())),args...)

"""
    GeometryHierarchy

A hierarchical representation of geometry
Fields:
* graph - encodes the hierarchy of geometries
* nodes - geometry nodes
"""
@with_kw struct GeometryHierarchy <: AbstractCustomNTree{GeomNode,GeometryKey}
    graph               ::Graphs.DiGraph               = Graphs.DiGraph()
    nodes               ::Vector{GeomNode}      = Vector{GeomNode}()
    vtx_map             ::Dict{GeometryKey,Int} = Dict{GeometryKey,Int}()
    vtx_ids             ::Vector{GeometryKey}   = Vector{GeometryKey}() # maps vertex uid to actual graph node
end
function geom_hierarchy(geom::GeomNode)
    h = GeometryHierarchy()
    add_node!(h,geom,BaseGeomKey())
    return h
end
for op in (:get_base_geom,:get_cached_geom)
    @eval begin
        function $op(n::GeometryHierarchy,k=BaseGeomKey())
            if Graphs.has_vertex(n,k)
                return $op(get_node(n,k))
            else
                return nothing
            end
        end
    end
end

distance_lower_bound(a::GeometryHierarchy,b::GeometryHierarchy) = distance_lower_bound(
    get_node(a,HypersphereKey()),
    get_node(b,HypersphereKey())
    )
function has_overlap(a::GeometryHierarchy,b::GeometryHierarchy,leaf_id=HypersphereKey())
    v = get_vtx(a,leaf_id)
    while Grahas_vertex(a,v)
        k = get_vtx_id(a,v)
        if Graphs.has_vertex(b,k)
            if !has_overlap(get_node(a,k),get_node(b,k))
                return false
            end
        end
        v = get_parent(a,k)
    end
    return true
end

export add_child_approximation!
"""
    add_child_approximation!(g::GeometryHierarchy, child_key, parent_key,
        base_geom=get_base_geom(g,parent_id), args...)

Overapproximate `g`'s type `parent_key` geometry with geometry of type
`child_key`, and add this new approximation to `g` under key `child_key` with an
edge from `parent_key` to `child_key`.
"""
function add_child_approximation!(g::GeometryHierarchy,
        child_id,
        parent_id,
        base_geom=get_base_geom(g,parent_id),
        args...
        )
    @assert Graphs.has_vertex(g,parent_id) "g does not have parent_id = $parent_id"
    @assert !Graphs.has_vertex(g,child_id) "g already has child_id = $child_id"
    node = get_node(g,parent_id)
    geom = construct_child_approximation(child_id,base_geom,args...)
    add_node!(g,
        GeomNode(geom,node.parent), # Share parent
        child_id
        ) # todo needs parent
    Graphs.add_edge!(g,parent_id,child_id)
    return g
end
add_child_approximation!(g,child_id) = add_child_approximation!(g,child_id,BaseGeomKey())

export construct_geometry_tree!
"""

TODO: Ensure that the parent member of the base geometry key is the parent
of the SceneNode to which the Geometry Hierarchy is bound.
"""
function construct_geometry_tree!(g::GeometryHierarchy,geom::GeomNode)
    add_node!(g,geom,BaseGeomKey())
    add_child_approximation!(g,PolyhedronKey(),BaseGeomKey())
    add_child_approximation!(g,HyperrectangleKey(),PolyhedronKey())
    add_child_approximation!(g,HypersphereKey(),PolyhedronKey())
end
construct_geometry_tree!(g::GeometryHierarchy,geom) = construct_geometry_tree!(g,GeomNode(geom))



export
    TransformDict,
    AssemblyID,
    TransportUnitID,
    SceneNode,
    RobotNode,
    ObjectNode,
    AssemblyNode,
        assembly_components,
        num_components,
        has_component,
        add_component!,
        child_transform,
    TransportUnitNode,
        robot_team,
        add_robot!,
        remove_robot!,
        cargo_id,
        cargo_type,
        is_in_formation,
    SceneTree,
        capture_child!,
        capture_robots!,
        disband!

const TransformDict{T} = Dict{T,CoordinateTransformations.Transformation}
"""
    abstract type SceneNode end

An Abstract type, of which all nodes in a SceneTree are concrete subtypes.
"""
@with_kw struct AssemblyID <: AbstractID
    id::Int = -1
end

"""
    abstract type SceneNode

A node of the `SceneTree`. Each concrete subtype of `SceneNode`
contains all of the following information:
- All required children (whether for a temporary or permanent edge)
- Required Parent (For a permanent edge only)
- Geometry of the entity represented by the node
- Unique ID accessible via `GraphUtils.node_id(node)`
- Current transform--both global (relative to base frame) and local (relative to
current parent)
- Required transform relative to parent (if applicable)

"""
abstract type SceneNode end
abstract type SceneAssemblyNode <: SceneNode end
# SceneNode interface
GraphUtils.node_id(n::SceneNode) = n.id

for op in geom_node_accessor_interface
    @eval $op(n::SceneNode) = $op(n.geom)
    @eval $op(n::CustomNode) = $op(node_val(n))
end
for op in geom_node_mutator_interface
    @eval $op(n::SceneNode,args...) = $op(n.geom,args...)
    @eval $op(n::CustomNode,args...) = $op(node_val(n),args...)
end
GraphUtils.set_parent!(a::SceneNode,b::SceneNode) = set_parent!(a.geom,b.geom)
GraphUtils.set_parent!(a::CustomNode,b::CustomNode) = set_parent!(node_val(a),node_val(b))
GraphUtils.rem_parent!(a::GeomNode) = rem_parent!(get_transform_node(a))
GraphUtils.rem_parent!(a::SceneNode) = rem_parent!(a.geom)
GraphUtils.rem_parent!(a::CustomNode) = rem_parent!(node_val(a))
for op in (:(GraphUtils.has_parent),:(GraphUtils.has_child),:(GraphUtils.has_descendant))
    @eval begin
        $op(a::SceneNode,b::SceneNode) = $op(get_transform_node(a),get_transform_node(b))
    end
end

get_cached_geom(n::SceneNode,k::GeometryKey) = get_cached_geom(n.geom_hierarchy,k)
get_base_geom(n::SceneNode,k::GeometryKey) = get_base_geom(n.geom_hierarchy,k)
function add_child_approximation!(n::SceneNode,args...)
    add_child_approximation!(n.geom_hierarchy,args...)
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
Base.copy(n::N) where {N<:SceneNode} = N(n,copy(n.geom))

struct RobotNode{R} <: SceneNode
    id::BotID{R}
    geom::GeomNode
    geom_hierarchy::GeometryHierarchy
end
RobotNode(id::BotID,geom) = RobotNode(id,geom,geom_hierarchy(geom))
RobotNode(n::RobotNode,geom) = RobotNode(n.id,geom)
RobotNode(id::BotID,n::RobotNode) = RobotNode(id,n.geom,n.geom_hierarchy)

struct ObjectNode <: SceneNode
    id::ObjectID
    geom::GeomNode
    geom_hierarchy::GeometryHierarchy
end
ObjectNode(id::ObjectID,geom) = ObjectNode(id,geom,geom_hierarchy(geom))
ObjectNode(n::ObjectNode,geom) = ObjectNode(n.id,geom)
has_component(n::SceneNode,id) = false
# Necessary for copying
RobotNode{R}(n::RobotNode,args...) where {R} = RobotNode(n.id,args...)

struct AssemblyNode <: SceneNode
    id::AssemblyID
    geom::GeomNode
    components::TransformDict{Union{ObjectID,AssemblyID}}
    geom_hierarchy::GeometryHierarchy
end
AssemblyNode(n::AssemblyNode,geom) = AssemblyNode(n.id,geom,n.components,geom_hierarchy(geom))
AssemblyNode(id,geom) = AssemblyNode(id,geom,TransformDict{Union{ObjectID,AssemblyID}}(),geom_hierarchy(geom))
assembly_components(n::AssemblyNode)         = n.components
add_component!(n::AssemblyNode,p)   = push!(n.components,p)
has_component(n::AssemblyNode,id)   = haskey(assembly_components(n),id)
child_transform(n::AssemblyNode,id) = assembly_components(n)[id]
num_components(n) = length(assembly_components(n))
required_transforms_to_children(n::AssemblyNode) = assembly_components(n)

struct TransportUnitNode{C<:Union{ObjectID,AssemblyID},T} <: SceneNode
    geom::GeomNode
    cargo::Pair{C,T}
    robots::TransformDict{BotID} # must be filled with unique invalid ids
    geom_hierarchy::GeometryHierarchy
end
TransportUnitNode(n::TransportUnitNode,geom) = TransportUnitNode(geom,n.cargo,n.robots,geom_hierarchy(geom))
TransportUnitNode(geom,cargo) = TransportUnitNode(geom,cargo,TransformDict{BotID}(),geom_hierarchy(geom))
TransportUnitNode(geom,cargo_id::Union{AssemblyID,ObjectID}) = TransportUnitNode(geom,cargo_id=>identity_linear_map())
TransportUnitNode(cargo::Pair) = TransportUnitNode(GeomNode(nothing),cargo)
TransportUnitNode(cargo_id::Union{AssemblyID,ObjectID}) = TransportUnitNode(cargo_id=>identity_linear_map())
TransportUnitNode(cargo::Union{AssemblyNode,ObjectNode}) = TransportUnitNode(node_id(cargo))
robot_team(n::TransportUnitNode) = n.robots
cargo_id(n::TransportUnitNode) = n.cargo.first
cargo_type(n::TransportUnitNode) = isa(cargo_id(n),AssemblyID) ? AssemblyNode : ObjectNode
Base.copy(n::TransportUnitNode) = TransportUnitNode(n,copy(n.geom))
function required_transforms_to_children(n::TransportUnitNode)
    merge(robot_team(n),Dict(n.cargo))
end

const TransportUnitID = TemplatedID{Tuple{T,C}} where {C,T<:TransportUnitNode}
GraphUtils.node_id(n::TransportUnitNode{C,T}) where {C,T} = TemplatedID{Tuple{TransportUnitNode,C}}(get_id(cargo_id(n)))
Base.convert(::Pair{A,B},pair) where {A,B} = convert(A,pair.first)=>convert(B,p.second)

has_component(n::TransportUnitNode,id::Union{ObjectID,AssemblyID})  = id == cargo_id(n)
has_component(n::TransportUnitNode,id::BotID)       = haskey(robot_team(n),id)
child_transform(n::TransportUnitNode,id::Union{ObjectID,AssemblyID}) = has_component(n,id) ? n.cargo.second : throw(ErrorException("TransportUnitNode $n does not have component $id"))
child_transform(n::TransportUnitNode,id::BotID)     = robot_team(n)[id]
add_robot!(n::TransportUnitNode,p)                  = push!(robot_team(n),p)
remove_robot!(n::TransportUnitNode,id)              = delete!(robot_team(n),id)
add_robot!(n::TransportUnitNode,r,t)                = add_robot!(n,r=>t)
add_robot!(n::TransportUnitNode,t::CoordinateTransformations.AffineMap)    = add_robot!(n,get_unique_invalid_id(RobotID)=>t)
function is_in_formation(n::TransportUnitNode,scene_tree)
    all([Graphs.has_edge(scene_tree,n,id) for (id,_) in robot_team(n)])
end
function capture_robots!(agent::TransportUnitNode,scene_tree)
    formed = true
    for (robot_id,_) in robot_team(agent)
        if !Graphs.has_edge(scene_tree,agent,robot_id)
            if !capture_child!(scene_tree,agent,robot_id)
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
function swap_robot_id!(transport_unit,old_id,new_id)
    team = robot_team(transport_unit)
    tform = child_transform(transport_unit,old_id)
    # @info "robot team before swap" team
    remove_robot!(transport_unit,old_id)
    add_robot!(transport_unit,new_id=>tform)
    # @info "robot team after swap" team
end

export recurse_child_geometry

"""
    recurse_child_geometry(node::SceneNode,tree,key=BaseGeomKey(),depth=typemax(Int))

Return an iterator over all geometry elements matching `key` that belong to
`node` or to any of `node`'s descendants down to depth `depth`.
"""
function recurse_child_geometry(node::SceneNode,tree,key=BaseGeomKey(),depth=typemax(Int))
    if depth < 0
        return nothing
    end
    geom = [get_base_geom(node,key)]
    if !(geom[1] === nothing)
        return geom
    end
    return nothing
end
function recurse_child_geometry(dict::TransformDict,tree,key=BaseGeomKey(),depth=typemax(Int))
    if depth < 0
        return nothing
    end
    geoms = []
    for (child_id,tform) in dict
        if Graphs.has_vertex(tree,child_id)
            child_geom = recurse_child_geometry(get_node(tree, child_id),tree,key,depth)
            if !(child_geom === nothing)
                push!(geoms, transform_iter(tform,child_geom))
            end
        end
    end
    if isempty(geoms)
        return nothing
    end
    Base.Iterators.flatten(geoms)
end
function recurse_child_geometry(node::Union{TransportUnitNode,AssemblyNode},
        tree,key=BaseGeomKey(),depth=typemax(Int)
        )
    if depth < 0
        return nothing
    end
    geom = [get_base_geom(node,key)]
    child_geom = recurse_child_geometry(required_transforms_to_children(node),tree,key,depth-1)
    if (geom[1] === nothing)
        return child_geom
    elseif (child_geom === nothing)
        return geom
    else
        return Base.Iterators.flatten((geom,child_geom))
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
for U in (:TransportUnitID,:TransportUnitNode)
    for V in (:RobotID,:RobotNode,:AssemblyID,:AssemblyNode,:ObjectID,:ObjectNode)
        @eval GraphUtils.make_edge(g,u::$U,v::$V) = TemporaryEdge()
    end
end
for U in (:AssemblyID,:AssemblyNode)
    for V in (:ObjectID,:ObjectNode,:AssemblyID,:AssemblyNode)
        @eval GraphUtils.make_edge(g,u::$U,v::$V) = PermanentEdge()
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
- `AssemblyNode` refers to a rigid collection of multiple objects and/or
subassemblies
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
    graph               ::Graphs.DiGraph               = Graphs.DiGraph()
    nodes               ::Vector{SceneNode}     = Vector{SceneNode}()
    vtx_map             ::Dict{AbstractID,Int}  = Dict{AbstractID,Int}()
    vtx_ids             ::Vector{AbstractID}    = Vector{AbstractID}()
    inedges             ::Vector{Dict{Int,SceneTreeEdge}}   = Vector{Dict{Int,SceneTreeEdge}}()
    outedges            ::Vector{Dict{Int,SceneTreeEdge}}   = Vector{Dict{Int,SceneTreeEdge}}()
end
GraphUtils.add_node!(tree::SceneTree,node::SceneNode) = add_node!(tree,node,node_id(node))
GraphUtils.get_vtx(tree::SceneTree,n::SceneNode) = get_vtx(tree,node_id(n))
function Base.copy(tree::SceneTree)
    SceneTree(
        graph = deepcopy(tree.graph),
        nodes = map(copy, tree.nodes), # TODO This may cause problems with TransformNode
        vtx_map = deepcopy(tree.vtx_map),
        vtx_ids = deepcopy(tree.vtx_ids)
    )
end

"""
    set_child!(tree::SceneTree,parent::AbstractID,child::AbstractID)

Only eligible children can be added to a candidate parent in a `SceneTree`.
Eligible edges are those for which the child node is listed in the components of
the parent. `ObjectNode`s and `RobotNode`s may not have any children.
"""
function set_child!(tree::SceneTree,parent::AbstractID,child::AbstractID)
    node = get_node(tree,parent)
    @assert has_component(node,child) "$(get_node(tree,child)) cannot be a child of $(node)"
    t = child_transform(node,child)
    child_node = get_node(tree,child)
    set_child!(tree,parent,get_vtx(tree,child),t,make_edge(tree,node,child_node))
end
set_child!(tree::SceneTree,parent::SceneNode,args...) = set_child!(tree,node_id(parent),args...)
set_child!(tree::SceneTree,parent::SceneNode,child::SceneNode) = set_child!(tree,node_id(parent),node_id(child))
set_child!(tree::SceneTree,parent::AbstractID,child::SceneNode,args...) = set_child!(tree,parent,node_id(child),args...)
function force_remove_edge!(tree::SceneTree,u,v)
    rem_parent!(get_node(tree,v))
    if Graphs.has_edge(tree,u,v)
        GraphUtils.delete_edge!(tree,u,v)
    end
end

"""
    Graphs.rem_edge!(tree::SceneTree,u,v)

Edge may only be removed if it is not permanent.
"""
function Graphs.rem_edge!(tree::SceneTree,u,v)
    if !Graphs.has_edge(tree,u,v)
        return true
    end
    @assert !isa(get_edge(tree,u,v),PermanentEdge) "Edge $u → $v is permanent!"
    force_remove_edge!(tree,u,v)
    # rem_parent!(get_node(tree,v))
    # GraphUtils.delete_edge!(tree,u,v)
end

"""
    disband!(tree::SceneTree,n::TransportUnitNode)

Disband a transport unit by removing the edge to its children.
"""
function disband!(tree::SceneTree,n::TransportUnitNode)
    for (r,tform) in robot_team(n)
        if !Graphs.has_edge(tree,n,r)
            @warn "Disbanding $n -- $r should be attached, but is not"
        end
        Graphs.rem_edge!(tree,n,r)
    end
    Graphs.rem_edge!(tree,n,cargo_id(n))
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
function is_within_capture_distance(parent::SceneNode,child::SceneNode,args...)
    t       = relative_transform(global_transform(parent),global_transform(child))
    t_des   = child_transform(parent,node_id(child))
    is_within_capture_distance(t,t_des,args...)
end

function is_within_capture_distance(t, t_des, ttol=capture_distance_tolerance(), rtol=capture_rotation_tolerance())
    et = norm(t.translation - t_des.translation) # translation error
    er = Inf
    # Rotation error
    # QuatVecMap -- cheapest to compute, but goes singular at 180 degrees  (sign ambiguity)
    # MRPMap     -- singular at 360 degrees (sign ambiguity)
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
function capture_child!(tree::SceneTree,u,v,ttol=capture_distance_tolerance(),rtol=capture_rotation_tolerance())
    nu = get_node(tree,u)
    nv = get_node(tree,v)
    @assert has_component(nu,node_id(nv)) "$nu cannot capture $nv"
    if is_within_capture_distance(get_node(tree,u),get_node(tree,v),ttol,rtol)
        if !is_root_node(tree,v)
            p = get_node(tree,get_parent(tree,v)) # current parent
            @assert(isa(p, TransportUnitNode),
                "Trying to capture child $v from non-TransportUnit parent $p")
            # NOTE that there may be a time gap if the cargo has to be lifted
            # into place by a separate "robot"
            disband!(tree,p)
        end
        return set_child!(tree,u,v)
    end
    # t = relative_transform(tree,u,v)
    # t_des = child_transform(nu,node_id(nv))
    # et = norm(t.translation - t_des.translation) # translation error
    # er = norm(Rotations.rotation_error(t,t_des)) # rotation_error
    # if et < ttol && er < rtol
    #     if !is_root_node(tree,v)
    #         p = get_node(tree,get_parent(tree,v)) # current parent
    #         @assert(isa(p, TransportUnitNode),
    #             "Trying to capture child $v from non-TransportUnit parent $p")
    #         # NOTE that there may be a time gap if the cargo has to be lifted
    #         # into place by a separate "robot"
    #         disband!(tree,p)
    #     end
    #     return set_child!(tree,u,v)
    # end
    return false
end

function construct_scene_dependency_graph(scene_tree)
    G = Graphs.DiGraph(Graphs.nv(scene_tree))
    for n in get_nodes(scene_tree)
        for (child_id,_) in required_transforms_to_children(n)
            Graphs.add_edge!(G,get_vtx(scene_tree,n),get_vtx(scene_tree,child_id))
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
function compute_approximate_geometries!(scene_tree,key=HypersphereKey(),base_key=key;
    leaves_only=false,
    ϵ=0.0,
    )
    # Build dependency graph (to enable full topological sort)
    G = construct_scene_dependency_graph(scene_tree)
    ids = (get_vtx_id(scene_tree,v) for v in reverse(Graphs.topological_sort_by_dfs(G)))
    for node in node_iterator(scene_tree, ids)
        h = node.geom_hierarchy
        if !Graphs.has_vertex(h,key)
            if isa(node,Union{ObjectNode,RobotNode})
                for (k1,k2) in [(BaseGeomKey(),base_key),(base_key, key)]
                    if !Graphs.has_vertex(h,k2)
                        base_geom = recurse_child_geometry(node,scene_tree,k1,1)
                        if !(base_geom === nothing)
                            add_child_approximation!(h,k2,k1,base_geom)
                        end
                    end
                end
            elseif leaves_only == false
                geoms = recurse_child_geometry(node,scene_tree,base_key,1)
                if !(geoms === nothing)
                    add_child_approximation!(h, key, BaseGeomKey(),geoms)
                else
                    @warn "Unable to recurse child geometry" node
                end
            end
        end
    end
    return scene_tree
end

export remove_geometry!
remove_geometry!(n::GeometryHierarchy,key) = rem_node!(n,key)
remove_geometry!(n::SceneNode,key) = rem_node!(n.geom_hierarchy,key)
function remove_geometry!(scene_tree::SceneTree,key)
    for n in get_nodes(scene_tree)
        remove_geometry!(n,key)
    end
end


"""
    jump_to_final_configuration!(scene_tree;respect_edges=false)

Jump to the final configuration state wherein all `ObjectNode`s and
`AssemblyNode`s are in the configurations specified by their parent assemblies.
"""
function jump_to_final_configuration!(scene_tree;respect_edges=false,set_edges=false)
    for node in get_nodes(scene_tree)
        if matches_template(AssemblyNode,node)
            for (id,tform) in assembly_components(node)
                # if Graphs.has_edge(scene_tree, node, id)
                    force_remove_edge!(scene_tree, node, id)
                    set_child!(scene_tree, node, id)
                    if !set_edges
                        force_remove_edge!(scene_tree,node,id)
                    end
                # end
                # if !respect_edges || Graphs.has_edge(scene_tree,node,id)
                #     set_local_transform!(get_node(scene_tree,id),tform)
                # end
            end
        end
    end
    scene_tree
end


### Collision Table
const CollisionStack = Dict{Int,Set{ID}} where {ID}
get_active_collision_ids(c::CollisionStack,t::Int) = get(c,t,valtype(c)())

"""
    CollisionTable <: AbstractCustomNGraph{Graphs.Graph,CollisionStack,I}

A Data structure for efficient discrete-time collision checking between large
    numbers of objects.
An edge between two nodes indicates that those nodes are "eligible" to collide.
If there is no edge, we don't need to check collisions.
Each node of the graph is a `CollisionStack`--a kind of timer bank that keeps
track of when collision checking needs to be performed between two objects.
The timing depends on the distance between the objects at a given time step, as
well as the maximum feasible speed of each object.
"""
@with_kw struct CollisionTable{ID} <: AbstractCustomNGraph{Graphs.Graph,CollisionStack{ID},ID}
    graph               ::Graphs.Graph                 = Graphs.Graph()
    nodes               ::Vector{CollisionStack{ID}} = Vector{CollisionStack{ID}}()
    vtx_map             ::Dict{ID,Int}           = Dict{ID,Int}()
    vtx_ids             ::Vector{ID}             = Vector{ID}()
end

# active struct CollisionTableNode
#   geom # get_cached_geom(geom)
#   vmax # maximum speed
#

# @with_kw struct CollisionTable <: AbstractCustomNGraph{Graphs.Graph,CollisionStack{ID},ID}
#     graph               ::Graphs.Graph                 = Graphs.Graph()
#     nodes               ::Vector{CollisionStack{ID}} = Vector{CollisionStack{ID}}()
#     vtx_map             ::Dict{ID,Int}           = Dict{ID,Int}()
#     vtx_ids             ::Vector{ID}             = Vector{ID}()
# end

function get_transformed_config end
function get_max_speed end

# function min_time_to_collision(x1,x2,vmax1,vmax2)
#     d_min = distance_lower_bound(x1,x2)
#     v_max = vmax1 + vmax2
#     dt = d_min / v_max
# end

"""
    update_collision_table!(table,env_state,env,i,t=0)

Updates the counters in a CollisionTable. Should be called each time the modeled
process "steps forward" in time.
"""
function update_collision_table!(table,env_state,env,i,t=0)
    c = get_tranformed_config(env,env_state,i,t)
    stack = get_node(table,i)
    active_ids = get_active_collision_ids(stack,t)
    while !isempty(active_ids)
        j = pop!(active_ids)
        Graphs.has_edge(table,i,j) ? nothing : continue
        cj = get_transformed_config(env,env_state,j,t)
        d_min = distance_lower_bound(c,cj)
        v_max = get_max_speed(env,i) + get_max_speed(env,j)
        dt = d_min / v_max
        push!(get!(stack,t+Int(floor(dt)),valtype(stack)()),j)
    end
end

function init_collision_table!(table,env_state,env,i,t=0)
    stack = get_node(table,i)
    for j in outneighbors(table,i)
        push!(get!(stack,t,valtype(stack)()),get_vtx_id(table,j))
    end
    update_collision_table!(table,env_state,env,i,t)
end

"""
    find_collision(table,env_state,env,i,t=0)

Checks for collisions between object "i" and all objects that may be (according
    to the table) near enough to warrant collision checking at time `t`.
"""
function find_collision(table,env_state,env,i,t=0)
    c = get_tranformed_config(env,env_state,i,t)
    stack = get_node(table,i)
    active_ids = get_active_collision_ids(stack,t)
    for j in active_ids
        Graphs.has_edge(table,i,j) ? nothing : continue
        cj = get_transformed_config(env,env_state,j,t)
        if has_overlap(c,cj)
            return true, j
        end
    end
    return false, -1
end

#### Visibility Graphs

"""
    project_point_to_line(p,p1,p2)

project `p` onto line between `p1` and `p2`
"""
function project_point_to_line(p,p1,p2)
    base = p2-p1
    leg = p-p1
    v = p1 + dot(leg,base) * base / norm(base)^2
end

"""
    project_onto_vector(vec,b)

project `vec` onto `b`.
"""
project_onto_vector(vec,b) = b*dot(vec,b)/dot(b,b)
# project_onto_unit_vector(vec,b) = project_onto_vector(vec,normalize(b))

"""
"""
projection_norm(a,b) = dot(a,b)/norm(b)

include("dynamics.jl")

"""
    circle_intersects_line(circle,line)

Return true if `circle` intersect `line`.
"""
function circle_intersection_with_line(circle,line)
    p1,p2 = line.points[1], line.points[2]
    c = get_center(circle)
    pt = project_point_to_line(c,p1,p2)
    d = norm(p2-p1)
    d1 = norm(p1-pt)
    d2 = norm(p2-pt)
    r = get_radius(circle)
    if isapprox(d,d1+d2)
        return norm(c-pt) - r
    else
        return min(norm(c-p1),norm(c-p2)) - r
    end
end
circle_intersects_line(args...) = circle_intersection_with_line(args...) < 0
circle_intersection_with_line(circle,pt1,pt2) = circle_intersection_with_line(circle,
    GeometryBasics.Line(GeometryBasics.Point(pt1...),GeometryBasics.Point(pt2...)))

"""
    construct_visibility_graph(circles)
"""
function construct_visibility_graph(circles::Dict)
    graph = NEGraph{Graphs.Graph,GeometryBasics.HyperSphere,Float64,keytype(circles)}()
    for (k,geom) in circles
        add_node!(graph,geom,k)
    end
    for v in Graphs.vertices(graph)
        n = node_val(get_node(graph,v))
        for v2 in Base.Iterators.rest(Graphs.vertices(graph),v)
            n2 = node_val(get_node(graph,v2))
            visible = true
            line = GeometryBasics.Line(GeometryBasics.Point(n.center),GeometryBasics.Point(n2.center))
            for v3 in Graphs.vertices(graph)
                v3 == v || v3 == v2 ? continue : nothing
                n3 = node_val(get_node(graph,v3))
                if circle_intersects_line(n3,line)
                    visible = false
                    break
                end
            end
            if visible
                Graphs.add_edge!(graph,v,v2,norm(n2.center-n.center))
            end
        end
    end
    graph
end
function Graphs.weights(g::NEGraph{Graphs.Graph,N,Float64,ID}) where {N,ID}
    m = zeros(Graphs.nv(g),Graphs.nv(g))
    for e in edges(g)
        edge = get_edge(g,e)
        m[e.src,e.dst] = edge_val(edge)
        m[e.dst,e.src] = edge_val(edge)
    end
    m
end

"""
    get_tangent_pts_on_circle(circle,base_pt;)

Returns the two points on a circle for which a line from `base_pt` through `pt`
    is tangent to `circle`.
"""
function get_tangent_pts_on_circle(circle,base_pt;)
    r = get_radius(circle)
    c = get_center(circle)
    c = c - base_pt
    θ = asin(r/norm(c)) # angle of ray from x1 that is tangent to a circle of radius r centered at x2
    R = SMatrix{2,2,Float64}([cos(θ+π/2) -sin(θ+π/2); sin(θ+π/2) cos(θ+π/2)])
    v_left = R * c/norm(c)
    v_right = R' * c/norm(c)
    v_left, v_right
end

end
