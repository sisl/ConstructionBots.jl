module FactoryRendering

using CoordinateTransformations
using Rotations
using GeometryBasics
using StaticArrays
using Parameters
using Compose
using Colors
using GraphUtils

export
    OrthographicProjection,
    BirdsEyeProjection,
    construct_unit_box,
    EntityRenderModel,
    RenderLayerCache,
    RenderCache,
    get_render_layer,
    update_layer_cache!,
    default_vtx_color,
    default_entity_radius,
    default_robot_radius,
    default_object_radius,
    default_entity_shape,
    default_robot_color,
    default_robot_shape,
    default_object_color,
    get_aspect_ratio

global DEFAULT_VTX_COLOR = colorant"LightGray"
global DEFAULT_ENTITY_RADIUS = 0.5
global DEFAULT_ROBOT_RADIUS = 0.5
global DEFAULT_OBJECT_RADIUS = 0.25
global DEFAULT_ROBOT_COLOR = RGB(0.1,0.5,0.9)
global DEFAULT_OBJECT_COLOR = colorant"black"
global DEFAULT_GOAL_COLOR = RGB(0.0,1.0,0.0)
global DEFAULT_GOAL_RADIUS = 0.3

abstract type RenderParam end
struct EntityRadius <: RenderParam end
_default_val(::Type{EntityRadius}) = DEFAULT_ENTITY_RADIUS
struct EntityColor <: RenderParam end
_default_val(::Type{EntityColor}) = DEFAULT_ROBOT_COLOR
struct EntityLabelScale <: RenderParam end
_default_val(::Type{EntityLabelScale}) = 0.7

global DEFAULT_RENDER_PARAMS = Dict()
function get_render_param(k,key_list...;default=nothing)
    GraphUtils.nested_default_dict_get(DEFAULT_RENDER_PARAMS,k,key_list...;default=default)
end
function set_render_param!(k,args...)
    global DEFAULT_RENDER_PARAMS
    GraphUtils.nested_default_dict_set!(DEFAULT_RENDER_PARAMS,k,args...)
end
set_render_param!(:Color,       "grey")
set_render_param!(:TextColor,   "black")
set_render_param!(:ShapeFunc,   (c,r)->Compose.circle(c[1],c[2],r))
set_render_param!(:Radius,      0.45)
set_render_param!(:LabelScale,  0.7)
set_render_param!(:ShowLabel,   true)
set_render_param!(:Color,         :Robot,       RGB(0.1,0.5,0.9))
set_render_param!(:Radius,        :Robot,       0.45)
set_render_param!(:LabelScale,    :Robot,       0.7)
set_render_param!(:Color,         :Object,      RGB(1.0,0.5,0.2))
set_render_param!(:Radius,        :Object,      0.35)
set_render_param!(:LabelScale,    :Object,      0.7)
set_render_param!(:ShapeFunc,     :Vtx,         (c,r)->Compose.rectangle(c[1]-r,c[2]-r,2*r,2*r))
set_render_param!(:Color,         :Vtx,         colorant"LightGray")
set_render_param!(:Radius,        :Vtx,         0.45)
set_render_param!(:LabelScale,    :Vtx,         0.7)
set_render_param!(:ShowLabel,     :Vtx,         false)
set_render_param!(:ShapeFunc,     :Goal,        (c,r)->Compose.rectangle(c[1]-r,c[2]-r,2*r,2*r))
set_render_param!(:Color,         :Goal,        RGB(0.0,1.0,0.0))
set_render_param!(:Radius,        :Goal,        0.35)
set_render_param!(:LabelScale,    :Goal,        0.7)

default_vtx_color()     = DEFAULT_VTX_COLOR
set_default_vtx_color() = DEFAULT_VTX_COLOR
default_entity_radius() = DEFAULT_ENTITY_RADIUS
default_robot_radius()  = DEFAULT_ROBOT_RADIUS
default_object_radius() = DEFAULT_OBJECT_RADIUS
default_robot_color()   = DEFAULT_ROBOT_COLOR
default_object_color()  = DEFAULT_OBJECT_COLOR
default_goal_color()    = DEFAULT_GOAL_COLOR
default_goal_radius()   = DEFAULT_GOAL_RADIUS

function set_default_vtx_color!(val)     
    global DEFAULT_VTX_COLOR=val 
end
function set_set_default_vtx_color!(val) 
    global DEFAULT_VTX_COLOR=val 
end
function set_default_entity_radius!(val) 
    global DEFAULT_ENTITY_RADIUS    =val 
end
function set_default_robot_radius!(val)  
    global DEFAULT_ROBOT_RADIUS=val 
end
function set_default_object_radius!(val) 
    global DEFAULT_OBJECT_RADIUS=val 
end
function set_default_robot_color!(val)   
    global DEFAULT_ROBOT_COLOR=val 
end
function set_default_object_color!(val)  
    global DEFAULT_OBJECT_COLOR=val 
end
function set_default_goal_color!(val)
    global DEFAULT_GOAL_COLOR = val
end
function set_default_goal_radius!(val)
    global DEFAULT_GOAL_RADIUS = val
end

default_entity_shape() = Circle(Point(0.0,0.0),default_entity_radius())
default_robot_shape() = Circle(Point(0.0,0.0),default_robot_radius())
get_aspect_ratio() = Compose.default_graphic_height / Compose.default_graphic_width

abstract type RenderProjection end
(p::RenderProjection)(v::Vector) = map(p,v)
struct BirdsEyeProjection <: RenderProjection end
struct OrthographicProjection <: RenderProjection
    theta::Float64 # yaw
    azimuth::Float64 # roll - π/2 is birds eye view
    lmap::LinearMap
end
function OrthographicProjection(theta,azimuth)
    tilt = LinearMap(SMatrix{2,3,Float64}(
        1.0, 0.0, 
        0.0, sin(azimuth), 
        0.0, cos(azimuth)))
    rot = LinearMap(RotZ(theta))
    OrthographicProjection(theta,azimuth,tilt ∘ rot)
end
function CoordinateTransformations.LinearMap(proj::OrthographicProjection) 
    proj.lmap
end

struct AxisAlignedEllipse <: GeometryPrimitive{2,Float64}
    center::Point2{Float64}
    rx::Float64
    ry::Float64
end
Compose.circle(c::Circle)               = Compose.circle(c.center[1],c.center[2],c.r)
Compose.ellipse(e::AxisAlignedEllipse) = Compose.ellipse(e.center[1],e.center[2],e.rx,e.ry)
Compose.polygon(n::GeometryBasics.Ngon) = Compose.polygon(Vector(map(NTuple{2,Float64}, coordinates(n))))
convert_to_compose_form(e::Circle)              = Compose.circle(e)
convert_to_compose_form(e::Rect2D)              = Compose.rectangle(e.origin..., e.widths...)
convert_to_compose_form(e::AxisAlignedEllipse)  = Compose.ellipse(e)
convert_to_compose_form(e::GeometryBasics.Ngon) = Compose.polygon(e)
Base.:(+)(c::Union{HyperSphere,Rect2D,GeometryBasics.Ngon,AxisAlignedEllipse},v::Vector) = map(p->c+p,v)
Base.:(+)(c::HyperSphere,p::Point2) = Circle(c.center+p,c.r)
Base.:(+)(c::AxisAlignedEllipse,p::Point2) = AxisAlignedEllipse(c.center+p,c.rx,c.ry)
function Base.:(+)(g::G,p::P) where {N,M,T,P<:Point{N,T},G<:GeometryBasics.Ngon{N,T,M,P}}
    G(map(c->c+p, coordinates(g)))
end

(proj::BirdsEyeProjection)(c::Point3{Float64}) = Point2{Float64}(c[1],c[2])
(proj::BirdsEyeProjection)(c::Point2{Float64}) = c
(proj::BirdsEyeProjection)(c::Circle) = c
(proj::BirdsEyeProjection)(c::Rect2D) = c
(proj::BirdsEyeProjection)(r::G) where {M,G<:GeometryBasics.Ngon{2,Float64,M,Point{2,Float64}}} = r

(proj::OrthographicProjection)(c::Point3{Float64}) = proj.lmap(c)
(proj::OrthographicProjection)(c::Point2{Float64}) = proj.lmap(Point(c[1],c[2],0.0))
# (proj::OrthographicProjection)(c::NTuple{2,T}) where {T<:Real} = proj.lmap(Point(c[1],c[2],0.0)).data
# (proj::OrthographicProjection)(c::NTuple{3,T}) where {T<:Real} = proj.lmap(Point(c[1],c[2],c[3])).data
(proj::OrthographicProjection)(c::Tuple{A,B,C}) where {A<:Real,B<:Real,C<:Real} = proj.lmap(Point(c[1],c[2],c[3])).data
(proj::OrthographicProjection)(c::Tuple{A,B}) where {A<:Real,B<:Real} = proj.lmap(Point(c[1],c[2],0.0)).data

function (proj::OrthographicProjection)(c::Circle)
    AxisAlignedEllipse(
        proj(Point(c.center[1],c.center[2],0.0)),
        c.r,
        c.r*sin(proj.azimuth)
    )
end
# function (proj::OrthographicProjection)(r::G) where {N,M,G<:GeometryBasics.Ngon{N,Float64,M,Point{N,Float64}}}
function (proj::RenderProjection)(r::G) where {N,M,G<:GeometryBasics.Ngon{N,Float64,M,Point{N,Float64}}}
    GeometryBasics.Ngon(SVector{M,Point2{Float64}}(map(proj,r.points)...))
end
function (tform::Translation{V})(r::G) where {V,N,M,G<:GeometryBasics.Ngon{N,Float64,M,Point{N,Float64}}}
    GeometryBasics.Ngon(SVector{M,Point2{Float64}}(map(tform,r.points)...))
end
(proj::OrthographicProjection)(v::Vector{Point{N,Float64}}) where {N} = map(proj,v)
(proj::OrthographicProjection)(v::Vector{T}) where {T<:Tuple} = map(proj,v)
function (proj::OrthographicProjection)(r::Rect2D)
    pts = map(Point2,collect(coordinates(r)))
    proj(GeometryBasics.Ngon(SVector(pts[1],pts[2],pts[4],pts[3])))
end

function bounding_hyperrect(vtxs::Vector{Point{N,Float64}},buffer=zero(Vec{N,Float64})) where {N}
    p1 = Vec{N,Float64}(map(i->minimum(map(v->v[i],vtxs)),1:N) .- buffer)
    p2 = Vec{N,Float64}(map(i->maximum(map(v->v[i],vtxs)),1:N) .+ 2*buffer)
    rect = Rect(p1, p2 .- p1)
end
function construct_unit_box(bbox,aspect=get_aspect_ratio())
    corners = collect(coordinates(bbox))
    xl = minimum(map(c->c[1],corners))
    yl = minimum(map(c->c[2],corners))
    xh = maximum(map(c->c[1],corners))
    yh = maximum(map(c->c[2],corners))
    construct_unit_box(xl,yl,xh,yh,aspect)
end
function construct_unit_box(xl,yl,xh,yh,aspect=get_aspect_ratio())
    @assert (xl < xh) "$xl >= $xh, but x_low must be less than x_high"
    @assert (yl < yh) "$yl >= $yh, but y_low must be less than y_high"
    dx = xh-xl
    dy = yh-yl
    if dy/dx > aspect
        dx = dy/aspect
    else
        dy = dx*aspect
    end
    cx = (1/2)*(xh+xl)
    cy = (1/2)*(yh+yl)
    UnitBox(cx-dx/2,cy-dy/2,dx,dy)
end

# """
#     EntityConfiguration{N}

# 3D Robot or object configuration.
# """
# @with_kw mutable struct EntityConfiguration
#     pos::Point{3,Float64}   = zero(Point{3,Float64})
#     theta::Float64          = 0.0
# end
# struct RenderPrimitive{G,C}
#     geom::G
#     color::C
# end
# const RenderStack{N} = NTuple{N,RenderPrimitive} 
# function circle_entity_render_stack(radius,color)
#     RenderStack{1}([
#         # RenderPrimitive(Line(Point2(0.0,0.0),Point2(0.0,0.0)),color),
#         RenderPrimitive(Circle(Point2(0.0,0.0),radius),color),
#     ])
# end
# circle_robot_render_stack(radius=default_robot_radius(),color=default_robot_color()) = circle_entity_render_stack(radius,color)
# circle_object_render_stack(radius=default_object_radius(),color=default_object_color()) = circle_entity_render_stack(radius,color)

# """
#     struct GridWorldModel{N}

# Models the layout of vertices on a grid floor
# """
# @with_kw mutable struct GridWorldModel{N,S}
#     vtxs::Vector{Point{N,Float64}}  = Vector{Point{N,Float64}}()
#     vtx_shape::S                    = Rect(zero(Vec{N,Float64}),0.45*Vec2(1.0,1.0))
#     vtx_colors::Vector{Color}       = Vector{Color}(map(v->default_vtx_color(),vtxs))
# end
# (t::AffineMap)(model::GridWorldModel) = GridWorldModel(
#         vtxs=map(t,model.vtxs),
#         vtx_shape=t(model.vtx_shape),
#         model.vtx_colors
#         )

"""
    mutable struct EntityRenderModel{G,C}

For rendering multiple entities. Fields:
*configs
*shapes
*colors (can be be a vector of colors or a single color)
"""
@with_kw mutable struct EntityRenderModel
    configs     = Vector{Point{3,Float64}}()
    shapes      = map(c->default_entity_shape(), configs)
    colors      = map(v->default_robot_color(),configs)
    # label_entities::Bool                = false
    # labels      = map(v->"",configs)
    # show_labels = false
end
struct StaticEntityRenderModel{V,G,C}
    configs::V
    shapes::G
    colors::C
end

function get_render_layer(m::EntityRenderModel,proj=BirdsEyeProjection())
    vtxs = proj(m.configs)
    shapes = proj(m.shapes)
    if isa(shapes,Vector)
        tformed_shapes = shapes .+ vtxs
    else
        tformed_shapes = shapes + vtxs
    end
    forms = map(convert_to_compose_form, tformed_shapes)
    if isa(m.colors, Vector)
        colors = m.colors
    else
        colors = map(i->m.colors,1:length(vtxs))
    end
    return Compose.compose(context(), [(context(),form,fill(c)) for (form,c) in zip(forms,colors)]...)
end
function construct_unit_box(m::EntityRenderModel,proj=BirdsEyeProjection(),buffer=zeros(2))
    vtxs = proj(m.configs)
    construct_unit_box(bounding_hyperrect(vtxs,buffer))
end

@with_kw mutable struct RenderLayerCache{E}
    model::E        = EntityRenderModel()
    layer::Context  = context()
end

export PathRenderModel

default_path_width() = 0.1
@with_kw mutable struct PathRenderModel
    paths       = Vector{Vector{Point{3,Float64}}}()
    thicknesses = map(c->default_path_width(), paths)
    colors      = map(v->default_robot_color(),paths)
end
function get_render_layer(m::PathRenderModel,proj=BirdsEyeProjection())
    paths = map(proj,m.paths)
    forms = [Compose.compose(context(),
        Compose.line(map(pt->(pt[1],pt[2]),path)),
        stroke(c),
        linewidth(t),
        ) for (path,c,t) in zip(paths,m.colors,m.thicknesses)]
    
    return Compose.compose(context(), forms..., stroke(m.colors))
end

function update_layer_cache!(c::RenderLayerCache,args...)
    c.layer = get_render_layer(c.model,args...)
    return c
end

struct RenderCache{T}
    caches::T
end

function draw_entity(keylist=[];
        label="",
        label_offset=[0.0,0.0],
        show_label=get_render_param(:ShowLabel,keylist...),
        color=get_render_param(:Color,keylist...),
        border_color=nothing,
        text_color=get_render_param(:TextColor,keylist...),
        label_scale=get_render_param(:LabelScale,keylist...),
        r=get_render_param(:Radius,keylist...),
        shape_func=get_render_param(:ShapeFunc,keylist...),
        t=0.0, # linewidth
    )
    x = 0.5
    y = 0.5
    xl = label_offset[1]+x
    yl = label_offset[2]+y
    c = (x,y)
    if show_label === nothing || show_label == false || label === nothing
        label = ""
    end

    Compose.compose(context(),
        (context(),
            Compose.text(xl, yl, label, hcenter, vcenter), 
            Compose.fill(text_color), 
            fontsize(label_scale*min(w,h))
            ),
        (context(),
            shape_func(c,r),
            fill(color), 
            Compose.stroke(border_color), 
            Compose.linewidth(t*w),
            ),
        )
end
draw_tile(;kwargs...) = draw_entity([:Vtx];kwargs...)
draw_robot(;kwargs...) = draw_entity([:Robot];kwargs...)
draw_object(;kwargs...) = draw_entity([:Object];kwargs...)
draw_goal(;kwargs...) = draw_entity([:Goal];kwargs...)

# 1st order path
function draw_path_segment(start,goal;
        color=get_render_param(:Color,:Path,default="black"),
        trel=get_render_param(:RelativeThickness,:Path,default=0.1),
        t=get_render_param(:Thickness,:Path,default=nothing),
    )
    if t === nothing
        t=trel*min(w,h)
    end
    Compose.compose(context(),
        (context(), Compose.line([start,goal]), stroke(color), linewidth(t),),
        (context(), Compose.circle(start..., t/2), 
            Compose.circle(goal..., t/2), fill(color),)
    )
end

function draw_path_segment(start,mid,goal;
        color=get_render_param(:Color,:Path,default="black"),
        trel=get_render_param(:RelativeThickness,:Path,default=0.1),
        t=get_render_param(:Thickness,:Path,default=nothing),
        fillet_radius=get_render_param(:FilletRadius,:Path,default=0.0),
    )
    if t === nothing
        t=trel*min(w,h)
    end
    d1 = mid .- start
    d2 = goal .- mid
    x1 = (start .+ mid) ./ 2
    x2 = (mid .+ goal) ./ 2
    if fillet_radius > 0 && abs(atan(d1...)-atan(d2...)) >= 1e-3
        v = mid .- (goal .+ start) ./ 2
        c = mid .- fillet_radius*normalize([v...])
    end
    Compose.compose(context(),
        (context(), Compose.line([x1,mid]), Compose.line([mid,x2]),
            stroke(color), linewidth(t),
        ),
        (context(), Compose.circle(mid..., t/2), fill(color),)
    )
end
function draw_path(vtxs;kwargs...)
    Compose.compose(context(),
        (draw_path_segment(v1,v2;kwargs...) for (v1,v2) in zip(vtxs,vtxs[2:end]))...
        )
end

"""
    draw_entities(xs,rs;
        draw_entity_function = v->draw_robot(label=v),
        )

xs is a vector of 2d positions
rs is a vector of 2d canvas sizes 
draw_entity_function is a custom function that renders each entity at its canvas
    location.
"""
function draw_entities(xs;
        keylist=[],
        sizes=[(1.0,1.0)],
        labels=[],
        draw_entity_function = v->draw_entity(keylist;label=get(labels,v,"")),
        kwargs...
    )
    entity_context(a,b,s) = context(
        (a-s[1]/2),
        (b-s[2]/2),
        s[1],
        s[2],
        units=UnitBox(0.0,0.0,1.0,1.0),
        )
    entities = (
        (entity_context(xs[v][1],xs[v][2],(get(sizes,v,sizes[1])...,)), 
        draw_entity_function(v)) for v in 1:length(xs)
    )
    Compose.compose(context(),entities...)
end
draw_robots(xs;kwargs...) = draw_entities(xs;keylist=[:Robot],kwargs...)
draw_objects(xs;kwargs...) = draw_entities(xs;keylist=[:Object],kwargs...)
draw_tiles(xs;kwargs...) = draw_entities(xs;keylist=[:Vtx],kwargs...)
draw_goals(xs;kwargs...) = draw_entities(xs;keylist=[:Goal],kwargs...)

end
