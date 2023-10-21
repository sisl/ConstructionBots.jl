
abstract type OverapproxModel end

LazySets.ρ(d::AbstractVector,geom::AbstractVector{N}) where {N<:GeometryBasics.Ngon} = maximum(map(v->ρ(d,v),geom))

export PolyhedronOverapprox
"""
    PolyhedronOverapprox{D,N}

Used to overrapproximate convex sets with a pre-determined set of support
vectors.
"""
struct PolyhedronOverapprox{D,N} <: OverapproxModel
    support_vectors::NTuple{N,SVector{D,Float64}}
end
get_support_vecs(model::PolyhedronOverapprox) = [v for v in model.support_vectors]
make_polytope(m::PolyhedronOverapprox) = HPolytope(map(v->LazySets.HalfSpace(v,1.0),get_support_vecs(m)))
make_polytope(m::PolyhedronOverapprox{2,N}) where {N} = HPolygon(map(v->LazySets.HalfSpace(v,1.0),get_support_vecs(m)))

"""
    BufferedPolygon

A polygon constrained to not have any degenerate vertices
"""
struct BufferedPolygon
    halfspaces::Vector{LazySets.HalfSpace}
    pts::Vector{SVector{3,Float64}}
    min_face_length::Float64 # minimum face length
end
struct BufferedPolygonPrism
    p::BufferedPolygon
    origin::SVector{3,Float64}
    extremity::SVector{3,Float64}
end
BufferedPolygonPrism(p::BufferedPolygon) = BufferedPolygonPrism(p,zero(SVector{3,Float64}),zero(SVector{3,Float64}))
function compute_buffered_polygon(vecs,buffer,points_and_radii,ϵ=0.0;
        z::Float64=0.0,
    )
    N = length(vecs)
    M = 3
    # compute minimum halfspace values
    hmin = -Inf*ones(N)
    for (pt,r) in points_and_radii
        for (i,vec) in enumerate(vecs)
            hmin[i] = max(hmin[i], dot(pt,vec) + r + ϵ)
        end
    end
    # compute minimum feasible h vals such that no corners are chopped off
    model = Model(default_optimizer())
    set_optimizer_attributes(model,default_optimizer_attributes()...)
    @variable(model, h[1:N] >= 0)
    @variable(model, pts[1:M,1:N])
    for i in 1:N
        @constraint(model,h[i] >= hmin[i])
    end
    for (i,j) in zip(1:N,Base.Iterators.drop(Base.Iterators.cycle(1:N),1))
        @constraint(model,dot(pts[1:M,i],vecs[i]) - h[i] == 0)
        @constraint(model,dot(pts[1:M,i],vecs[j]) - h[j] == 0)
        # ensure that points are at least `buffer` distance apart
        @constraint(model,
            dot(
                pts[1:M,j] .- pts[1:M,i],
                normalize(cross(cross(vecs[i],vecs[j]),vecs[j]))
                ) >= buffer
            )
    end
    @objective(model,Min,sum(h))
    optimize!(model)
    hvals = value.(h)
    pt_vals = value.(pts)
    BufferedPolygon(
        map(i->LazySets.HalfSpace(SVector{M,Float64}(vecs[i]),hvals[i]),1:N),
        map(i->SVector{M,Float64}(pt_vals[1:M,i]),1:N),
        buffer
    )
end
function compute_buffered_polygon_prism(vecs,buffer,points_and_radii,ϵ=0.0)
    # assumes that vecs are in the X-Y plane
    hlo = Inf
    hhi = -Inf
    for (x,r) in points_and_radii
        hlo = min(x[3]-(r+ϵ),hlo)
        hhi = max(x[3]+(r+ϵ),hhi)
    end
    origin = SVector{3,Float64}(0.0,0.0,hlo)
    extremity = SVector{3,Float64}(0.0,0.0,hhi)
    p = compute_buffered_polygon(vecs,buffer,points_and_radii,ϵ)
    BufferedPolygonPrism(
        BufferedPolygon(
            p.halfspaces,
            [pt .+ origin for pt in p.pts],
            p.min_face_length),
            origin,extremity)
end
function GeometryBasics.coordinates(p::BufferedPolygonPrism)
    d = p.extremity - p.origin
    [(GeometryBasics.Point{3,Float64}(pt[1],pt[2],pt[3]) for pt in p.p.pts)...,
    (GeometryBasics.Point{3,Float64}(pt[1]+d[1],pt[2]+d[2],pt[3]+d[3]) for pt in p.p.pts)...]
end
function GeometryBasics.faces(p::BufferedPolygonPrism)
    n = length(p.p.pts)
    [
        (NgonFace{4,Int}(i,i+1,i+n+1,i+n) for i in 1:n-1)...,
        NgonFace{4,Int}(n,1,n+1,2n),
        (NgonFace{4,Int}(1,i,i+1,i+2) for i in 1:n-2)..., # bottom
        (NgonFace{4,Int}(1+n,i+n,i+1+n,i+2+n) for i in 1:n-2)..., # top
    ]
end
"""
    get_wireframe_mesh(p::BufferedPolygonPrism)

Render edges only (each as a generate triangle face that is really just a line).
"""
function get_wireframe_mesh(p::BufferedPolygonPrism)
    n = length(p.p.pts)
    c = GeometryBasics.coordinates(p)
    f = [
        (NgonFace{3,Int}(i,i+n,i) for i in 1:n)...,
        (NgonFace{3,Int}(i,i+1,i) for i in 1:n-1)...,
        NgonFace{3,Int}(n,1,n),
        (NgonFace{3,Int}(i,i+1,i) for i in n+1:2*n-1)...,
        NgonFace{3,Int}(2n,n+1,2n),
        ]
    GeometryBasics.Mesh(c,f)
end
get_wireframe_mesh(n) = GeometryBasics.Mesh(collect(GeometryBasics.coordinates(n)),collect(faces(n)))
function get_wireframe_mesh(n::GeometryBasics.Cylinder)
    coords = collect(GeometryBasics.coordinates(n))[1:end-2]
    n = length(coords)/2
    f = [
        # (NgonFace{3,Int}(i,i+1,i) for i in 1:2:2n-1)...,
        (NgonFace{3,Int}(i,i+2,i) for i in 1:2n-2)...,
        NgonFace{3,Int}(2n-1,1,2n-1),
        NgonFace{3,Int}(2n,2,2n),
        ]
    GeometryBasics.Mesh(coords,f)
end

function extract_points_and_radii(p::BufferedPolygonPrism)
    d = p.extremity - p.origin
    Base.Iterators.flatten(
        [((SVector{3,Float64}(pt[1],pt[2],pt[3]),0.0) for pt in p.p.pts),
        ((SVector{3,Float64}(pt[1]+d[1],pt[2]+d[2],pt[3]+d[3]),0.0) for pt in p.p.pts)],
    )
end

function regular_buffered_polygon(n::Int,r::Float64;θ₀=0.0,buffer=0.0)
    Δθ = 2π/n
    rc = cos(Δθ/2)
    hspaces = Vector{LazySets.HalfSpace}(undef,n)
    pts = Vector{SVector{3,Float64}}(undef,n)
    for i in 1:n
        θ = θ₀ + (i-1)*Δθ
        θp = θ + Δθ/2
        hspaces[i]  = LazySets.HalfSpace(SVector{3,Float64}(cos(θ),sin(θ),0.0),rc)
        pts[i]      = SVector{3,Float64}(r*cos(θp),r*sin(θp),0.0)
    end
    BufferedPolygon(hspaces,pts,buffer)
end

"""
    PolyhedronOverapprox(dim::Int,N::Int,epsilon=0.1)

Construct a regular polyhedron overapproximation model by specifying the number
of dimensions and the number of support vectors to be arranged radially about
the axis formed by the unit vector along each dimension.
epsilon if the distance between support vector v and an existing support vector
is less than epsilon, the new vector will not be added.
"""
function PolyhedronOverapprox(dim::Int,N::Int,epsilon=0.1)
    vecs = Vector{SVector{dim,Float64}}()
    # v - the initial support vector for a given dimension
    v = zeros(dim)
    v[end] = 1.0
    for i in 1:dim
        d = zeros(dim)
        d[i] = 1.0
        A = cross_product_operator(d)
        for j in 1:N
            v = normalize(exp(A*j*2*pi/N)*v)
            add = true
            for vp in vecs
                if norm(v-vp) < epsilon
                    add = false
                    break
                end
            end
            if add
                @show v
                push!(vecs,v)
            end
        end
        v = d
    end
    PolyhedronOverapprox(tuple(vecs...))
end

export
    equatorial_overapprox_model,
    ngon_overapprox_model

"""
    equatorial_overapprox_model(lat_angles=[-π/4,0.0,π/4],lon_angles=collect(0:π/4:2π),epsilon=0.1)

Returns a PolyhedronOverapprox model generated with one face for each
combination of pitch and yaw angles specified by lat_angles and lon_angles,
respectively. There are also two faces at the two poles
"""
function equatorial_overapprox_model(lat_angles=[-π/4,0.0,π/4],lon_angles=collect(0:π/4:2π),epsilon=0.1)
    vecs = Vector{SVector{3,Float64}}()
    push!(vecs,[0.0,0.0,1.0])
    push!(vecs,[0.0,0.0,-1.0])
    for phi in lat_angles
        for theta in lon_angles
            v = normalize([
                cos(phi)*cos(theta),
                cos(phi)*sin(theta),
                sin(phi)
            ])
            add = true
            for vp in vecs
                if norm(v-vp) < epsilon
                    add = false
                    break
                end
            end
            if add
                push!(vecs,v)
            end
        end
    end
    PolyhedronOverapprox(tuple(vecs...))
end
"""
    ngon_overapprox_model(lon_angles::Vector{Float64})

2D overapproximation.
"""
function ngon_overapprox_model(lon_angles::Vector{Float64})
    vecs = map(θ->SVector{2,Float64}(cos(θ),sin(θ)),lon_angles)
    PolyhedronOverapprox(tuple(vecs...))
end
ngon_overapprox_model(step::Float64=π/4,start=0.0,stop=2π-step) = ngon_overapprox_model(collect(start:step:stop))
ngon_overapprox_model(n::Int,start=0.0) = ngon_overapprox_model(2π/n,start)

function LazySets.overapproximate(lazy_set,model::H,ϵ::Float64=0.0) where {H<:AbstractPolytope}
    hpoly = H()
    for h in constraints_list(model)
        addconstraint!(hpoly,LazySets.HalfSpace(h.a, ρ(h.a, lazy_set)+ϵ))
    end
    hpoly
end
LazySets.overapproximate(lazy_set,m::PolyhedronOverapprox,args...) = overapproximate(lazy_set,make_polytope(m),args...)

# Base.convert(::Type{Hyperrectangle{T,V,V}},r::Hyperrectangle) where {T,V,V} = Hyperrectangle(V(r.center),V(r.radius))

for TYPE in (:VPolytope,:HPolytope)
    @eval convert_vec_type(::$TYPE,h::Hyperrectangle) = Hyperrectangle(h.center, h.radius)
end

# function LazySets.overapproximate(pts::AbstractVector,::Type{Hyperrectangle},ϵ::Float64=0.0)
#     V = eltype(pts)
#     high = -Inf*ones(V)
#     low = Inf*ones(V)
#     for v in pts
#         high = max.(high,v)
#         low = min.(low,v)
#     end
#     ctr = (high .+ low) / 2
#     widths = (high .- low) / 2
#     Hyperrectangle(ctr,widths .+ (ϵ / 2))
# end
# function LazySets.overapproximate(p::H,model::Type{Hyperrectangle},ϵ::Float64=0.0) where {H<:AbstractPolytope}#{V,T,H<:$TYPE{T,V}}
#     overapproximate(vertices_list(p),model)
# end

"""
    extract_points_and_radii(lazy_set)

Returns an iterator over points and radii.
"""
extract_points_and_radii(n::GeometryBasics.Ngon) = zip(GeometryBasics.coordinates(n),Base.Iterators.repeated(0.0))

function extract_points_and_radii(n::GeometryBasics.Cylinder,c=16)
    # Approximate, because there's no other way to account for the cylinder's flat top and bottom
    v = normalize(n.extremity - n.origin)
    θ_range = 0.0:(2π/c):(2π*(c-1)/c)
    extract_points_and_radii(Base.Iterators.flatten([
        map(θ->SVector(n.origin+normalize(cross(SVector{3,Float64}(cos(θ),sin(θ),0.0),v))*n.r),θ_range),
        map(θ->SVector(n.extremity+normalize(cross(SVector{3,Float64}(cos(θ),sin(θ),0.0),v))*n.r),θ_range),
    ]))
end
extract_points_and_radii(n::Ball2) = [(n.center,n.radius)]
extract_points_and_radii(n::Union{Hyperrectangle,AbstractPolytope}) = zip(LazySets.vertices(n),Base.Iterators.repeated(0.0))
for T in (:AbstractVector,:(Base.Iterators.Flatten),:(Base.Generator))
    @eval extract_points_and_radii(n::$T) = Base.Iterators.flatten(map(extract_points_and_radii,n))
end
extract_points_and_radii(n::SVector) = [(n,0.0)]
extract_points_and_radii(n::GeometryBasics.Point) = [(n,0.0)]
LazySets.dim(::Type{GeometryBasics.Ngon{N,T,M,P}}) where {N,T,M,P} = N
LazySets.dim(::GeometryBasics.Ngon{N,T,M,P}) where {N,T,M,P} = N
LazySets.dim(::Vector{GeometryBasics.Ngon{N,T,M,P} where {T,M,P}}) where {N} = N
LazySets.dim(::Type{G}) where {N,T,G<:GeometryBasics.GeometryPrimitive{N,T}} = N
LazySets.dim(n::GeometryBasics.GeometryPrimitive) = LazySets.dim(typeof(n))
# LazySets.dim(n::AbstractVector{G}) where {G<:GeometryBasics.GeometryPrimitive} = LazySets.dim(en)
LazySets.dim(::GeometryBasics.Point{N,T}) where {N,T} = N
LazySets.dim(::SVector{N,T}) where {N,T} = N
LazySets.dim(::Type{GeometryBasics.Point{N,T}}) where {N,T} = N
LazySets.dim(::Type{SVector{N,T}}) where {N,T} = N
LazySets.dim(::Type{Ball2{T,V}}) where {T,V} = LazySets.dim(V)
LazySets.dim(::Type{Hyperrectangle{T,U,V}}) where {T,U,V} = LazySets.dim(V)
LazySets.dim(::AbstractVector{V}) where {V} = LazySets.dim(V)
LazySets.dim(it::Base.Iterators.Flatten) = LazySets.dim(collect(Base.Iterators.take(it,1))[1])
LazySets.dim(it::Base.Generator) = LazySets.dim(collect(Base.Iterators.take(it,1))[1])
# LazySets.dim(::V) where {U<:AbstractGeometry,V<:AbstractVector{U}} = LazySets.dim(U)

Base.convert(::Type{Ball2{T,V}},b::Ball2) where {T,V} = Ball2(V(b.center),T(b.radius))

"""
    overapproximate_sphere(points_and_radii,N=3,ϵ=0.0)

The bounding sphere ought to be constructed instead by iteratively adding points
and updating the sphere.
"""
function overapproximate_sphere(points_and_radii,N=3,ϵ=0.0)
    # ball = nothing
    # for (pt,r) in points_and_radii
    #     if ball === nothing
    #         ball = Ball2(SVector(pt...),r+ϵ)
    #     else
    #         # THIS IS WRONG
    #         vec = pt - get_center(ball)
    #         if norm(vec) + r + ϵ > get_radius(ball)
    #             diam = get_radius(ball) + norm(vec) + r + ϵ
    #             c = get_center(ball) + (diam/2-get_radius(ball))*normalize(vec)
    #             ball = Ball2(SVector(c...),diam/2)
    #         end
    #     end
    # end
    model = Model(default_optimizer())
    set_optimizer_attributes(model,default_optimizer_attributes()...)
    @variable(model,v[1:N])
    @variable(model,d >= 0.0)
    @objective(model,Min,d)
    list_is_empty = true
    for (pt,r) in points_and_radii
        list_is_empty = false
        @constraint(model,[(d-r-ϵ),map(i->(v[i]-pt[i]),1:N)...] in SecondOrderCone())
    end
    if list_is_empty
        # return nothing
        return Ball2(SVector(zeros(N)...),0.0)
    end
    optimize!(model)
    ball = Ball2(SVector{N,Float64}(value.(v)),max(0.0,value(d)))
    return ball
end
function overapproximate_cylinder(points_and_radii,ϵ=0.0)
    N = 2
    b = overapproximate_sphere(points_and_radii,N,ϵ)
    zhi = -Inf
    zlo = Inf
    for (pt,r) in points_and_radii
        if pt[3] > zhi
            zhi = pt[3]
        end
        if pt[3] < zlo
            zlo = pt[3]
        end
    end
    # get z values
    return GeometryBasics.Cylinder(
        GeometryBasics.Point(b.center[1],b.center[2],zlo),
        GeometryBasics.Point(b.center[1],b.center[2],zhi),
        b.radius)
end
# function overapproximate_hyperrectangle(points_and_radii,ϵ=0.0)
#     high = nothing
#     low = nothing
#     # high = -Inf*ones(V)
#     # low = Inf*ones(V)
#     for (pt,r) in points_and_radii
#         if high === nothing
#             high = pt .+ r
#             low = pt .- r
#         else
#             high = max.(high,pt .+ r)
#             low = min.(low,pt .- r)
#         end
#     end
#     if high === nothing
#         return nothing
#     end
#     ctr = (high .+ low) / 2
#     widths = (high .- low) / 2
#     Hyperrectangle(ctr,widths .+ (ϵ / 2))
# end


for T in (:AbstractPolytope,:LazySet,:AbstractVector,:(GeometryBasics.Ngon),:Any)
    @eval begin
        function LazySets.overapproximate(lazy_set::$T,::Type{H},ϵ::Float64=0.0,N = LazySets.dim(H)) where {H<:Ball2}
            return convert(H,overapproximate_sphere(extract_points_and_radii(lazy_set),N,ϵ))
        end
        function LazySets.overapproximate(lazy_set::$T,::Type{H},ϵ::Float64=0.0) where {A,V<:AbstractVector,H<:Hyperrectangle{A,V,V}}
            # overapproximate_hyperrectangle(extract_points_and_radii(lazy_set),ϵ)
            high = -Inf*ones(V)
            low = Inf*ones(V)
            for (pt,r) in extract_points_and_radii(lazy_set)
                high = max.(high,pt .+ r)
                low = min.(low,pt .- r)
            end
            ctr = (high .+ low) / 2
            widths = (high .- low) / 2
            if any(widths .< 0)
                ctr = zeros(V)
                widths = zeros(V)
            end
            Hyperrectangle(ctr,widths .+ (ϵ / 2))
        end
        function LazySets.overapproximate(lazy_set::$T,m::Type{C},ϵ::Float64=0.0) where {C<:GeometryBasics.Cylinder}
            overapproximate_cylinder(extract_points_and_radii(lazy_set),ϵ)
        end
        function LazySets.overapproximate(lazy_set::$T,m::BufferedPolygon,ϵ::Float64=0.0)
            compute_buffered_polygon(
                [h.a for h in m.halfspaces],
                m.min_face_length,
                extract_points_and_radii(lazy_set),
                ϵ)
        end
        function LazySets.overapproximate(lazy_set::$T,m::BufferedPolygonPrism,ϵ::Float64=0.0)
            compute_buffered_polygon_prism(
                [h.a for h in m.p.halfspaces],
                m.p.min_face_length,
                extract_points_and_radii(lazy_set),
                ϵ)
        end
    end
end
function LazySets.overapproximate(lazy_set::Ball2,model::Type{H},ϵ::Float64=0.0) where {A,V<:AbstractVector,H<:Hyperrectangle{A,V,V}}
    ctr = lazy_set.center
    r = lazy_set.radius + ϵ
    Hyperrectangle(V(ctr),r*ones(V))
end

function LazySets.overapproximate(lazy_set,sphere::H,ϵ::Float64=0.0) where {V,T,H<:Ball2{T,V}}
    r = 0.0
    for (pt,rad) in extract_points_and_radii(lazy_set)
        r = max(norm(pt-get_center(sphere))+rad+ϵ, r)
    end
    Ball2(V(get_center(sphere)),T(r))
end



export
    GridDiscretization,
    GridOccupancy

struct GridDiscretization{N,T}
    origin::SVector{N,T}
    discretization::SVector{N,T}
end
get_hyperrectangle(m::GridDiscretization,idxs) = Hyperrectangle(m.origin .+ idxs.*m.discretization, [m.discretization/2...])
"""
    cell_indices(m::GridDiscretization,v)

get indices of cell of `m` in which `v` falls
"""
cell_indices(m::GridDiscretization,v) = SVector(ceil.(Int,(v .- m.origin .- m.discretization/2)./m.discretization)...)
struct GridOccupancy{N,T,A<:AbstractArray{Bool,N}}
    grid::GridDiscretization{N,T}
    occupancy::A
    offset::SVector{N,Int}
end
GridOccupancy(m::GridDiscretization{N,T},o::AbstractArray) where {N,T} = GridOccupancy(m,o,SVector(zeros(Int,N)...))
Base.:(+)(o::GridOccupancy,v) = GridOccupancy(o.grid,o.occupancy,SVector(o.offset.+v...))
Base.:(-)(o::GridOccupancy,v) = o+(-v)
get_hyperrectangle(m::GridOccupancy,idxs) = get_hyperrectangle(m.grid,idxs .+ m.offset)
function Base.intersect(o1::G,o2::G) where {G<:GridOccupancy}
    offset = o2.offset - o1.offset
    starts = max.(1,offset .+ 1)
    stops = min.(SVector(size(o1.occupancy)),size(o2.occupancy) .+ offset)
    idxs = CartesianIndex(starts...):CartesianIndex(stops...)
    overlap = o1.occupancy[idxs] .* o2.occupancy[idxs .- CartesianIndex(offset...)]
    G(o1.grid,overlap,o2.offset)
end
has_overlap(o1::G,o2::G) where {G<:GridOccupancy} = any(intersect(o1,o2).occupancy)
LazySets.is_intersection_empty(o1::G,o2::G) where {G<:GridOccupancy} = !has_overlap(o1,o2)
function LazySets.overapproximate(o::GridOccupancy,::Type{Hyperrectangle})
    origin = o.grid.origin
    start = findnext(o.occupancy,CartesianIndex(ones(Int,size(origin))...))
    finish = findprev(o.occupancy,CartesianIndex(size(o.occupancy)...))
    s = get_hyperrectangle(o,start.I .- 1)
    f = get_hyperrectangle(o,finish.I .- 1)
    ctr = (s.center .+ f.center) / 2
    radii = (f.center .- s.center .+ o.grid.discretization) / 2
    Hyperrectangle(ctr ,radii)
end
function LazySets.overapproximate(lazy_set,grid::GridDiscretization)
    rect = overapproximate(lazy_set,Hyperrectangle)
    starts = LazySets.center(rect) .- radius_hyperrectangle(rect)
    stops = LazySets.center(rect) .+ radius_hyperrectangle(rect)
    @show start_idxs = cell_indices(grid,starts)
    @show stop_idxs = cell_indices(grid,stops)
    @show offset = SVector(start_idxs...) .- 1
    occupancy = falses((stop_idxs .- offset)...)
    approx = GridOccupancy(grid,occupancy,offset)
    for idx in CartesianIndices(occupancy)
        r = get_hyperrectangle(approx,[idx.I...])
        if !is_intersection_empty(lazy_set,r)
            approx.occupancy[idx] = true
        end
    end
    GridOccupancy(grid,occupancy,offset)
end

# select robot carry locations
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
function construct_support_placement_aggregator(pts, n,
        f_neighbor=v->1.0*minimum(v)+(0.5/n)*sum(v),
        f_inner=v->(0.1/(n^2))*minimum(v)
    )
    D = [norm(v-vp) for (v,vp) in Base.Iterators.product(pts,pts)]
    d_neighbor = (idxs)->f_neighbor(
        map(i->wrap_get(D,(idxs[i],wrap_get(idxs,i+1))),1:length(idxs))
        )
    d_inner = (idxs)->f_inner(
        [wrap_get(D,(i,j)) for (i,j) in Base.Iterators.product(idxs,idxs)]
        )
    d = (idxs)->d_neighbor(idxs)+d_inner(idxs)
end

"""
    spaced_neighbors(polygon,n::Int,aggregator=sum)

Return the indices of the `n` vertices of `polygon` whose neighbor distances
maximize the utility metric defined by `aggregator`. Uses local optimization,
so there is no guarantee of global optimality.
"""
function spaced_neighbors(polygon::AbstractPolygon, n::Int,
        score_function=construct_support_placement_aggregator(vertices_list(polygon), n),
        ϵ=1e-8)
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
        for deltas in Base.Iterators.product(map(i->(-1,0,1),1:n)...)
            idxs = sort(map(i->wrap_idx(length(pts),i),best_idxs .+ deltas))
            if length(unique(idxs)) == n
                if score_function(idxs) > d_hi + ϵ
                    best_idxs = map(i->wrap_idx(length(pts),i),idxs)
                    d_hi = score_function(idxs)
                    push!(idx_list,best_idxs)
                    # @info "best_idxs = $best_idxs, d_hi = $d_hi"
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
function extremal_points(pts)
    D = [norm(v-vp) for (v,vp) in Base.Iterators.product(pts,pts)]
    val, idx = findmax(D)
    return val, idx.I
end
proj_to_line(v,vec) = vec*dot(v,vec)/(norm(vec)^2)
function proj_to_line_between_points(p,p1,p2)
    v = p.-p1
    vec = normalize(p2-p1)
    p1 .+ proj_to_line(v,vec)
end
perimeter(pts) = sum(map(v->norm(v[1]-v[2]),zip(pts[1:end-1],pts[2:end]))) #sum(map(i->norm(wrap_get(pts,(i,i+1))),1:length(pts)))
perimeter(p::LazySets.AbstractPolygon) = perimeter(vertices_list(p))
get_pts(p::LazySets.AbstractPolytope) = vertices_list(p)
get_pts(v::AbstractVector) = v

"""
    select_support_locations(polygon, r)

Select support locations where options are the vertices of `polygon` and the
robot radius is `r`.
"""
function select_support_locations(polygon::LazySets.AbstractPolygon, r;
        score_function_constructor=construct_support_placement_aggregator,
    )
    n = select_num_robots(polygon, r)
    if n == 1
        sphere = overapproximate(polygon,Ball2{Float64,SVector{2,Float64}})
        support_pts = [LazySets.center(sphere)]
    else
        pts = vertices_list(polygon)
        pts = reduce_to_valid_candidate_list(pts, r)
        n = min(n, length(pts)) # Can cannot be greater than the number of valid points
        reduced_polygon = VPolygon(pts)
        @assert length(pts) >= n "length(pts) = $(length(pts)), but n = $n"

        score_function=score_function_constructor(pts, n)
        best_idxs, _ = spaced_neighbors(reduced_polygon, n, score_function)
        if length(best_idxs) < n
            @warn "$n support points requested, but only $(length(best_idxs)) returned for polygon with $(length(pts)) vertices"
        end
        support_pts = pts[best_idxs]
    end
    support_pts
end

function reduce_to_valid_candidate_list(pts, r; ϵ=r)
    pt_candidates = pts
    for pt in pts
        if !(pt in pt_candidates)
            continue
        end
        dist_to_others = norm.([pt] .- pt_candidates)
        bool_mask = dist_to_others .> (2 * r + ϵ)
        self_idx = findfirst([pt] .== pt_candidates)
        bool_mask[self_idx] = true

        pt_candidates = pt_candidates[bool_mask]
        @assert !isempty(pt_candidates) "Starting point should always be an option"
    end
    return pt_candidates
end

"""
    select_num_robots(polygon,r)

polygon: the polygon whose vertices are eligible locations for placing robots
r: the robot radius
"""
function select_num_robots(polygon,r)
    # Extract length and width from spectral value decomposition
    pts = vertices_list(polygon)
    _,S,_ = svd(hcat(convert.(Vector, pts)...))
    L = S[1] # length
    W = S[2] # width

    neighbor_dists = map(i->norm(pts[i]-pts[i+1]),1:length(pts)-1)
    push!(neighbor_dists,norm(pts[end]-pts[1]))
    N = length(pts) - length(findall(neighbor_dists .< 2*r))
    #
    p = perimeter(polygon)
    # A = LazySets.area(polygon)
    # # V = A*H # volume = area * height
    # # Choose the number of robots
    # n = max(1, min(Int(floor(p/(π*r))), 2))
    # if W >= 2*r
    #     # L > W, so p is at least (2+2+2√2)r
    #     n = 3 # Can fit at least three robots
    #     # Compute upper bound on the number of robots
    #     N = length(pts)
    #     n_max = min(N, Int(floor(p/(π*r))))
    #     # @assert n_max >= 1 "N=$N, L=$L, W=$W, p=$p, n=$n, pts=$pts"
    #     n = min(n_max, 4) # try jumping up to 4
    #     # Now impose a lower bound based on sqrt(perimeter), so n will increase
    #     # more slowly as p grows.
    #     n = min(n_max, Int(floor( 2*sqrt(p/(π*r)) )))
    # end
    # TRY THIS (and get rid of everything else above)
    x = p / (π * r)
    if W >= 2*r
    #   n = Int(floor(max(1, min(length(pts), min(x, 2*sqrt(x))))))
      n = Int(floor(max(1, min(N, min(x, 2*sqrt(x))))))
    else # long and skinny
      n = Int(floor(max(1, min(x, 2))))
    end
    @info "number of robots in transport unit: $(n)"
    return n
end
# select_support_locations(p::AbstractPolytope,args...) = select_support_locations(vertices_list(p),args...)

# spread out sub assemblies

"""
    compute_hierarchical_2d_convex_hulls(scene_tree)

Compute the 2d projected convex hull of each `ObjectNode` and each
`AssemblyNode` in scene_tree.
"""
function compute_hierarchical_2d_convex_hulls(scene_tree)
    cvx_hulls = Dict{AbstractID,Vector{GeometryBasics.Point{2,Float64}}}()
    for v in reverse(Graphs.topological_sort_by_dfs(scene_tree))
        node = get_node(scene_tree,v)
        if matches_template(ObjectNode,node)
            pts = map(HierarchicalGeometry.project_to_2d, GeometryBasics.coordinates(get_base_geom(node)))
            cvx_hulls[node_id(node)] = convex_hull(pts)
        elseif matches_template(AssemblyNode,node)
            pts = nothing
            for (child_id,tform) in assembly_components(node)
                composite_tform = HierarchicalGeometry.project_to_2d ∘ tform ∘ HierarchicalGeometry.project_to_3d
                child_pts = map(composite_tform, cvx_hulls[child_id])
                if pts === nothing
                    pts = child_pts
                else
                    append!(pts,child_pts)
                end
            end
            cvx_hulls[node_id(node)] = convex_hull(pts)
        end
    end
    cvx_hulls
end

"""
    mesh_solid_by_marching_cubes(geom,cube_size,dims)

Compute a voxel-based approximation of `geom`'s volume, with voxels of size
`cube_size`.
"""
function mesh_solid_by_marching_cubes(f,cube_size,dims)
    indicator_tensor = falses(dims)
    explored = falses(dims)
    hit_counts = zeros(Int,dims) # number of f == 1 along a path to a given voxel
    frontier = Set{Tuple{NTuple{length(dims),Int}},Int}() # (idxs,hit_count)
    push!(frontier,tuple(ones(length(dims))))
    while length(frontier)
        (idxs, hit_count) = pop!(frontier)
        # check for intersection
        if f(idxs,cube_size) > 0 # geometry intersected
            # if intersected, are we leaving or entering?
            hit_counts[idxs] = hit_counts[idxs]
        end
    end
end
