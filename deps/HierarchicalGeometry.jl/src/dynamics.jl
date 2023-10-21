inertia_of_point(p,m,Id=one(SMatrix{3,3,Float64})) = m*(dot(p,p)*Id .- p*p')

"""
    inertia_from_points(pts,masses)

Compute the inertia matrix of a set of points
"""
function inertia_from_points(pts,masses)
    Id = one(SMatrix{3,3,Float64})
    inertia = sum(m*(dot(p,p)*Id .- p*p') for (p,m) in zip(pts,masses))
end

"""
    com_from_points(pts,masses)

Center of mass from points.
"""
com_from_points(pts,masses) = sum(m*p for (p,m) in zip(pts,masses))/sum(masses)

"""
    parallel_axis_3d(p,m)

Determines the additional inertia matrix terms created by a translation `p`.
"""
function parallel_axis_3d(p,m)
    Id = one(SMatrix{3,3,Float64})
    return m*(dot(p,p)*Id .- 2*p'*Id*p .+ m*p*p')
end

parallel_axis_1d(p::Real,m) = m*p^2
parallel_axis_1d(p,m) = parallel_axis_1d(norm(p),m)
parallel_axis_skew(x,y,m) = m*x*y

"""
    rotate_inertia(inertia,R)

Transform inertia tensor by R, where R is the rotation transform from the
inertia's current frame to the new frame.
"""
rotate_inertia(inertia::A,R) where {A} = A(R*inertia*R')

"""
    rotate_and_transform_inertia(inertia,t,R)

first transforms the inertia into the correct frame, then applies
parallel_axis_3d with the translation.
"""
function rotate_and_translate_inertia(inertia::A,m,p,R) where {A}
    rotate_inertia(inertia,R) .+ parallel_axis_3d(p,m)
end

"""
    inertia_of_triangular_plate(plate,planar_density)

Computes the intertia of a triangular plate
# p1-p2 is the longest leg
"""
function inertia_of_triangular_plate(pts,ρ)
    # integrate over all points
    A = GeometryBasics.area(pts)    # area
    m = ρ * A                       # mass
    COM = sum(pts) / length(pts)    # center of mass
    p1,p2,p3 = pts
    # begin by defining the base
    v12 = p2-p1 # x-axis : this is the longest side
    v23 = p3-p2
    v13 = p3-p1

    x_vec = normalize(v12)   # base vector
    y_vec = normalize(v13 - project_onto_vector(v13,v12)) # height vector
    z_vec = normalize(cross(normalize(x_vec),normalize(y_vec))) # out of plane vector

    Icxx = inertia_of_triangle_about_axis(pts,x_vec,ρ,COM,m)
    Icyy = inertia_of_triangle_about_axis(pts,y_vec,ρ,COM,m)
    # Icxy = inertia_of_triangle_about_axis(pts,normalize(x_vec .+ y_vec),ρ,COM,m)
    Icxy = skew_inertia_of_triangle_about_axes(pts,x_vec,y_vec,ρ,COM,m)

    Izz = inertia_of_triangle_out_of_plane(pts,ρ,COM,m)


    inertia = [
        Icxx    -Icxy   0.0;
        -Icxy   Icyy    0.0;
        0.0     0.0     Izz;
    ]
    R = hcat(x_vec,y_vec,z_vec)
    @show rotate_inertia(inertia,R)
    inertia
    # rotate_inertia(inertia,inv(R))
end

"""
    inertia_of_triangle_about_axis(pts,vec,ρ,

Compute inertia of triangle defined by pts about axis vec, with density ρ.
"""
function inertia_of_triangle_about_axis(pts,vec,ρ,
        COM=sum(pts)/3,
        m = ρ*GeometryBasics.area(pts),
        ;
        about_com::Bool = true,
        )
    @assert isapprox(norm(vec),1.0)
    i = argmin(map(p->dot(p,cross(GeometryBasics.Point(0.0,0.0,1.0),vec)),pts))
    origin = pts[i]
    com = COM-origin
    pts_sorted = sort(pts,by=p->dot(vec,project_onto_vector(p,vec))) .- origin
    proj_vecs = map(p->project_onto_vector(p,vec),pts_sorted)
    x = map(p->dot(p,vec),proj_vecs)
    h = [norm(pt-p) for (pt,p) in zip(pts_sorted,proj_vecs)]
    # compute inertia of trapezoidal sections
    I1 = inertia_of_vertical_trapezoid_about_base(h[1],h[2],x[2]-x[1],ρ)
    I2 = inertia_of_vertical_trapezoid_about_base(h[1],h[3],x[3]-x[1],ρ)
    I3 = inertia_of_vertical_trapezoid_about_base(h[2],h[3],x[3]-x[2],ρ)
    It = I2 - (I1 + I3)
    # parallel axis adjustment
    if about_com == true
        Ipa = -parallel_axis_1d(com-project_onto_vector(com,vec),m)
    else
        Ipa = 0.0
    end
    if It < 0
        return -It + Ipa
    end
    return It + Ipa
end
function skew_inertia_of_triangle_about_axes(pts,vec1,vec2,ρ,
        COM=sum(pts)/3,
        m = ρ*GeometryBasics.area(pts),
        )
    @assert isapprox(norm(vec1),1.0) && isapprox(norm(vec2),1.0)
    A,B,C = pts
    a = norm(B-C)
    b = norm(A-C)
    c = norm(A-B)
    # inertia of first right triangle
    D = A + project_onto_vector(C-A,B-A)
    d = norm(D-A) # projection of C onto A B
    h = norm(C-D)
    m1 = ρ*(1/2)*d*h
    com1 = A + (2/3)*(D-A) + (1/3)*(C-D)
    I1 = skew_inertia_of_right_triangle_about_corner(d,h,ρ) - parallel_axis_skew(dot(com1-A,vec1),dot(com1-A,vec2),m1)
    # inertia of second right triangle
    m2 = ρ*(1/2)*(b-d)*h
    com2 = D + (1/3)*(B-D) + (1/3)*(C-D)
    I2 = skew_inertia_of_right_triangle_about_base_corner(c-d,h,ρ) - parallel_axis_skew(dot(com2-D,vec1),dot(com2-D,vec2),m2)

    dc1 = com1-COM
    dc2 = com2-COM
    I3 = I1 + parallel_axis_skew(dot(dc1,vec1),dot(dc1,vec2),m1)
        + I2 + parallel_axis_skew(dot(dc2,vec1),dot(dc2,vec2),m2)

end

function inertia_of_vertical_trapezoid_about_base(h1,h2,d,ρ)
    a = h1
    if isapprox(d,0)
        return 0.0
    end
    c = (h2-h1) / d
    (1/3)*ρ*( d*a^3 + (3/2)*c*(a*d)^2 + a*(c^2)*(d^3) + (1/4)*(c^3)*d^4 )
end

function inertia_of_right_triangle_out_of_plane_about_base(α,b,ρ)
    Izz = ρ*((b^4)/24)*((3*sin(α)+sin(3*α))/(cos(α)^3))
end

"""
    inertia_of_triangle_out_of_plane(pts,ρ,

Returns centroidal moment of inertia.
"""
function inertia_of_triangle_out_of_plane(pts,ρ,
        COM=sum(pts)/length(pts),
        m=GeometryBasics.area(pts)*ρ,
    )
    # orient triangle
    dists = [norm(pts[i]-pts[j]) for i in 1:3, j in 1:3]
    idxs = sortperm([sum(dists,dims=1)...])
    # idxs = [2,3,1]
    A = pts[idxs[3]]
    B = pts[idxs[2]]
    C = pts[idxs[1]]
    a = norm(B-C)
    b = norm(A-C)
    c = norm(A-B)
    # inertia of first right triangle
    α1 = find_angle(b,c,a) # law of cosines
    b1 = c*cos(α1)
    C1 = A .+ normalize(C-A)*b1
    com1 = (A .+ B .+ C1) / 3
    m1 = ρ*(1/2)*b1*c*sin(α1)
    I1 = inertia_of_right_triangle_out_of_plane_about_base(α1,b1,ρ) - parallel_axis_1d(com1-A,m1)
    # inertia of second right triangle
    b2 = b - b1
    α2 = atan(c*sin(α1),b2)
    com2 = (C .+ B .+ C1) / 3
    m2 = ρ*(1/2)*b2*b2*tan(α2)
    I2 = inertia_of_right_triangle_out_of_plane_about_base(α2,b2,ρ) - parallel_axis_1d(com2-C,m2)
    # combined inertia
    I3 = I1 + parallel_axis_1d(com1-COM,m1) + I2 + parallel_axis_1d(com2-COM,m2)
end

"""
    find_angle(a,b,c)

Using the law of cosines, find angle at point C (opposite side c) from sides
a, b, and c.
"""
find_angle(a,b,c) = acos(-(c^2 - (a^2 + b^2)) / (2*a*b))

inertia_of_right_triangle_about_base(b,h,ρ)             = ρ*(1/12)*b*h^3
skew_inertia_of_right_triangle_about_corner(b,h,ρ)      = ρ*(1/8)*h*b^3
skew_inertia_of_right_triangle_about_base_corner(b,h,ρ) = ρ*(5/24)*h*b^3
