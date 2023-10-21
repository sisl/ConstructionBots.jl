abstract type TransformConstraint end

""" 
    abstract type TranslationConstraint <: TransformConstraint 

Abstract constraint on translation.
"""
struct TranslationConstraint{C} <: TransformConstraint
    constraint::C
end

""" 
    RotationConstraint{C}

`c::RotationConstraint` represents a constraint `c.constraint::C` to be applied 
to column `c.axis` of the rotation matrix.
"""
struct RotationConstraint{C} <: TransformConstraint
    axis::Int # 1 = x, 2 = y, 3 = 7
    constraint::C
end

"""
    CoPlanar{N}

Constrain vector `v` to be coplanar with `c::CoPlanar`
    `v⋅c.h == c.b`
"""
struct CoPlanar{N}
    h::SVector{N,Float64} # normal vector
    b::Float64
end

"""
    CoLinear{N}

Constrain vector `v` to be colinear with `c::CoLinea`
    `v⋅c.h == norm(v)`
"""
struct CoLinear{N}
    h::SVector{N,Float64}
end

"""
    TransFormConstraintModel{N}

`m::TransFormConstraintModel` stores a JuMP model `m.model` for computing 
translation `m::v` and orientation `m::R` against `m.constraints`.
"""
struct TransFormConstraintModel{N}
    model::JuMP.Model
    v::Vector{JuMP.VariableRef}
    R::Matrix{JuMP.VariableRef}
    constraints::Set{TransformConstraint}
end

function init_3d_transform_constraint_model(c_list...)
    model = JuMP.Model(default_optimizer())
    set_optimizer_attributes(model,default_optimizer_attributes()...)
    @variable(model, v[1:3])
    @variable(model, R[1:3,1:3])
    # orthogonality (nevermind -- not convex)
    @constraint(model, dot(R[1:3,1],R[1:3,2]) == 0.0)
    TransFormConstraintModel{3}(
        model,v,R,Set{TransformConstraint}([c_list...])
    )
end
