@with_kw mutable struct CustomPolicy <: DeconflictStrategy
    # Add fields for your custom policy
    name::String="CustomPolicy"
    custom_field_1::Float64=0.0
    custom_field_2::Int64=0
end

function perform_twist_deconfliction(CustomPolicy, params)
    # Implement this method
end
