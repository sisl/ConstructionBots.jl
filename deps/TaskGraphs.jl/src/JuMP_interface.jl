export
    default_milp_optimizer,
    set_default_milp_optimizer!,
    default_optimizer_attributes,
    set_default_optimizer_attributes!,
    clear_default_optimizer_attributes!

global MILP_OPTIMIZER = nothing
global DEFAULT_OPTIMIZER_ATTRIBUTES = Dict{Union{String,MOI.AbstractOptimizerAttribute},Any}()
"""
    default_milp_optimizer()

Returns the black box optimizer to be use when formulating JuMP models.
"""
default_milp_optimizer() = MILP_OPTIMIZER
"""
    set_default_milp_optimizer!(optimizer)

Set the black box optimizer to be use when formulating JuMP models.
"""
function set_default_milp_optimizer!(optimizer)
    global MILP_OPTIMIZER = optimizer
end

"""
    default_optimizer_attributes()

Return a dictionary of default optimizer attributes.
"""
default_optimizer_attributes() = DEFAULT_OPTIMIZER_ATTRIBUTES

"""
    set_default_optimizer_attributes!(vals)

Set default optimizer attributes.
e.g. `set_default_optimizer_attributes!(Dict("PreSolve"=>-1))`
"""
function set_default_optimizer_attributes!(pair::Pair,pairs...)
    push!(DEFAULT_OPTIMIZER_ATTRIBUTES,pair)
    set_default_optimizer_attributes!(pairs...)
end
set_default_optimizer_attributes!(d::Dict) = set_default_optimizer_attributes!(d...)
set_default_optimizer_attributes!() = nothing

"""
    clear_default_optimizer_attributes!()

Clear the default optimizer attributes.
"""
function clear_default_optimizer_attributes!() 
    empty!(DEFAULT_OPTIMIZER_ATTRIBUTES)
end