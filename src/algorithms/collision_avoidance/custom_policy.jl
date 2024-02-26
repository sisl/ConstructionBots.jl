# Use this file to implement your custom deconfliction algorithm
export CustomPolicy

@with_kw mutable struct CustomPolicy <: DeconflictStrategy
    name::String = "CustomName"
    # Add your fields here
end

function update_twists_with_deconfliction(policy::CustomPolicy, params_to_update)
    # Implement your method updating an agent's velocity at a single timestep
end
