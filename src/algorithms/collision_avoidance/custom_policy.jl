# Use this file to implement your custom deconfliction algorithm

export CustomPolicy

@with_kw mutable struct CustomPolicy <: DeconflictStrategy
    name::String = "CustomName"
    # Step 1: Add your fields here
end

function update_twists_with_deconfliction(policy::CustomPolicy, params_to_update)
    # Step 2: Implement your method updating an agent's velocity at a single timestep
end

# Step 3: Remember to add your policy to `supported_deconfliction_options` in
# deconfliction_strategy.jl.
