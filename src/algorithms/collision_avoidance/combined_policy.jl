@with_kw mutable struct CombinedPolicy <: DeconflictStrategy
    name::String="RVO-TangentBug-Dispersion"
    # Policies can be combined in a specified order:
    policies::Vector{DeconflictStrategy}=[
        ReciprocalVelocityObstacle(),
        TangentBugPolicy(),
        Dispersion(),
    ]
end
