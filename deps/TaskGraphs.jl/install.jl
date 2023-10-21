using Pkg

packages = [
    # Unregistered dependency.
    PackageSpec(url="https://github.com/sisl/GraphUtils.jl"),

    PackageSpec(url="https://github.com/sisl/CRCBS.jl"),

    # "This" package.
    PackageSpec(url=joinpath(@__DIR__)),
]

if haskey(ENV, "CI") && ENV["CI"] == "true"
    # remove "own" package when on CI
    pop!(packages)
end

# Run dev altogether
# This is important that it's run together so there
# are no "expected pacakge X to be registered" errors.
Pkg.develop(packages)