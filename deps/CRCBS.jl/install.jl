using Pkg

packages = [
    # Unregistered dependency.
    PackageSpec(url="https://github.com/sisl/GraphUtils.jl"),
]

Pkg.add(packages)
