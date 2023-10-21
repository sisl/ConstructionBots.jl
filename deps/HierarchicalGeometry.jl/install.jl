import Pkg: Pkg, PackageSpec

Pkg.activate(".")

packages = [
    PackageSpec(url="git@github.com:sisl/GraphUtils.jl.git"),
]
Pkg.add(packages)

Pkg.instantiate()
Pkg.build()
Pkg.precompile()
