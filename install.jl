import Pkg: Pkg, PackageSpec

Pkg.activate(".")

packages = [
    PackageSpec(url="git@github.com:sisl/CRCBS.jl.git"),
    PackageSpec(url="git@github.com:sisl/GraphUtils.jl.git"),
    PackageSpec(url="git@github.com:sisl/TaskGraphs.jl.git"),
    PackageSpec(url="git@github.com:sisl/HierarchicalGeometry.jl.git"),
    PackageSpec(url="git@github.com:sisl/LDrawParser.jl.git")
]
Pkg.add(packages)

include("deps/GraphPlottingBFS.jl")
include("deps/FactoryRendering.jl")
