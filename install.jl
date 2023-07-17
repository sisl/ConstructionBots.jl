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

Pkg.instantiate()
Pkg.build()
Pkg.precompile()

# TODO: Check for RVO2 python access. If not, can we automate the build process here?
# using PyCall
# rvo = pyimport("rvo2")
