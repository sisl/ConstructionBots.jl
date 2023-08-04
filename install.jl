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

# Working process to install rvo2
# conda create -n lego python=3.7 anaconda
# conda activate lego
# conda install pip
# pip install -r requirements.txt

# remove the build directory

# python setup.py build
# python setup.py install

# Set ENV[“PYTHON”] = /home/dylan/miniconda3/envs/lego/bin/python3.7
# Pkg.build(“PyCall”)
# Restarted Julia
