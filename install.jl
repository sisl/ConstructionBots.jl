import Pkg: Pkg, PackageSpec

Pkg.activate(".")

packages = [
    PackageSpec(path="deps/GraphUtils.jl"),
    PackageSpec(path="deps/HierarchicalGeometry.jl"),
    PackageSpec(path="deps/GraphPlottingBFS.jl")
]

Pkg.develop(packages)

Pkg.instantiate()
Pkg.build()
Pkg.precompile()

# include("deps/GraphPlottingBFS.jl")

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
