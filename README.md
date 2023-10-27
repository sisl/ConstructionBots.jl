# ConstructionBots

## TODO: Fix Badges
[![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://sisl.github.io/ConstructionBots.jl/stable)
[![Build Status](https://github.com/sisl/ConstructionBots.jl/workflows/CI/badge.svg)](https://github.com/sisl/ConstructionBots.jl/actions)

### Under Construction

ConstructionBots.jl is an open-source multi-robot manufacturing simulator and is designed to test algorithms for multi-robot assembly planning. This system approaches multi-robot assembly planning from a higher level of abstraction and addresses task planning and transit planning but abstracts away the kino-dynamic details of piecing together assemblies. Problems addressed and simulated:
- **Trasport Team configuration:** How many robots are needed and how should robots be positioned when transporting a particular payload?
- **Spatial layout of the construction site:** Where will each assembly be built, and where will the components of those assemblies be delivered?
- **Sequential task allocation and team forming:** Which robots will collect and deliver which payloads? Wince there are generally far more payloads than robots, individual robots generally have to transport multiple payloads in sequence.
- **Collision avoidance with heterogeneous agent geometry and dynamics:** How must laden and unloaden robots and robot teams move, subject to motion constraints that depend on the payload size and team configuration, to avoid collision with other robots and the active construction sites in the environment?

## Installation

### LDraw Parts Library
Individual part geometry is needed for all features. The parts library can be downloaded from [LDraw™ Parts Library](https://library.ldraw.org/updates?latest). Place the unzipped library in your desired path. The default path assumed by LDrawParser is `joinpath(homedir(), "Documents/ldraw")`. It is recommended to download the complete library (~80 MB zipped, ~450 MB unzipped).

If you did not place the parts library in the default path, you can change the path LDrawParser uses by the `set_part_library_dir!` command.

### RVO


### ConstructionBots


## Usage

## Citation
Under review