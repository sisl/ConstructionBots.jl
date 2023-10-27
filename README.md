# ConstructionBots

## TODO: Fix Badges
[![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://sisl.github.io/ConstructionBots.jl/stable)
[![Dev](https://img.shields.io/badge/docs-dev-blue.svg)](https://sisl.github.io/ConstructionBots.jl/dev)
[![Build Status](https://github.com/sisl/ConstructionBots.jl/workflows/CI/badge.svg)](https://github.com/sisl/ConstructionBots.jl/actions)

### Under Construction

ConstructionBots.jl is an open-source multi-robot manufacturing simulator and is designed to test algorithms for multi-robot assembly planning. This system approaches multi-robot assembly planning from a higher level of abstraction and addresses task planning and transit planning but abstracts away the kino-dynamic details of piecing together assemblies. Problems addressed and simulated:
- **Trasport Team configuration:** How many robots are needed and how should robots be positioned when transporting a particular payload?
- **Spatial layout of the construction site:** Where will each assembly be built, and where will the components of those assemblies be delivered?
- **Sequential task allocation and team forming:** Which robots will collect and deliver which payloads? Wince there are generally far more payloads than robots, individual robots generally have to transport multiple payloads in sequence.
- **Collision avoidance with heterogeneous agent geometry and dynamics:** How must laden and unloaden robots and robot teams move, subject to motion constraints that depend on the payload size and team configuration, to avoid collision with other robots and the active construction sites in the environment?

## Installation

## Usage

## Citation
Under review