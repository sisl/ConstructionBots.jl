using Documenter
using ConstructionBots

makedocs(
    modules = [ConstructionBots],
    format = Documenter.HTML(),
    sitename = "ConstructionBots.jl",
    checkdocs = :none,
)

deploydocs(
    repo = "github.com/sisl/ConstructionBots.jl",
    versions = ["stable" => "v^", "v#.#"],
)
