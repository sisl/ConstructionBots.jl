using ConstructionBots
using Documenter

makedocs(;
    modules=[ConstructionBots],
    authors="kylebrown <kylejbrown17@gmail.com> and contributors",
    repo="https://github.com/kylejbrown17/ConstructionBots.jl/blob/{commit}{path}#L{line}",
    sitename="ConstructionBots.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://kylejbrown17.github.io/ConstructionBots.jl",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
    ],
)

deploydocs(;
    repo="github.com/kylejbrown17/ConstructionBots.jl",
)
