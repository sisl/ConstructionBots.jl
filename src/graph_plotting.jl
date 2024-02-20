
@with_kw struct BFS_state
    d::Int = 0  # current depth
    w::Int = 0  # current width
    d_max::Int = 0  # max depth so far
end

function leaf_case!(G, v, s::BFS_state)
    set_prop!(G, v, :height, 0)  # distance to leaf
    set_prop!(G, v, :depth, s.d)  # distance to root
    set_prop!(G, v, :left, s.w)
    set_prop!(G, v, :width, 1)
    set_prop!(G, v, :right, s.w + 1)
    set_prop!(G, v, :center, (get_prop(G, v, :left) + get_prop(G, v, :right)) / 2.0)
    s = BFS_state(d = s.d, w = s.w + 1, d_max = max(s.d_max, s.d))
end

function initial_branch_case!(G, v, s::BFS_state)
    set_prop!(G, v, :height, 0)  # distance to leaf
    set_prop!(G, v, :left, s.w)
    set_prop!(G, v, :depth, s.d)
    set_prop!(G, v, :edge_count, 0)
    set_prop!(G, v, :center, 0.0)
end

function backup!(G, v, v2, s::BFS_state)
    set_prop!(G, v, :height, max(get_prop(G, v, :height), 1 + get_prop(G, v2, :height)))  # distance to leaf
    set_prop!(G, v, :right, s.w)
    set_prop!(G, v, :width, s.w - get_prop(G, v, :left))
    set_prop!(G, v, :edge_count, 1 + get_prop(G, v, :edge_count))
    c = get_prop(G, v, :center)
    e = get_prop(G, v, :edge_count)
    set_prop!(G, v, :center, c * ((e - 1) / e) + get_prop(G, v2, :center) * (1 / e))
end

step_in(s::BFS_state) = BFS_state(d = s.d + 1, w = s.w, d_max = s.d_max)
step_out(s::BFS_state) = BFS_state(d = s.d - 1, w = s.w, d_max = s.d_max)

function bfs!(G, v, s, visited = Set{Int}(), enforce_visited = false)
    s = step_in(s)
    if indegree(G, v) == 0
        s = leaf_case!(G, v, s)
    else
        initial_branch_case!(G, v, s)
        for v2 in inneighbors(G, v)
            if !(v2 in visited) || !enforce_visited
                s = bfs!(G, v2, s, visited, enforce_visited)
                backup!(G, v, v2, s)
            end
        end
    end
    push!(visited, v)
    return step_out(s)
end

abstract type GraphInfoFeature end
struct ForwardDepth <: GraphInfoFeature end
struct BackwardDepth <: GraphInfoFeature end
struct ForwardWidth <: GraphInfoFeature end
struct BackwardWidth <: GraphInfoFeature end
struct VtxWidth <: GraphInfoFeature end
struct ForwardIndex <: GraphInfoFeature end
struct ForwardCenter <: GraphInfoFeature end
struct BackwardIndex <: GraphInfoFeature end
struct BackwardCenter <: GraphInfoFeature end

"""
    Track the width of the tree above a node, and the parent of that tree
"""
struct ForwardTreeWidth <: GraphInfoFeature end
struct BackwardTreeWidth <: GraphInfoFeature end
initial_value(f::Union{ForwardTreeWidth,BackwardTreeWidth}) = Dict{Int,Int}()

function forward_propagate(::ForwardTreeWidth, G, v, v2, vec)
    for (k, val) in vec[v2]
        vec[v][k] = max(get(vec[v], k, 0), val)
    end
end

function backward_propagate(::BackwardTreeWidth, G, v, v2, vec)
    for (k, val) in vec[v2]
        vec[v][k] = max(get(vec[v], k, 0), val)
    end
end

function forward_accumulate(::ForwardTreeWidth, G, v, vtxs, vec)
    if indegree(G, v) == 0
        vec[v][v] = 1
    end
    return vec[v]
end

function backward_accumulate(::BackwardTreeWidth, G, v, vtxs, vec)
    if outdegree(G, v) == 0
        vec[v][v] = 1
    end
    return vec[v]
end

initial_value(f) = 1.0
initial_value(f::Union{ForwardDepth,BackwardDepth}) = 1
forward_propagate(f, G, v, v2, vec) = vec[v]
backward_propagate(f, G, v, v2, vec) = vec[v]
forward_accumulate(f, G, v, vtxs, vec) = vec[v]
backward_accumulate(f, G, v, vtxs, vec) = vec[v]
backward_propagate(::BackwardDepth, G, v, v2, vec) = max(vec[v], vec[v2] + 1)
forward_propagate(::ForwardDepth, G, v, v2, vec) = max(vec[v], vec[v2] + 1)
forward_propagate(::ForwardWidth, G, v, v2, vec) =
    max(vec[v], vec[v2] / max(1, outdegree(G, v2)))
backward_propagate(::BackwardWidth, G, v, v2, vec) =
    max(vec[v], vec[v2] / max(1, indegree(G, v2)))
forward_accumulate(::ForwardWidth, G, v, vtxs, vec) =
    max(vec[v], sum([0, map(v2 -> vec[v2] / outdegree(G, v2), vtxs)...]))
backward_accumulate(::BackwardWidth, G, v, vtxs, vec) =
    max(vec[v], sum([0, map(v2 -> vec[v2] / indegree(G, v2), vtxs)...]))

function forward_pass!(
    G,
    feats,
    feat_vals = Dict(f => map(v -> initial_value(f), Graphs.vertices(G)) for f in feats),
)
    for v in topological_sort_by_dfs(G)
        for (f, vec) in feat_vals
            vec[v] = forward_accumulate(f, G, v, inneighbors(G, v), vec)
            for v2 in inneighbors(G, v)
                vec[v] = forward_propagate(f, G, v, v2, vec)
            end
        end
    end
    return feat_vals
end

function backward_pass!(
    G,
    feats,
    feat_vals = Dict(f => map(v -> initial_value(f), Graphs.vertices(G)) for f in feats),
)
    for v in reverse(topological_sort_by_dfs(G))
        for (f, vec) in feat_vals
            vec[v] = backward_accumulate(f, G, v, outneighbors(G, v), vec)
            for v2 in outneighbors(G, v)
                vec[v] = backward_propagate(f, G, v, v2, vec)
            end
        end
    end
    return feat_vals
end

function get_graph_layout(
    G,
    feats = [ForwardDepth(), BackwardDepth(), ForwardWidth(), BackwardWidth()];
    enforce_visited = false,
)
    @assert !is_cyclic(G)
    feat_vals = forward_pass!(G, feats)
    feat_vals = backward_pass!(G, feats, feat_vals)
    feat_vals[VtxWidth()] = max.(feat_vals[ForwardWidth()], feat_vals[BackwardWidth()])
    graph = MetaDiGraph(nv(G))
    for e in edges(G)
        add_edge!(graph, e)
    end
    end_vtxs = [v for v in Graphs.vertices(graph) if outdegree(graph, v) == 0]
    s = BFS_state(0, 0, 0)
    for v in end_vtxs
        s = bfs!(graph, v, s, Set{Int}(), enforce_visited)
    end
    for (idx_key, ctr_key, depth_key) in [
        (ForwardIndex(), ForwardCenter(), ForwardDepth()),
        ((BackwardIndex(), BackwardCenter(), BackwardDepth())),
    ]
        vec = feat_vals[depth_key]
        forward_width_counts = map(v -> Int[], 1:maximum(vec))
        forward_idxs = zeros(nv(G))
        backward_idxs = zeros(nv(G))
        for v in topological_sort_by_dfs(G)
            push!(forward_width_counts[vec[v]], v)
        end
        for vec in forward_width_counts
            sort!(vec, by = v -> get_prop(graph, v, :left))
            for (i, v) in enumerate(vec)
                if i > 1
                    v2 = vec[i-1]
                    forward_idxs[v] += forward_idxs[v2] + feat_vals[VtxWidth()][v2]
                end
                if indegree(G, v) > 0
                    min_idx = minimum(map(v2 -> forward_idxs[v2], inneighbors(G, v)))
                    forward_idxs[v] = max(forward_idxs[v], min_idx)
                    for v2 in vec[1:i-1]
                        forward_idxs[v] = max(
                            forward_idxs[v],
                            forward_idxs[v2] + feat_vals[VtxWidth()][v2],
                        )
                    end
                end
            end
        end
        feat_vals[idx_key] = forward_idxs
        feat_vals[ctr_key] = forward_idxs .+ 0.5 * feat_vals[VtxWidth()]
    end
    feat_vals
end

# Node plotting utilities
_title_string(n::Int) = string(n)
_title_string(n) = ""
_subtitle_string(n) = ""
_node_color(n) = "red"
_node_bg_color(n) = "white"
_text_color(n) = _node_color(n)
_node_shape(n, t = 0.1) = Compose.circle(0.5, 0.5, 0.5 - t / 2)
_title_text_scale(n) = 0.6
_subtitle_text_scale(n) = 0.2
for op in (:_node_shape, :_node_color, :_subtitle_string, :_text_color, :_title_string)
    @eval $op(graph::AbstractGraph, v, args...) = $op(v, args...)
end

function draw_node(
    n;
    t = 0.1,  # line thickness
    title_scale = _title_text_scale(n),
    subtitle_scale = _subtitle_text_scale(n),
    bg_color = _node_bg_color(n),
    text_color = _text_color(n),
    node_color = _node_color(n),
    title_text = _title_string(n),
    subtitle_text = _subtitle_string(n),
)
    title_y = 0.5
    if !isempty(subtitle_text)
        title_y -= subtitle_scale / 2
    end
    subtitle_y = title_y + (title_scale) / 2
    Compose.compose(
        context(),
        (
            context(),
            Compose.text(0.5, title_y, title_text, hcenter, vcenter),
            Compose.fill(text_color),
            fontsize(title_scale * min(w, h)),
        ),
        (
            context(),
            Compose.text(0.5, subtitle_y, subtitle_text, hcenter, vcenter),
            Compose.fill(text_color),
            Compose.fontsize(subtitle_scale * min(w, h)),
        ),
        (
            context(),
            _node_shape(n, t),
            fill(bg_color),
            Compose.stroke(node_color),
            Compose.linewidth(t * w),
        ),
    )
end

draw_node(graph, v; kwargs...) = draw_node(v; kwargs...)

function default_draw_node(G, v; fg_color = "red", bg_color = "white", stroke_width = 0.1)
    draw_node(G, v; t = stroke_width)
end

default_draw_edge(G, v1, v2, pt1, pt2; fg_color = "blue", stroke_width = 0.01) = (
    context(),
    Compose.line([pt1, pt2]),
    Compose.stroke(fg_color),
    Compose.linewidth(stroke_width * w),
)

function draw_ortho_edge(G, v1, v2, pt1, pt2; fg_color = "blue", stroke_width = 0.01)
    ymid = (pt1[2] + pt2[2]) / 2
    (
        context(),
        Compose.line([pt1, (pt1[1], ymid), (pt2[1], ymid), pt2]),
        Compose.stroke(fg_color),
        Compose.linewidth(stroke_width * w),
    )
end

"""
    display_graph(graph;kwargs...)

Plot a graph.
"""
function display_graph(
    graph;
    draw_node_function = (G, v) -> default_draw_node(G, v),
    draw_edge_function = (G, v, v2, pt1, pt2) -> default_draw_edge(G, v, v2, pt1, pt2),
    grow_mode = :from_bottom,  # :from_left, :from_bottom, :from_top,
    align_mode = :leaf_aligned,  # :root_aligned
    node_size = (0.75, 0.75),
    scale = 1.0,
    pad = (0.5, 0.5),
    aspect_stretch = (1.0, 1.0),
    bg_color = "white",
    enforce_visited = false,
)
    feat_vals = get_graph_layout(graph; enforce_visited = enforce_visited)
    # Check alignment mode
    if align_mode == :leaf_aligned
        x = feat_vals[ForwardDepth()]
        y = feat_vals[ForwardCenter()]
    elseif align_mode == :root_aligned
        x = feat_vals[BackwardDepth()]
        x = 1 + maximum(x) .- x
        y = feat_vals[BackwardCenter()]
    elseif align_mode == :split_aligned
        x = feat_vals[ForwardDepth()]
        y = feat_vals[ForwardCenter()]
        # Start with the root node, which is the max x value for forward_depth
        root_node = argmax(x)
        parents = [root_node]
        temp_parents = []
        while !isempty(parents)
            for parent in parents
                for v in inneighbors(graph, parent)
                    push!(temp_parents, v)
                    for vc in outneighbors(graph, v)
                        if vc == parent
                            continue
                        end
                        x[vc] = x[parent]
                    end
                    x[v] = x[parent] - 1
                end
            end
            parents = temp_parents
            temp_parents = []
        end
    else
        throw(ErrorException("Unknown align_mode $align_mode"))
    end
    # Check growth direction
    if grow_mode == :from_left
        # Do nothing
    elseif grow_mode == :from_right
        x = -x
    elseif grow_mode == :from_bottom
        x, y = y, -x
    elseif grow_mode == :from_top
        x, y = y, x
    else
        throw(ErrorException("Unknown grow_mode $grow_mode"))
    end
    # Ensure positive and shift to be on the canvas (how to shift the canvas instead?)
    x *= aspect_stretch[1]
    y *= aspect_stretch[2]
    x = x .- minimum(x) .+ node_size[1] / 2  # .+ pad[1]
    y = y .- minimum(y) .+ node_size[2] / 2  # .+ pad[2]
    context_size = (maximum(x) + node_size[1] / 2, maximum(y) + node_size[2] / 2)
    canvas_size = (context_size[1] .+ 2 * pad[1], context_size[2] .+ 2 * pad[2])
    set_default_graphic_size((scale * canvas_size[1])cm, (scale * canvas_size[2])cm)
    node_context(a, b, s = node_size) = context(
        (a - s[1] / 2),
        (b - s[2] / 2),
        s[1],
        s[2],
        units = UnitBox(0.0, 0.0, 1.0, 1.0),
    )
    edge_context(a, b) = context(a, b, 1.0, 1.0, units = UnitBox(0.0, 0.0, 1.0, 1.0))
    nodes = map(
        v -> (node_context(x[v], y[v]), draw_node_function(graph, v)),
        Graphs.vertices(graph),
    )
    lines = []
    for e in edges(graph)
        dx = x[e.dst] - x[e.src]
        dy = y[e.dst] - y[e.src]
        push!(
            lines,
            (
                edge_context(x[e.src], y[e.src]),
                draw_edge_function(graph, e.src, e.dst, (0.0, 0.0), (dx, dy)),
            ),
        )
    end
    Compose.compose(
        context(units = UnitBox(0.0, 0.0, canvas_size...)),
        (
            context(
                pad[1],
                pad[2],
                context_size...,
                units = UnitBox(0.0, 0.0, context_size...),
            ),
            nodes...,
            lines...,
        ),
        Compose.compose(context(), rectangle(), fill(bg_color)),
    )
end
