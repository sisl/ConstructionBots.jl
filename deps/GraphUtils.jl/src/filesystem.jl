export
    matches_keywords,
    get_files_matching

"""
    matches_keywords(p,keywords)

Check if any of the keywords occurs in p.
"""
function matches_keywords(p,keywords)
    if isempty(keywords)
        return true
    end
    for k in keywords
        if findfirst(k,p) != nothing
            return true
        end
    end
    return false
end

"""
    get_files_matching(base_path,ext,keywords=[])

Get all files in a directory that match the extension `ext`, and (optionally)
which contain any of `keywords`
"""
function get_files_matching(base_path,ext,keywords=[])
    paths = String[]
    for (root,dirs,files) in walkdir(base_path)
        for f in files
            if matches_keywords(f,keywords) && splitext(f)[end] == ext
                push!(paths,joinpath(root,f))
            end
        end
    end
    return paths
end


export redirect_to_files

"""
	redirect_to_files(dofunc, outfile, errfile)

redirects output of `stdout` and `stderr` to `outfile` and `errfile`,
respectively.
Usage:
```julia
redirect_to_files(prefix * ".log", prefix * ".err") do
    compute(...)
end
```
"""
function redirect_to_files(dofunc, outfile, errfile)
    open(outfile, "w") do out
        open(errfile, "w") do err
            redirect_stdout(out) do
                redirect_stderr(err) do
                    dofunc()
                end
            end
        end
    end
end
