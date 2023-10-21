let
    basepath = joinpath(pwd(),"foo")
    subdirs = ["bar","baz"]
    files = ["hello.a","goodbye.a","hello.b","goodbye.b"]
    for d in subdirs
        for f in files
            mkpath(joinpath(basepath,d,f))
        end
    end
    for ext in [".a",".b"]
        for kw in ["hello","goodbye"]
            @test all(p->findfirst(ext,p) != nothing, get_files_matching(basepath,ext))
            @test all(p->findfirst(kw,p) != nothing, get_files_matching(basepath,ext,[kw]))
        end
    end
    rm(basepath;recursive=true)
end
let
    logfile = "tmp.log"
    errfile = "tmp.err"
    logmsg = "hello world"
    errmsg = "error world"
    redirect_to_files(logfile,errfile) do
        println(logmsg)
        println(stderr,errmsg)
    end
    @test readline(logfile) == logmsg
    @test readline(errfile) == errmsg
    rm(logfile)
    rm(errfile)
end
