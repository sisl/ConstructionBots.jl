export
    global_verbosity,
    set_global_verbosity!

global VERBOSITY = 0

"""
	global_verbosity()

Query the global verbosity setting (VERBOSITY::Int)
"""
global_verbosity() = copy(VERBOSITY)

"""
	set_global_verbosity!(val::Int)

Set the global verbosity to val
"""
set_global_verbosity!(val::Int) = begin global VERBOSITY = val end

function print_styled_header(header,msg...;bold=true,color=136)
	printstyled(header;bold=bold,color=color)
	println(msg...)
end

export @log_info

"""
    @log_info

A helper macro for printing at various verbosity levels.
Usage:
	`@log_info(limit::Int,level::Int,msg...)`
	`@log_info(limit::Int,solver,msg...)`
Args:
* limit::Int - logging threshold
* level::Int - the verbosity level
* msg... - message to print if level > limit
"""
macro log_info(limit,level,msg...)
	filename = split(string(__source__.file),"/")[end]
	ex = :(print_styled_header(log_preamble))
    for x in esc(msg).args[1]
		push!(ex.args,esc(x))
    end
	return quote
		local log_preamble = string("[ logger ](",$filename,"-",$(__source__.line),"): ")
		$(esc(level)) > $(esc(limit)) || global_verbosity() > $(esc(limit)) ?
			$ex : nothing
	end
end