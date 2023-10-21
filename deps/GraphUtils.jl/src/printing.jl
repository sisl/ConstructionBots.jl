export
   sprint_padded,
   sprint_padded_list,
   sprint_padded_list_array,
   sprint_indexed_list_array


"""
   sprint_padded(i,pad=3,leftaligned=false)

Pads the string representation of i
"""
function sprint_padded(i;
      pad::Int=4,
      leftaligned::Bool=false,
      kwargs...
      )
   padsym = leftaligned ? "-" : ""
   s = "%$(padsym)$(pad).$(pad)s"
   str = "$i"
   eval(quote
       @sprintf($s,$str)
   end)
end
function sprint_padded(io::IO,i;
      pad::Int=4,
      leftaligned::Bool=false,
      kwargs...
      )
   if leftaligned
      print(io,i)
   end
   for t in 1:(pad - length(string(i)))
      print(io," ")
   end
   if !leftaligned
      print(io,i)
   end
   # padsym = leftaligned ? "-" : ""
   # s = "%$(padsym)$(pad).$(pad)s"
   # str = "$i"
   # eval(quote
   #     @sprintf($s,$str)
   # end)
end

"""
   sprint_padded_list(vec,pad=3,leftaligned=false)

Returns a string as in:
   ```
   julia> sprint_padded_list([1,2,3],3)
   "[  1  2  3]"
   ```
"""
function sprint_padded_list(io::IO,vec;
      lchar::String="[",
      rchar::String="]",
      kwargs...
      )
   string(lchar,map(x->sprint_padded(x;kwargs...),vec)...,rchar)
   print(io,lchar)
   for x in vec
      # print(io,sprint_padded(x;kwargs...))
      sprint_padded(io,x;kwargs...)
   end 
   print(io,rchar)
end

function sprint_padded_list_array(io::IO,vecs;id_pad::Int=4,kwargs...)
   for (i,vec) in enumerate(vecs)
      print(io,sprint_padded(i,pad=id_pad),": ")
      sprint_padded_list(io,vec;kwargs...)
      print(io,"\n")
   end
   # string([string(
   #       sprint_padded(i;pad=id_pad),
   #       ": ",
   #       sprint_padded_list(vec;kwargs...),
   #       "\n"
   #    ) for (i,vec) in enumerate(vecs)]...)
end

function sprint_indexed_list_array(vecs;kwargs...)
   buffer = IOBuffer()
   sprint_indexed_list_array(buffer,vecs;kwargs...)
   String(take!(buffer))
end
function sprint_indexed_list_array(io::IO,vecs;idx_str::String="T",id_pad::Int=4,
      kwargs...)
   print(io,sprint_padded(idx_str;pad=id_pad),": ",)
   sprint_padded_list(io,0:maximum(map(length,vecs))-1;lchar=" ",rchar=" ",kwargs...)
   print(io,"\n")
   sprint_padded_list_array(io,vecs;kwargs...)
   # idx_string = string(
   #       sprint_padded(idx_str;pad=id_pad),
   #       ": ",
   #       sprint_padded_list(0:maximum(map(length,vecs))-1;
   #          lchar=" ",rchar=" ",kwargs...),
   #       "\n"
   #    )
   # return string(idx_string,sprint_padded_list_array(vecs;kwargs...))
end

function TOML.print(io,dict::Dict{Symbol,A}) where {A}
    TOML.print(io,Dict(string(k)=>v for (k,v) in dict))
end