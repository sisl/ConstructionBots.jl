# Tools for dealing with nested dictionaries
"""
    nested_default_dict_get(dict,k,keylist...;default=nothing)

Query into an arbitrarily deep nested dict until finding a non-dict value or 
returning `default` when the current key does not point to a dict.
Example
```julia
dict = Dict()
nested_default_dict_set!(dict,1,2,"val")
nested_default_dict_set!(dict,1,2,3,"other val")
nested_default_dict_get(dict,1,2) == "val"
nested_default_dict_get(dict,1,2,3) == "val"
```
"""
function nested_default_dict_get(dict,k,keylist...;default=nothing,_default_key=:Default)
    if haskey(dict,k)
        val = dict[k]
        if isa(val,Dict)
            return nested_default_dict_get(val,keylist...;default=default,_default_key=_default_key)
        else
            # return value early
            return val
        end
    end
    if !(default === nothing)
        return default
    else
        return get(dict,_default_key,default)
    end
    return get(dict,_default_key,default)
end
function nested_default_dict_get(dict,k;default=nothing,_default_key=:Default)
    if haskey(dict,k)
        val = dict[k]
        if isa(val,Dict)
            if haskey(val,_default_key)
                return val[_default_key]
            end
        else
            return val
        end
    end
    if !(default === nothing)
        return default
    else
        return get(dict,_default_key,default)
    end
    return get(dict,_default_key,default)
end
"""
    nested_default_dict_set!(dict,keylist...,[val])

Sets the value of a nested dictionary, indexed by a list of keys, to `val`. If 
the full `keylist` does not yet exist, create the intermediate dicts along the 
way.
Example:
```julia
dict = Dict()
nested_default_dict_set!(dict,1,2,3,"my val")
```
"""
function nested_default_dict_set!(dict,k,k2,args...;_default_key=:Default)
    subdict = get!(dict,k,Dict())
    if !isa(subdict,Dict)
        subdict = Dict{Any,Any}(_default_key=>subdict)
        dict[k] = subdict
    end
    nested_default_dict_set!(subdict,k2,args...;_default_key=_default_key)
end
function nested_default_dict_set!(dict,k,val;_default_key=:Default)
    if haskey(dict,k)
        old_val = dict[k]
        if isa(old_val,Dict)
            old_val[_default_key] = val
            return val
        end
    end
    dict[k] = val
end
