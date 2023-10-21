export
    CachedElement,
    get_element,
    is_up_to_date,
    time_stamp,
    set_up_to_date!,
    set_element!,
    set_time_stamp!,
    update_element!

"""
    CachedElement{E}

A mutable container for caching things.
"""
mutable struct CachedElement{E} 
    element::E
    is_up_to_date::Bool
    timestamp::Float64
end
CachedElement(element,flag) = CachedElement(element,flag,time())
CachedElement(element) = CachedElement(element,false)
"""
    get_element(n::CachedElement)

Retrieve the element stored in n.
"""
get_element(n::CachedElement) = n.element
"""
    is_up_to_date(n::CachedElement)

Check if the n is up to date.
"""
is_up_to_date(n::CachedElement) = n.is_up_to_date
"""
    time_stamp(n::CachedElement)

Return the time_stamp at which n was last modified.
"""
time_stamp(n::CachedElement) = n.timestamp
"""
    set_up_to_date!(n::CachedElement,val::Bool=true)

Set the time stamp of n.
"""
function set_up_to_date!(n::CachedElement,val::Bool=true)
    n.is_up_to_date = val
end
"""
    set_element!(n::CachedElement,g)

Set the element stored in n. Does NOT set the `is_up_to_date` flag.
"""
function set_element!(n::CachedElement,g)
    n.element = g
end
"""
    set_time_stamp!(n::CachedElement,t=time())

Set the time stamp of n.
"""
function set_time_stamp!(n::CachedElement,t=time())
    n.timestamp = t
end
"""
    update_element!(n::CachedElement,g)

Set the element of n to g, update the `is_up_to_date` flag and the time stamp.
"""
function update_element!(n::CachedElement,g)
    set_element!(n,g)
    set_up_to_date!(n,true)
    set_time_stamp!(n)
    return g
end

"""
    Base.copy(e::CachedElement)

Shares the e.element, since it doesn't need to be replaced until `set_element!`
is called. Copies `e.is_up_to_date` to preserve the cache state.
"""
Base.copy(e::CachedElement) = CachedElement(e.element,copy(is_up_to_date(e)),time())
Base.convert(::Type{CachedElement{T}},e::CachedElement) where {T} = CachedElement{T}(get_element(e),is_up_to_date(e),time())



const cached_element_accessor_interface = [:is_up_to_date,:time_stamp]
const cached_element_mutator_interface = [:update_element!,:set_time_stamp!,:set_element!,:set_up_to_date!]

export
    Toggle,
    set_toggle_status!,
    get_toggle_status

mutable struct Toggle
    status::Bool
end
function set_toggle_status!(t::Toggle,val=true)
    t.status = val
end
get_toggle_status(t::Toggle) = copy(t.status)

export
    Counter,
    set_counter_status!,
    get_counter_status

mutable struct Counter
    status::Int
end
function set_counter_status!(t::Counter,val::Int)
    t.status = val
end
get_counter_status(t::Counter) = copy(t.status)