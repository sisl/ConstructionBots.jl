# Test CachedElement
let
    a = CachedElement(1)
    @test get_element(a) == 1
    set_up_to_date!(a,false)
    @test !is_up_to_date(a)
    update_element!(a,2)
    @test is_up_to_date(a)
    @test get_element(a) == 2
    b = CachedElement(1)
    update_element!(b,4)
    @test time_stamp(b) > time_stamp(a)
    update_element!(a,2)
    @test time_stamp(b) < time_stamp(a)
end