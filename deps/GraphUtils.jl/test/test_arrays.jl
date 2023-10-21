let
    arr = ones(2,2)
    pad_matrix(arr,(2,2),0)
end
let
    a = [1.0,2,3]
    b = [-4.0,5,6]
    @test array_isapprox(cross(a,b),cross_product_operator(a)*b)
end
let
    n = 4
    for i in 1:n
        v = one_hot(n,i)
        @test v[i] == 1
        v = one_hot(i,n)
        @test v[i] == 1
        for j in 1:n
            if i != j
                @test v[j] == 0
            end
        end
    end
    @test_throws AssertionError one_hot(-1,3)
    @test_throws AssertionError one_hot(1,-3)
    @test_throws AssertionError one_hot(-1,-3)
end
let
    vec = [1,2,3,4]
    for i in 1:4
        @test wrap_idx(length(vec),i) == i
        @test wrap_get(vec,i)[1] == i
    end
    @test wrap_idx(length(vec),5) == 1
    @test wrap_get(vec,5)[1] == 1
    @test wrap_idx(length(vec),9) == 1
    @test wrap_idx(length(vec),0) == 4
    @test wrap_idx(length(vec),-4) == 4
    @test wrap_idx(length(vec),-5) == 3
end
let
    a = [
        1 2;
        3 4
        ]
    @test wrap_get(a,[1,1]) == 1
    @test wrap_get(a,[3,3]) == 1
    @test wrap_get(a,[4,4]) == 4
    @test wrap_get(a,[0,-1]) == 3
end
