# test abstract trees
let
    ID_TYPE = VtxID
    EL_TYPE = Int
    a = GraphUtils.TreeNode{EL_TYPE,ID_TYPE}(1)
    b = GraphUtils.TreeNode{EL_TYPE,ID_TYPE}(2)
    c = GraphUtils.TreeNode{EL_TYPE,ID_TYPE}(3)
    GraphUtils._id_type(a)
    set_parent!(b,a)
    @test get_parent(a) === a
    @test get_parent(b) === a
    @test haskey(get_children(a),node_id(b))
    @test validate_tree(a)
    @test validate_tree(b)
    # Check cached behavior
    update_element!(a,2)
    @test GraphUtils.cached_node_up_to_date(a)
    @test !GraphUtils.cached_node_up_to_date(b)
    update_element!(b,2)
    @test GraphUtils.cached_node_up_to_date(b)
    @test GraphUtils.get_cached_value!(b) == 2
    # make tree cyclic and check for error in validation 
    set_parent!(c,b)
    set_parent!(a,c)
    @test_throws ErrorException validate_tree(b)
    # check that parent-child edge is removed when a new parent is set.
    d = GraphUtils.TreeNode{EL_TYPE,ID_TYPE}(4)
    set_parent!(d,a)
    @test haskey(get_children(a),node_id(d))
    set_parent!(d,b)
    @test haskey(get_children(b),node_id(d))
    @test !haskey(get_children(a),node_id(d))
end
# test cached trees
let

end