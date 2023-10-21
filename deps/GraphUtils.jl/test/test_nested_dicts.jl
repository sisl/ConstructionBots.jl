let
    dict = Dict()
    GraphUtils.nested_default_dict_set!(dict,1,"my val")
    @test GraphUtils.nested_default_dict_get(dict,1) == "my val"
    GraphUtils.nested_default_dict_set!(dict,1,2,"other val")
    @test GraphUtils.nested_default_dict_get(dict,1,2) == "other val"
    # Setting a value deeper in tree should not affect previous leaf
    @test GraphUtils.nested_default_dict_get(dict,1) == "my val"
    # if partial path exists, should return the leaf of that path
    @test GraphUtils.nested_default_dict_get(dict,1,2,3) == "other val"
    # if no partial path exists, should return the default kw value
    @test GraphUtils.nested_default_dict_get(dict,2;default="default val") == "default val"
    @test GraphUtils.nested_default_dict_get(dict,2,3,4;default="default val") == "default val"
    # change an existing value
    GraphUtils.nested_default_dict_set!(dict,1,2,"yet another val")
    @test GraphUtils.nested_default_dict_get(dict,1,2) == "yet another val"
    # should return default kw from a shallow point with no value
    GraphUtils.nested_default_dict_set!(dict,3,4,5,"fifth val")
    @test GraphUtils.nested_default_dict_get(dict,3;default="no val") == "no val"

end