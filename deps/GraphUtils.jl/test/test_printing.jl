let
    @test sprint_padded(4;pad=5)                == "    4"
    @test sprint_padded("hi";pad=5)             == "   hi"
    @test sprint_padded("hi";
        pad=5,leftaligned=true)                 == "hi   "
    io = IOBuffer()
    sprint_padded_list(io,[1,10,2];pad=3)
    @test String(take!(io))    == "[  1 10  2]"
    sprint_padded_list(io,[1,10,2];
        pad=3,leftaligned=true) 
    @test String(take!(io))    == "[1  10 2  ]"
    sprint_padded_list_array(io,[[1,2,3],[4,5,6]];
        id_pad=3,pad=3,leftaligned=true)        
    @test String(take!(io))                     ==  """
                                                      1: [1  2  3  ]
                                                      2: [4  5  6  ]
                                                    """
    sprint_indexed_list_array(io,[[1,2,3],[4,5,6]];
        id_pad=4,pad=3,leftaligned=true) 
    @test String(take!(io))                     ==  "   T:  0  1  2   \n   1: [1  2  3  ]\n   2: [4  5  6  ]\n"

end
