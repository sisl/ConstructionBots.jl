let
    reset_action_id_counter!()
    reset_operation_id_counter!()
    reset_all_id_counters!()

    @test get_unique_id(ActionID) == ActionID(1)
    @test get_unique_id(ObjectID) == ObjectID(1)
    @test get_unique_id(OperationID) == OperationID(1)

    @test get_unique_operation_id() == OperationID(2)
    @test get_unique_action_id() == ActionID(2)

    reset_all_id_counters!()
    @test get_unique_id(ActionID) == ActionID(1)
    @test get_unique_id(ObjectID) == ObjectID(1)
    @test get_unique_id(OperationID) == OperationID(1)

    @test !valid_id(get_unique_invalid_id(ActionID))

    reset_all_id_counters!()
    reset_all_invalid_id_counters!()

end
let
    ObjectID()
    RobotID()
    LocationID()
    ActionID()
    OperationID()

    get_id(ObjectID())

    @test ObjectID(1) < ObjectID(2)
    @test ObjectID(2) > ObjectID(1)
    @test ObjectID(1) + 1 == ObjectID(2)
    @test ObjectID(1) == ObjectID(2) - 1
end
