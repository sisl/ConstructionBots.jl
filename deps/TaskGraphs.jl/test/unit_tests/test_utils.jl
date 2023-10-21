let
    ID_LIST = [ActionID(),ObjectID(),BotID{DeliveryBot}(),BotID{CleanUpBot}()]
    for id in ID_LIST
        @test read_abstract_id(TOML.parse(id)) == id
    end
end
