ament_add_gtest(test_ego_entity_simulation test_ego_entity_simulation.cpp)
target_link_libraries(test_ego_entity_simulation simple_sensor_simulator_component ${traffic_simulator_libraries})
