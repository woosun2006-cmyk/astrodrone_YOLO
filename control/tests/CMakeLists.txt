add_executable(test_target_json test_target_json.cpp)
target_link_libraries(test_target_json PRIVATE target_json)
add_test(NAME target_json COMMAND test_target_json)

add_executable(test_mav_connection
    test_mav_connection.cpp
    fake_transport.cpp
)
target_link_libraries(test_mav_connection PRIVATE drone_lib Threads::Threads)
target_include_directories(test_mav_connection PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME mav_connection COMMAND test_mav_connection)

add_executable(test_command_characterization
    test_command_characterization.cpp
    fake_transport.cpp
)
target_link_libraries(test_command_characterization PRIVATE drone_lib Threads::Threads)
target_include_directories(test_command_characterization PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME command_characterization COMMAND test_command_characterization)
