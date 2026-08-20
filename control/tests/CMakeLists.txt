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

add_executable(test_pos_calculator test_pos_calculator.cpp)
target_link_libraries(test_pos_calculator PRIVATE drone_lib)
add_test(NAME pos_calculator COMMAND test_pos_calculator)

add_executable(test_health_monitor test_health_monitor.cpp)
target_link_libraries(test_health_monitor PRIVATE drone_lib)
add_test(NAME health_monitor COMMAND test_health_monitor)

add_executable(test_preflight_state
    test_preflight_state.cpp
    fake_transport.cpp
)
target_link_libraries(test_preflight_state PRIVATE drone_lib Threads::Threads)
target_include_directories(test_preflight_state PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME preflight_state COMMAND test_preflight_state)

add_executable(test_control_authority
    test_control_authority.cpp
    fake_transport.cpp
)
target_link_libraries(test_control_authority PRIVATE drone_lib Threads::Threads)
target_include_directories(test_control_authority PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME control_authority COMMAND test_control_authority)

add_executable(test_target_centered_hold test_target_centered_hold.cpp)
target_include_directories(test_target_centered_hold PRIVATE ${CMAKE_SOURCE_DIR})
add_test(NAME target_centered_hold COMMAND test_target_centered_hold)

add_executable(test_pixel_guidance test_pixel_guidance.cpp)
target_link_libraries(test_pixel_guidance PRIVATE drone_lib)
add_test(NAME pixel_guidance COMMAND test_pixel_guidance)

add_executable(test_diagonal_approach_guidance test_diagonal_approach_guidance.cpp)
target_link_libraries(test_diagonal_approach_guidance PRIVATE drone_lib)
add_test(NAME diagonal_approach_guidance COMMAND test_diagonal_approach_guidance)

add_executable(test_runtime_policy test_runtime_policy.cpp)
target_link_libraries(test_runtime_policy PRIVATE drone_lib Threads::Threads)
target_compile_definitions(test_runtime_policy PRIVATE
    ASTRODRONE_REPO_ROOT="${CMAKE_SOURCE_DIR}/..")
add_test(NAME runtime_policy COMMAND test_runtime_policy)
