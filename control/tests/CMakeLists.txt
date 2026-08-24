add_executable(test_target_json test_target_json.cpp)
target_link_libraries(test_target_json PRIVATE target_json)
add_test(NAME target_json COMMAND test_target_json)

add_executable(test_target_link
    test_target_link.cpp
)
target_link_libraries(test_target_link PRIVATE target_distance_support Threads::Threads)
add_test(NAME target_link COMMAND test_target_link)

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

add_executable(test_autopilot_adapter
    test_autopilot_adapter.cpp
    fake_transport.cpp
)
target_link_libraries(test_autopilot_adapter PRIVATE drone_lib Threads::Threads)
target_include_directories(test_autopilot_adapter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME autopilot_adapter COMMAND test_autopilot_adapter)

add_executable(test_health_check
    test_health_check.cpp
    fake_transport.cpp
)
target_link_libraries(test_health_check PRIVATE drone_lib Threads::Threads)
target_include_directories(test_health_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME health_check COMMAND test_health_check)

add_executable(test_mission_setup
    test_mission_setup.cpp
    fake_transport.cpp
)
target_link_libraries(test_mission_setup PRIVATE drone_lib Threads::Threads)
target_include_directories(test_mission_setup PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME mission_setup COMMAND test_mission_setup)

add_executable(test_pos_calculator test_pos_calculator.cpp)
target_link_libraries(test_pos_calculator PRIVATE drone_lib)
add_test(NAME pos_calculator COMMAND test_pos_calculator)

add_executable(test_flight_phase
    test_flight_phase.cpp
    fake_transport.cpp
)
target_link_libraries(test_flight_phase PRIVATE drone_lib Threads::Threads)
target_include_directories(test_flight_phase PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME flight_phase COMMAND test_flight_phase)

add_executable(test_safety_monitor
    test_safety_monitor.cpp
    fake_transport.cpp
)
target_link_libraries(test_safety_monitor PRIVATE drone_lib Threads::Threads)
target_include_directories(test_safety_monitor PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME safety_monitor COMMAND test_safety_monitor)

add_executable(test_centered_vertical_descent_algorithm
    test_centered_vertical_descent_algorithm.cpp)
target_link_libraries(test_centered_vertical_descent_algorithm PRIVATE drone_lib)
add_test(NAME centered_vertical_descent_algorithm
    COMMAND test_centered_vertical_descent_algorithm)

add_executable(test_diagonal_approach_algorithm
    test_diagonal_approach_algorithm.cpp)
target_link_libraries(test_diagonal_approach_algorithm PRIVATE drone_lib)
add_test(NAME diagonal_approach_algorithm COMMAND test_diagonal_approach_algorithm)

add_executable(test_runtime_policy test_runtime_policy.cpp)
target_link_libraries(test_runtime_policy PRIVATE drone_lib Threads::Threads)
target_compile_definitions(test_runtime_policy PRIVATE
    ASTRODRONE_REPO_ROOT="${CMAKE_SOURCE_DIR}/..")
add_test(NAME runtime_policy COMMAND test_runtime_policy)

add_executable(test_flight_mission_app_composition
    test_flight_mission_app_composition.cpp
    ${CMAKE_SOURCE_DIR}/app/flight_mission_app.cpp
)
target_link_libraries(test_flight_mission_app_composition PRIVATE drone_lib Threads::Threads)
add_test(NAME flight_mission_app_composition COMMAND test_flight_mission_app_composition)

add_executable(test_preflight_readiness test_preflight_readiness.cpp)
target_link_libraries(test_preflight_readiness PRIVATE drone_lib Threads::Threads)
add_test(NAME preflight_readiness COMMAND test_preflight_readiness)
