cmake_minimum_required(VERSION 3.16)

project(sour_angel_pico_control LANGUAGES CXX)

add_library(sour_angel_pico_control
    src/control_state_estimator.cpp
    src/lateral_controller.cpp
    src/longitudinal_controller.cpp
    src/setpoint_safety.cpp
)

target_include_directories(sour_angel_pico_control
    PUBLIC include
)

target_compile_features(sour_angel_pico_control PUBLIC cxx_std_17)

option(SOUR_ANGEL_ENABLE_PICO_SDK
    "Build the RP2040 multicore command mailbox (requires pico_sync)"
    OFF
)

if(SOUR_ANGEL_ENABLE_PICO_SDK)
    if(NOT TARGET pico_sync)
        message(FATAL_ERROR
            "SOUR_ANGEL_ENABLE_PICO_SDK requires the Pico SDK to be initialized first")
    endif()

    target_sources(sour_angel_pico_control PRIVATE
        src/communication_service.cpp
        src/setpoint_mailbox.cpp
        src/telemetry_mailbox.cpp
    )
    target_link_libraries(sour_angel_pico_control PUBLIC pico_sync)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(sour_angel_pico_control PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wsign-conversion
    )
endif()
