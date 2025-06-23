cmake_minimum_required(VERSION 3.10)
project(physics_engine_sdk VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Enable better debug info
set(CMAKE_VERBOSE_MAKEFILE ON)

# Add more warnings and debugging info
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g")

# Enable ARM NEON SIMD optimizations when available
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    message(STATUS "ARM processor detected, enabling NEON optimizations")
    add_compile_options(-mfpu=neon)
endif()

# Set include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Create the SDK library with minimal source file
add_library(physics_sdk STATIC src/sdk_core.cpp)

# Add threading support
find_package(Threads REQUIRED)
target_link_libraries(physics_sdk Threads::Threads)

# Now that the library compiles correctly, let's build the demo
add_executable(vr_physics_demo examples/vr_physics_demo.cpp)
target_link_libraries(vr_physics_demo physics_sdk)

message(STATUS "Physics Engine SDK configured successfully")
