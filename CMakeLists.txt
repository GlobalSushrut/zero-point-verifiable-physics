cmake_minimum_required(VERSION 3.14)
project(cpp_physics_engine VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

# Set include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

# Add library source files
file(GLOB LIB_SOURCES "src/*.cpp")
add_library(${PROJECT_NAME} STATIC ${LIB_SOURCES})

# Link with OpenMP
target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)

# Add executable
add_executable(engine_test test/test_engine.cpp)
target_link_libraries(engine_test PRIVATE ${PROJECT_NAME})

# Add test executable
add_executable(performance_test test/performance_test.cpp)
target_link_libraries(performance_test PRIVATE ${PROJECT_NAME})

# Add black hole simulation executable
add_executable(black_hole_sim test/black_hole_simulation.cpp)
target_link_libraries(black_hole_sim PRIVATE ${PROJECT_NAME})

# Include the zero_point subdirectory for military-grade simulation
add_subdirectory(zero_point)

# Enable testing
enable_testing()
add_test(NAME EngineTest COMMAND engine_test)
add_test(NAME PerformanceTest COMMAND performance_test)

# Installation
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
