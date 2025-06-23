cmake_minimum_required(VERSION 3.13)
project(ZeroPointHumanoid VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default optimizations for all platforms
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Wall -Wextra -pedantic")

# Check if we're on ARM architecture for Raspberry Pi 5 optimizations
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mfpu=neon-fp-armv8" HAS_ARM_NEON)
if(HAS_ARM_NEON)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon-fp-armv8 -march=armv8-a+crypto")
    message(STATUS "ARM NEON optimizations enabled for Raspberry Pi 5")
endif()

# Check CPU architecture
include(CheckCXXSourceRuns)
check_cxx_source_runs("
#include <arm_neon.h>
int main() {
    float32x4_t test = vdupq_n_f32(1.0f);
    return 0;
}
" HAVE_NEON)

if(HAVE_NEON)
    add_definitions(-DUSE_NEON)
    message(STATUS "ARM NEON optimizations enabled")
else()
    message(STATUS "ARM NEON optimizations disabled - incompatible CPU")
endif()

# Find required packages
find_package(PkgConfig REQUIRED)

# Make OpenGL optional for military demo (headless mode)
find_package(OpenGL)
if(OPENGL_FOUND)
    pkg_check_modules(GLM glm)
    pkg_check_modules(GLFW glfw3)
    pkg_check_modules(GLEW glew)
    pkg_check_modules(SPIRV TOOLS spirv-tools)
    set(ENABLE_VISUALIZATION TRUE)
    add_definitions(-DENABLE_VISUALIZATION)
else()
    message(STATUS "OpenGL not found - building in headless mode")
    set(ENABLE_VISUALIZATION FALSE)
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
)

# Add visualization includes if enabled
if(ENABLE_VISUALIZATION)
    include_directories(
        ${GLM_INCLUDE_DIRS}
        ${GLFW_INCLUDE_DIRS}
        ${GLEW_INCLUDE_DIRS}
    )
endif()

# Define source files by component
set(CORE_SOURCES
    core/intent_heap.cpp
    core/binary_space_tree.cpp 
    core/zk_corom_light.cpp
    core/physics_solver.cpp
    core/formal_verify.cpp
    core/numerical_stability.cpp
    core/integrity_check.cpp
    core/rt_scheduler.cpp
    core/physics_solver_military.cpp
)

set(INTERFACE_SOURCES
    interface/camera_input.cpp
    interface/mic_input.cpp
    interface/proprio_input.cpp
    interface/neural_mesh.cpp
)

set(RENDER_SOURCES
    render/pipeline.cpp
    render/intent_shader_linker.cpp
    render/fallback_compositor.cpp
)

set(AGENT_SOURCES
    agent/reflex_handler.cpp
    agent/perception_loop.cpp
    agent/energy_regulator.cpp
)

# Copy shader files to build directory
file(GLOB SHADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/shaders/*)
file(COPY ${SHADER_FILES} DESTINATION ${CMAKE_BINARY_DIR}/shaders)

# Create main executable only if visualization is available
if(ENABLE_VISUALIZATION)
    # Only build if main.cpp actually exists
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
        add_executable(zero_point
            main.cpp
            ${CORE_SOURCES}
            ${INTERFACE_SOURCES}
            ${RENDER_SOURCES}
            ${AGENT_SOURCES}
        )

        # Link libraries
        target_link_libraries(zero_point
            ${OPENGL_LIBRARIES}
            ${GLFW_LIBRARIES}
            ${GLEW_LIBRARIES}
            pthread
            m
        )

        # Installation
        install(TARGETS zero_point DESTINATION bin)
        install(DIRECTORY ${CMAKE_BINARY_DIR}/shaders DESTINATION share/zero_point)
    else()
        message(STATUS "main.cpp not found - skipping zero_point executable")
    endif()
else()
    message(STATUS "Visualization disabled - skipping zero_point executable")
endif()

# Testing
enable_testing()
# No tests subdirectory in zero_point

# Military-grade simulation demo
add_executable(military_demo
    ../test/military_demo/military_grade_simulation.cpp
    ${CORE_SOURCES}
)

# Link with appropriate libraries based on mode
if(ENABLE_VISUALIZATION)
    target_link_libraries(military_demo
        ${OPENGL_LIBRARIES}
        ${GLFW_LIBRARIES}
        ${GLEW_LIBRARIES}
        pthread
        m
    )
else()
    target_link_libraries(military_demo
        pthread
        m
    )
endif()

install(TARGETS military_demo DESTINATION bin)

# Print configuration summary
message(STATUS "ZeroPoint Humanoid Simulation System configuration:")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  OpenGL Libraries: ${OPENGL_LIBRARIES}")
