cmake_minimum_required(VERSION 3.10)
project(PhysicsEngine VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add executables
add_executable(GameEngine src/main.cpp)
add_executable(Blocks src/main_blocks.cpp)
add_executable(Parachutes src/main_parachutes.cpp)
add_executable(Tests tests/test_math_physics.cpp)
add_executable(Benchmark bench/benchmark.cpp)
add_executable(ControlDemo src/main_control_demo.cpp)

# The renderer's row-sharing loop uses std::thread.
find_package(Threads REQUIRED)
target_link_libraries(GameEngine PRIVATE Threads::Threads)
target_link_libraries(Blocks PRIVATE Threads::Threads)
target_link_libraries(Parachutes PRIVATE Threads::Threads)
target_link_libraries(Tests PRIVATE Threads::Threads)
target_link_libraries(Benchmark PRIVATE Threads::Threads)
target_link_libraries(ControlDemo PRIVATE Threads::Threads)

# SDL2 linking removed for now to fix build errors; uncomment when UI is enabled
# find_package(SDL2 REQUIRED)
# include_directories(${SDL2_INCLUDE_DIRS})
# target_link_libraries(GameEngine ${SDL2_LIBRARIES})
# target_link_libraries(Blocks ${SDL2_LIBRARIES})
# target_link_libraries(Parachutes ${SDL2_LIBRARIES})
# target_compile_definitions(GameEngine PRIVATE SDL_MAIN_HANDLED)
# target_compile_definitions(Blocks PRIVATE SDL_MAIN_HANDLED)
# target_compile_definitions(Parachutes PRIVATE SDL_MAIN_HANDLED)

# Compiler flags for optimization
if(MSVC)
    target_compile_options(GameEngine PRIVATE /W4 /O2)
    target_compile_options(Blocks PRIVATE /W4 /O2)
    target_compile_options(Parachutes PRIVATE /W4 /O2)
    target_compile_options(Tests PRIVATE /W4 /O2)
    target_compile_options(Benchmark PRIVATE /W4 /O2)
    target_compile_options(ControlDemo PRIVATE /W4 /O2)
else()
    target_compile_options(GameEngine PRIVATE -Wall -Wextra -O2)
    target_compile_options(Blocks PRIVATE -Wall -Wextra -O2)
    target_compile_options(Parachutes PRIVATE -Wall -Wextra -O2)
    target_compile_options(Tests PRIVATE -Wall -Wextra -O2)
    target_compile_options(Benchmark PRIVATE -Wall -Wextra -O2)
    target_compile_options(ControlDemo PRIVATE -Wall -Wextra -O2)

    # BVH::intersect4's packet box test (src/math/AABB.h) is written against
    # AVX2 intrinsics behind #if defined(__AVX2__), with a plain scalar
    # fallback when that macro isn't defined -- so passing -mavx2 is what
    # turns the fallback on, not a requirement to build at all. Checked
    # rather than assumed for the same reason the CUDA block below checks:
    # a build on a machine without AVX2 must still succeed, just without the
    # vectorized path.
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
    if(COMPILER_SUPPORTS_AVX2)
        target_compile_options(GameEngine PRIVATE -mavx2)
        target_compile_options(Blocks PRIVATE -mavx2)
        target_compile_options(Parachutes PRIVATE -mavx2)
        target_compile_options(Tests PRIVATE -mavx2)
        target_compile_options(Benchmark PRIVATE -mavx2)
        target_compile_options(ControlDemo PRIVATE -mavx2)
    else()
        message(STATUS "Compiler doesn't support -mavx2 -- BVH::intersect4 falls back to its scalar box test")
    endif()
endif()

# Optional GPU ray-dispatch port (src/gpu/raytrace_gpu.cu). Only built when a
# CUDA compiler is actually found, so every CPU target above still builds on
# a machine without the CUDA toolkit -- which is every machine this repo has
# been built on so far. See README.md's GPU section: this has been written
# but not compiled or run here.
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    add_executable(GpuRayTrace src/gpu/raytrace_gpu.cu)
    set_target_properties(GpuRayTrace PROPERTIES CUDA_STANDARD 17)
    target_link_libraries(GpuRayTrace PRIVATE Threads::Threads)
else()
    message(STATUS "No CUDA compiler found -- skipping GpuRayTrace target (src/gpu/raytrace_gpu.cu)")
endif()
