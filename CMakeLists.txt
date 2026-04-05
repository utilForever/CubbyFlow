# CMake version
cmake_minimum_required(VERSION 3.31.6 FATAL_ERROR)
cmake_policy(SET CMP0063 NEW) # visibility
if (POLICY CMP0146)
    cmake_policy(SET CMP0146 OLD) # Keep legacy FindCUDA module behavior on newer CMake.
endif()
if (POLICY CMP0147)
    cmake_policy(SET CMP0147 OLD) # Required for legacy FindCUDA with Visual Studio generators.
endif()
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

# Include cmake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Builds/CMake")

# Declare project
project(CubbyFlow)

# Useful paths
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Libraries)

# Set output directories
set(DEFAULT_CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

# Includes
include_directories(Includes)
include_directories(Libraries)
include_directories(Libraries/pybind11/include)
include_directories(Libraries/Clara/include)
include_directories(Libraries/tinyobj)
include_directories(Libraries/doctest/doctest)

# Set enable output of compile commands during generation
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build type - Release by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
elseif (CMAKE_BUILD_TYPE MATCHES "Debug")
    add_definitions(-DDEBUG)
endif()
message(STATUS "CMake build type: ${CMAKE_BUILD_TYPE}")

# CUDA
option(USE_CUDA "Use CUDA features" ON)
set(CUDA_CRT_LINKAGE "dynamic" CACHE STRING 
                     "CUDA targets' CRT options. For 'static', use /MT or /MTd. For 'dynamic', use /MD or /MDd")
if (USE_CUDA)
    set(CUDA_LINK_LIBRARIES_KEYWORD PUBLIC)
    set(CUDA_SEPARABLE_COMPILATION OFF)
    set(CUDA_STANDARD 17)
    find_package(CUDA)
    if (CUDA_FOUND) # Modify project options for CUDA build
        enable_language(CUDA)
        if (EXISTS "${CUDA_TOOLKIT_ROOT_DIR}/include/cccl")
            include_directories("${CUDA_TOOLKIT_ROOT_DIR}/include/cccl")
        endif()
        cuda_select_nvcc_arch_flags(CUDA_NVCC_FLAGS "7.5" "Turing")
        # We will use CUDA_TARGET_OPTIONS instead of CUDA_NVCC_FLAGS to be explicit.
        # start with architecture options
        list(APPEND CUDA_TARGET_OPTIONS
            "--std=c++17" "-Wno-deprecated-gpu-targets")
        if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:preprocessor")
            list(APPEND CUDA_TARGET_OPTIONS "-Xcompiler" "/FS") # prevent race on PDB
            list(APPEND CUDA_TARGET_OPTIONS "-Xcompiler" "/Zc:preprocessor") # required by CUDA 13 + VS 2026
            if (CUDA_CRT_LINKAGE STREQUAL "dynamic")
                if (CMAKE_BUILD_TYPE MATCHES Debug)
                    list(APPEND CUDA_TARGET_OPTIONS "-Xcompiler" "/MDd" "--debug" "-O0")
                else()
                    list(APPEND CUDA_TARGET_OPTIONS "-Xcompiler" "/MD" "-O2")
                endif()
            elseif (CUDA_CRT_LINKAGE STREQUAL "static")
                if(CMAKE_BUILD_TYPE MATCHES Debug)
                    list(APPEND CUDA_TARGET_OPTIONS "-Xcompiler" "/MTd" "--debug" "-O0")
                else()
                    list(APPEND CUDA_TARGET_OPTIONS "-Xcompiler" "/MT" "-O2")
                endif()
            endif()
            list(APPEND DEFAULT_COMPILE_OPTIONS
                /wd4515 # -> disable warning: [Thrust issue] 'detail': namespace uses itself
                /wd4996 # -> disable warning: [Thrust issue] 'strerror': This function or variable may be unsafe.
                /wd4800 # -> disable warning: [Thrust issue] forcing value to bool 'true' or 'false' (performance warning)
            )
        endif()
        add_compile_definitions(CUBBYFLOW_USE_CUDA)
        message(STATUS "Using CUDA: ${CUDA_VERSION}")
        message(STATUS "  CRT: ${CUDA_CRT_LINKAGE}")
        message(STATUS "  options: ${CUDA_TARGET_OPTIONS}")
        message(STATUS "  nvcc flags: ${CUDA_NVCC_FLAGS}")
        # note: This is for CI build. It fails to find this library so we are mentioning it here
        if(EXISTS ${CUDA_cudadevrt_LIBRARY})
            message(STATUS "  cudadevrt: ${CUDA_cudadevrt_LIBRARY}")
            get_filename_component(CUDA_LIB_DIR ${CUDA_cudadevrt_LIBRARY} DIRECTORY)
            link_directories(${CUDA_LIB_DIR})
            message(STATUS "  lib: ${CUDA_LIB_DIR}")
        endif()
    else()
        set(USE_CUDA OFF)
    endif()
endif()

# Find TBB
include(Builds/CMake/FindTBB.cmake)

# Tasking system options
include(Builds/CMake/TaskingSystemOptions.cmake)

# Compile options
include(Builds/CMake/CompileOptions.cmake)
if (USE_CUDA AND CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    list(APPEND DEFAULT_COMPILE_OPTIONS /wd4505)
endif()

# Code coverage - Debug only
# NOTE: Code coverage results with an optimized (non-Debug) build may be misleading
option(BUILD_COVERAGE "Build code coverage" OFF)
if (CMAKE_BUILD_TYPE MATCHES Debug AND CMAKE_COMPILER_IS_GNUCXX AND BUILD_COVERAGE)
    include(CodeCoverage)
    setup_target_for_coverage(${PROJECT_NAME}_coverage UnitTests coverage)
endif()

# Some project don't build - SonarCloud only
option(BUILD_SONARCLOUD "Build for SonarCloud" OFF)

# Overrides
set(CMAKE_MACOSX_RPATH ON)

# Set resources
get_filename_component(RESOURCES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Resources ABSOLUTE)
set(RESOURCES_OBJS
        ${RESOURCES_DIR}/bunny.obj
        ${RESOURCES_DIR}/cube.obj
        ${RESOURCES_DIR}/dragon.obj)

# Project modules
set(BUILD_GTEST ON CACHE BOOL "" FORCE)
set(BUILD_GMOCK ON CACHE BOOL "" FORCE)
find_package(GTest CONFIG)
if(FALSE) # GTest_FOUND
    # link_libraries(GTest::gmock GTest::gtest)
else()
    set(gtest_force_shared_crt ON CACHE BOOL "")
    set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
    add_subdirectory(Libraries/googletest EXCLUDE_FROM_ALL)
    add_subdirectory(Libraries/googlebenchmark EXCLUDE_FROM_ALL)
endif()
add_subdirectory(Libraries/cnpy)
add_subdirectory(Libraries/pystring)
add_subdirectory(Sources/Core)

option(BUILD_TESTS "Build the CubbyFlow test programs" ON)
if (BUILD_TESTS)
    add_subdirectory(Tests/ManualTests)
    add_subdirectory(Tests/UnitTests)
    if (USE_CUDA)
        add_subdirectory(Tests/CUDATests)
    endif()
    if (NOT BUILD_SONARCLOUD)
        add_subdirectory(Tests/MemPerfTests)
        add_subdirectory(Tests/TimePerfTests)
    endif()
endif()

option(BUILD_EXAMPLES "Build the CubbyFlow example programs" ON)
if (BUILD_EXAMPLES)
    if (USE_CUDA)
        add_subdirectory(Examples/CUDASPHSim)
    endif()
    if (NOT BUILD_SONARCLOUD)
        add_subdirectory(Examples/HelloFluidSim)
        add_subdirectory(Examples/HybridLiquidSim)
        add_subdirectory(Examples/LevelSetLiquidSim)
        add_subdirectory(Examples/Obj2Sdf)
        add_subdirectory(Examples/Particles2Obj)
        add_subdirectory(Examples/Particles2Xml)
        add_subdirectory(Examples/SmokeSim)
        add_subdirectory(Examples/SPHSim)
    endif()
endif()

if (NOT USE_CUDA AND NOT BUILD_SONARCLOUD)
    add_subdirectory(Libraries/pybind11)
    if (BUILD_FROM_PIP)
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${DEFAULT_CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    endif()
    add_subdirectory(Sources/API/Python)
endif()
