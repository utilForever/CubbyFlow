# Target name
set(target CUDASPHSim)

# Includes
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Sources
if (USE_CUDA OR USE_HIP)
    file(GLOB_RECURSE sources
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cu)
else()
    file(GLOB_RECURSE sources
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
endif()

# Build executable
if (USE_CUDA)
    cuda_add_executable(${target} ${sources}
        OPTIONS ${CUDA_TARGET_OPTIONS}
    )
elseif (USE_HIP)
    file(GLOB_RECURSE hip_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cu)
    set_source_files_properties(${hip_sources} PROPERTIES LANGUAGE HIP)
    add_executable(${target} ${sources})
else()
    add_executable(${target} ${sources})
endif()

# Project options
set_target_properties(${target}
    PROPERTIES
    ${DEFAULT_PROJECT_OPTIONS}
)

# Compile options
target_compile_options(${target}
    PRIVATE

    PUBLIC
    ${DEFAULT_COMPILE_OPTIONS}

    INTERFACE
)
target_compile_definitions(${target}
    PRIVATE
    RESOURCES_DIR="${RESOURCES_DIR}"
)

# Link libraries
if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    target_link_libraries(${target}
        PRIVATE
        ${DEFAULT_LINKER_OPTIONS}
        CubbyFlow
        pystring)
else()
    target_link_libraries(${target}
        PRIVATE
        ${DEFAULT_LINKER_OPTIONS}
        CubbyFlow
        pystring)
endif()