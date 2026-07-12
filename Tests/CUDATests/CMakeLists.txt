# Target name
set(target CUDATests)

# Sources
file(GLOB_RECURSE sources
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cu)

# Build executable
if (USE_HIP)
    file(GLOB_RECURSE hip_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cu)
    set_source_files_properties(${hip_sources} PROPERTIES LANGUAGE HIP)
    add_executable(${target} ${sources})
else()
    cuda_add_executable(${target} ${sources}
        OPTIONS ${CUDA_TARGET_OPTIONS}
    )
endif()

# Project options
set_target_properties(${target}
    PROPERTIES
    ${DEFAULT_PROJECT_OPTIONS}
)

# Compile options
# GCC and Clang compiler options
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(DEFAULT_COMPILE_OPTIONS ${DEFAULT_COMPILE_OPTIONS}
        -Wno-unused-variable
        )
endif ()
target_compile_options(${target}
    PRIVATE
    ${DEFAULT_COMPILE_OPTIONS}
)
target_compile_definitions(${target}
    PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:DOCTEST_CONFIG_USE_STD_HEADERS>
    RESOURCES_DIR="${RESOURCES_DIR}"
)

# Link libraries
target_link_libraries(${target}
    PRIVATE
    ${DEFAULT_LINKER_OPTIONS}
    CubbyFlow)
