# Target name
set(target CubbyFlow)

# Define
set(root_dir ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# Includes
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Sources
file(GLOB header_dir
    ${root_dir}/Includes/Core)

if (USE_CUDA)
    file(GLOB_RECURSE sources
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CUDA/*.cu)
else()
    file(GLOB_RECURSE sources
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
endif()

# Build library
if (USE_CUDA)
    cuda_add_library(${target} ${sources} STATIC
        OPTIONS ${CUDA_TARGET_OPTIONS}
    )
else()
    add_library(${target} STATIC
        ${sources})
endif()

# Project options
set_target_properties(${target}
    PROPERTIES
    ${DEFAULT_PROJECT_OPTIONS}
)

# Compile options
target_compile_options(${target}
    PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/wd4244>

    PUBLIC
    ${DEFAULT_COMPILE_OPTIONS}

    INTERFACE
)

target_link_libraries(${target}
    PRIVATE

    PUBLIC
    ${DEFAULT_LINKER_OPTIONS}
	${DEFAULT_LIBRARIES}

    PRIVATE
    tinyobjloader::tinyobjloader_double

    INTERFACE
)

target_include_directories(${target}
    SYSTEM
    PRIVATE
    ${FLATBUFFERS_INCLUDE_DIR}
)

# Install
install(TARGETS ${target} DESTINATION lib)
install(DIRECTORY ${header_dir} DESTINATION include)
