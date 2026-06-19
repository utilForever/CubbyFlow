# Target name
set(target CubbyFlow)

# Define
set(root_dir ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# Includes
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Libraries
)

# Sources
file(GLOB header_dir
    ${root_dir}/Includes/Core)

if (USE_CUDA)
    file(GLOB_RECURSE sources
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CUDA/*.cu)
elseif (USE_HIP)
    file(GLOB_RECURSE sources
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CUDA/*.cu)
    # The CUDA directory's .cpp host sources also touch device types (device
    # memory, thrust device_ptr), so compile the whole directory with hipcc.
    file(GLOB_RECURSE hip_sources
        ${CMAKE_CURRENT_SOURCE_DIR}/CUDA/*.cu
        ${CMAKE_CURRENT_SOURCE_DIR}/CUDA/*.cpp)
    set_source_files_properties(${hip_sources} PROPERTIES LANGUAGE HIP)
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

    PUBLIC
    ${DEFAULT_COMPILE_OPTIONS}

    INTERFACE
)

target_link_libraries(${target}
    PRIVATE

    PUBLIC
    ${DEFAULT_LINKER_OPTIONS}
	${DEFAULT_LIBRARIES}

    INTERFACE
)

# Install
install(TARGETS ${target} DESTINATION lib)
install(DIRECTORY ${header_dir} DESTINATION include)