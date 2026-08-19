set(target Particles2Vol)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB sources
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_executable(${target}
    ${sources})

set_target_properties(${target}
    PROPERTIES
    ${DEFAULT_PROJECT_OPTIONS}
)

target_compile_options(${target}
    PRIVATE
    ${DEFAULT_COMPILE_OPTIONS}
)

target_link_libraries(${target}
    PRIVATE
    ${DEFAULT_LINKER_OPTIONS}
    CubbyFlow)
