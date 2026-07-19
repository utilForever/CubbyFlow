# Target name
set(target ManualTests)

# Includes
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR})

# Sources
file(GLOB sources
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

# Build executable
add_executable(${target}
    ${sources})

# Project options
set_target_properties(${target}
    PROPERTIES
    ${DEFAULT_PROJECT_OPTIONS}
)

# Compile options
if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(DEFAULT_COMPILE_OPTIONS ${DEFAULT_COMPILE_OPTIONS}
        /wd4018       # -> disable warning: signed/unsigned mismatch (because of cnpy)
        /wd4100       # -> disable warning: unreferenced formal parameter (because of cnpy)
        /wd4310       # -> disable warning: cast truncates constant value (because of cnpy)
        /wd4996       # -> disable warning: non-Standard std::tr1 namespace and TR1-only machinery (because of gtest)
        )
    add_definitions(-D_USE_MATH_DEFINES) # for M_PI
endif ()

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
target_link_libraries(${target}
    PRIVATE
    ${DEFAULT_LINKER_OPTIONS}
    CubbyFlow
    cnpy
    gtest
    pystring)
