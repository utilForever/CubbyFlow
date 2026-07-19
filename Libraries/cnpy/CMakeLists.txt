cmake_minimum_required(VERSION 3.12)
project(cnpy LANGUAGES CXX)

find_package(ZLIB REQUIRED)

add_library(cnpy cnpy.cpp)
target_link_libraries(cnpy PUBLIC ZLIB::ZLIB)
target_include_directories(cnpy PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>)

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/cnpy-config.cmake"
    "include(CMakeFindDependencyMacro)\nfind_dependency(ZLIB)\ninclude(\"\${CMAKE_CURRENT_LIST_DIR}/cnpy-targets.cmake\")\n")

install(TARGETS cnpy EXPORT cnpy-targets)
install(FILES cnpy.h DESTINATION include/cnpy)
install(EXPORT cnpy-targets NAMESPACE cnpy:: DESTINATION share/cnpy)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/cnpy-config.cmake" DESTINATION share/cnpy)
