cmake_minimum_required(VERSION 3.22)

include(dependencies.cmake)

set(TDA_SHADER_DIR ${CMAKE_CURRENT_SOURCE_DIR})
cmake_path(APPEND TDA_SHADER_DIR shaders)
configure_file(TDAConfig.hpp.in TDAConfig.hpp)

file(GLOB TDA_SRC *.cc)

add_library(TDA ${TDA_SRC})

target_link_libraries(TDA PUBLIC Renderer)
target_link_libraries(TDA PUBLIC GUI)

target_include_directories(TDA PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(TDA PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(TDA PUBLIC tda_compile_flags)

target_link_libraries(TDA PUBLIC TDADependencies)
