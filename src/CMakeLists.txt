cmake_minimum_required(VERSION 3.22)

include(dependencies.cmake)

file(GLOB TDA_SRC *.cc)

add_library(TDA ${TDA_SRC})

target_link_libraries(TDA PUBLIC Renderer)
target_link_libraries(TDA PUBLIC GUI)

target_include_directories(TDA PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(TDA PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(TDA PUBLIC tda_compile_flags)

target_link_libraries(TDA PUBLIC TDADependencies)
