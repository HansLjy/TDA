cmake_minimum_required(VERSION 3.22)

set(TDA_MODEL_DIR ${CMAKE_CURRENT_SOURCE_DIR})
cmake_path(APPEND TDA_MODEL_DIR models)
configure_file(TestConfig.hpp.in TestConfig.hpp)

add_executable(3d-model 3d-model.cc)
target_link_libraries(3d-model PRIVATE TDA)
target_include_directories(3d-model PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(3d-model PRIVATE tda_compile_flags)
