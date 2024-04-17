cmake_minimum_required(VERSION 3.22)

set(3D_MODEL_DIR ${CMAKE_CURRENT_SOURCE_DIR})
cmake_path(APPEND 3D_MODEL_DIR 3d-models)
set(HIGH_DIM_MODEL_DIR ${CMAKE_CURRENT_SOURCE_DIR})
cmake_path(APPEND HIGH_DIM_MODEL_DIR high-dim-models)
set(RAW_DATA_DIR ${CMAKE_CURRENT_SOURCE_DIR})
cmake_path(APPEND RAW_DATA_DIR raw)
configure_file(DataConfig.hpp.in DataConfig.hpp)

include(FetchContent)

FetchContent_Declare(
    ezc3d
    GIT_REPOSITORY git@github.com:pyomeca/ezc3d.git
    GIT_TAG        39c4e7c14c868de8f75417b8f6df06f9adedf490
)

FetchContent_MakeAvailable(ezc3d)

if (BUILD_DATA_GENERATOR)
    add_executable(gen-circle gen-circle.cc)
    target_include_directories(gen-circle PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

    find_package(Eigen3 3.4.0 REQUIRED)
    add_executable(transform-c3d transform-c3d.cc)
	target_include_directories(transform-c3d PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
    target_link_libraries(transform-c3d Eigen3::Eigen)
    target_link_libraries(transform-c3d ezc3d)
endif()