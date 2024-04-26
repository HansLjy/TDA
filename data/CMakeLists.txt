cmake_minimum_required(VERSION 3.22)

configure_file(DataConfig.hpp.in DataConfig.hpp)

include(FetchContent)

FetchContent_Declare(
	get_cpm
	URL	https://github.com/cpm-cmake/CPM.cmake/releases/latest/download/get_cpm.cmake
	DOWNLOAD_NO_EXTRACT ON
)

FetchContent_MakeAvailable(
	get_cpm
)

FetchContent_GetProperties(
	get_cpm
	SOURCE_DIR get_cpm_DIR
)

include(${get_cpm_DIR}/get_cpm.cmake)

message("-- downloading ezc3d ...")
CPMAddPackage (
    NAME ezc3d
    GIT_REPOSITORY git@github.com:pyomeca/ezc3d.git
	GIT_TAG 39c4e7c14c868de8f75417b8f6df06f9adedf490
)
message("-- ezc3d download success")

add_executable(gen-circle gen-circle.cc)
target_include_directories(gen-circle PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

find_package(Eigen3 3.4.0 REQUIRED)
add_executable(transform-c3d transform-c3d.cc)
target_include_directories(transform-c3d PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(transform-c3d Eigen3::Eigen)
target_link_libraries(transform-c3d ezc3d)