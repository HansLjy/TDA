cmake_minimum_required(VERSION 3.22)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(TDA)

set(3D_MODEL_DIR ${CMAKE_CURRENT_SOURCE_DIR})
cmake_path(APPEND 3D_MODEL_DIR data 3d-models)
set(HIGH_DIM_MODEL_DIR ${CMAKE_CURRENT_SOURCE_DIR})
cmake_path(APPEND HIGH_DIM_MODEL_DIR data high-dim-models)
set(RAW_DATA_DIR ${CMAKE_CURRENT_SOURCE_DIR})
cmake_path(APPEND RAW_DATA_DIR data raw)

add_library(tda_compile_flags INTERFACE)
target_compile_features(tda_compile_flags INTERFACE cxx_std_17)
target_compile_options(
	tda_compile_flags INTERFACE
	"$<$<CONFIG:DEBUG>:-O0;-ggdb;-Wall>"
	"$<$<CONFIG:RELEASE>:-O3>"
)
target_link_options(
	tda_compile_flags INTERFACE
	"$<$<CONFIG:DEBUG>:-g>"
)

add_subdirectory(3rdParty)
add_subdirectory(src)
if (BUILD_DATA_GENERATORS)
	add_subdirectory(data)
endif()
if (BUILD_TESTS)
	add_subdirectory(test)
endif()