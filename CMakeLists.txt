cmake_minimum_required(VERSION 3.22)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(TDA)

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
add_subdirectory(test)