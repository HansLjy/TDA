cmake_minimum_required(VERSION 3.22)

project(TDA)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(main main.cc)

find_package(Eigen3 3.4.0 EXACT REQUIRED)
find_package(spdlog REQUIRED)

target_link_libraries(main Eigen3::Eigen)
target_link_libraries(main spdlog::spdlog)

find_package(CGAL QUIET REQUIRED COMPONENTS Core)
if (CGAL_VERSION VERSION_LESS 4.11.0)
    message(FATAL_ERROR "CGAL is considered too old to be used by Gudhi. Must be > 4.11.")
endif()

target_link_libraries(main CGAL::CGAL CGAL::CGAL_Core)

find_package(GUDHI 2.0.0 REQUIRED)
message("GUDHI_VERSION = ${GUDHI_VERSION}")
message("GUDHI_INCLUDE_DIRS = ${GUDHI_INCLUDE_DIRS}")
target_include_directories(main PRIVATE ${GUDHI_INCLUDE_DIRS})

add_subdirectory(src)