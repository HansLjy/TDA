cmake_minimum_required(VERSION 3.22)

add_library(TDADependencies INTERFACE)

find_package(Eigen3 3.4.0 EXACT REQUIRED)
find_package(spdlog REQUIRED)

target_link_libraries(TDADependencies INTERFACE Eigen3::Eigen)
target_link_libraries(TDADependencies INTERFACE spdlog::spdlog)

find_package(CGAL QUIET REQUIRED COMPONENTS Core)
if (CGAL_VERSION VERSION_LESS 4.11.0)
    message(FATAL_ERROR "CGAL is considered too old to be used by Gudhi. Must be > 4.11.")
endif()

target_link_libraries(TDADependencies INTERFACE CGAL::CGAL CGAL::CGAL_Core)

find_package(GUDHI 2.0.0 REQUIRED)
message("GUDHI_VERSION = ${GUDHI_VERSION}")
message("GUDHI_INCLUDE_DIRS = ${GUDHI_INCLUDE_DIRS}")

target_include_directories(TDADependencies INTERFACE ${GUDHI_INCLUDE_DIRS})
