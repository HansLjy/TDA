cmake_minimum_required(VERSION 3.22)

add_library(LocalCircularCoordinate INTERFACE)

target_include_directories(LocalCircularCoordinate INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Eigen3 3.4.0 EXACT REQUIRED)
find_package(spdlog REQUIRED)

target_link_libraries(LocalCircularCoordinate INTERFACE Eigen3::Eigen)
target_link_libraries(LocalCircularCoordinate INTERFACE spdlog::spdlog)

find_package(CGAL QUIET REQUIRED COMPONENTS Core)
if (CGAL_VERSION VERSION_LESS 4.11.0)
    message(FATAL_ERROR "CGAL is considered too old to be used by Gudhi. Must be > 4.11.")
endif()

target_link_libraries(LocalCircularCoordinate INTERFACE CGAL::CGAL CGAL::CGAL_Core)

find_package(GUDHI 2.0.0 REQUIRED)
target_include_directories(LocalCircularCoordinate INTERFACE ${GUDHI_INCLUDE_DIRS})
