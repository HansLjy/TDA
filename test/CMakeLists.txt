cmake_minimum_required(VERSION 3.22)

configure_file(TestConfig.hpp.in TestConfig.hpp)

add_executable(3d-model 3d-model.cc)
target_link_libraries(3d-model PRIVATE TDA)
target_include_directories(3d-model PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(3d-model PRIVATE tda_compile_flags)

add_executable(high-dim-model high-dim-model.cc)
target_link_libraries(high-dim-model PRIVATE TDA)
target_include_directories(high-dim-model PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(high-dim-model PRIVATE tda_compile_flags)

if (BUILD_UNIT_TESTS)
	find_package(GTest REQUIRED)

	add_executable(test test.cc)
	target_link_libraries(test PRIVATE TDA)
	target_include_directories(test PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
	target_link_libraries(test PRIVATE tda_compile_flags)

	enable_testing()
	include(GoogleTest)
	gtest_discover_tests(test)
	target_link_libraries(test PRIVATE GTest::gtest_main)
endif()