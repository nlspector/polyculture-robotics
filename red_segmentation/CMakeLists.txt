cmake_minimum_required(VERSION 3.8)

project(RealSenseGrabCutsExample)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

add_executable(rs-grabcuts rs-grabcuts.cpp ../cv-helpers.hpp)
set_property(TARGET rs-grabcuts PROPERTY CXX_STANDARD 11)
target_link_libraries(rs-grabcuts ${DEPENDENCIES})
target_link_libraries(rs-grabcuts ${OpenCV_LIBS})
target_link_libraries(rs-grabcuts ${realsense2_LIBRARY})
set_target_properties (rs-grabcuts PROPERTIES
	FOLDER "Examples/OpenCV"
)

install(
	TARGETS

	rs-grabcuts

	RUNTIME DESTINATION
	${CMAKE_INSTALL_PREFIX}/bin
)
