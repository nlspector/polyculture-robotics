cmake_minimum_required(VERSION 3.8)

project(RealSenseGrabCutsExample)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

add_executable(redball redball.cpp ../cv-helpers.hpp)
set_property(TARGET redball PROPERTY CXX_STANDARD 11)
target_link_libraries(redball ${DEPENDENCIES})
target_link_libraries(redball ${OpenCV_LIBS})
target_link_libraries(redball ${realsense2_LIBRARY})
set_target_properties (redball PROPERTIES
	FOLDER "Examples/OpenCV"
)

install(
	TARGETS

	redball

	RUNTIME DESTINATION
	${CMAKE_INSTALL_PREFIX}/bin
)
