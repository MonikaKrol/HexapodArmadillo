cmake_minimum_required(VERSION 2.8)
project( hex )

set(CMAKE_CXX_STANDARD 11)

set(OpenCV_INCLUDE_DIRS
	/home/pi/opencv/opencv/¬/home/pi/opencv/build/include
    /home/pi/opencv/opencv/¬/home/pi/opencv/build/include/opencv2
    ${PROJECT_SOURCE_DIR}/src
)

set(OpenCV_LIB_DIR
	/home/pi/opencv/opencv/¬/home/pi/opencv/build/lib
)

set(OpenCV_LIBS
	opencv_core
	opencv_highgui
	pthread
)

set(SOURCES
	src/main.cpp
	src/Robot/leg.h src/Robot/leg.cpp
	src/Robot/robot.h src/Robot/robot.cpp
	src/Robot/robotcontroler.h src/Robot/robotcontroler.cpp
	src/Robot/robotwalk.h src/Robot/robotwalk.cpp
	src/maestro.h src/maestro.cpp
	src/util.h
	src/TCP/tcpconnector.h src/TCP/tcpconnector.cpp
	src/TCP/tcpacceptor.h src/TCP/tcpacceptor.cpp
	src/TCP/tcpstream.h src/TCP/tcpstream.cpp src/Robot/Point.cpp)

include_directories( ${OpenCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR})
link_directories( ${OpenCV_LIB_DIR} )
add_executable( hex ${SOURCES} )
target_link_libraries( hex ${OpenCV_LIBS} )
