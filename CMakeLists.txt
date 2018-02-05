cmake_minimum_required(VERSION 2.8)
project( hex )

set(CMAKE_CXX_STANDARD 11)

set(SOURCES
	src/main.cpp
	src/Robot/Point.h src/Robot/Point.cpp
	src/Robot/leg.h src/Robot/leg.cpp
	src/Robot/robot.h src/Robot/robot.cpp
	src/Robot/robotcontroler.h src/Robot/robotcontroler.cpp
	src/Robot/robotwalk.h src/Robot/robotwalk.cpp
	src/maestro.h src/maestro.cpp
	src/util.h
	src/TCP/tcpconnector.h src/TCP/tcpconnector.cpp
	src/TCP/tcpacceptor.h src/TCP/tcpacceptor.cpp
	src/TCP/tcpstream.h src/TCP/tcpstream.cpp)

include_directories( ${PROJECT_SOURCE_DIR} ${libopenblas} ${liblapack})
add_executable( hex ${SOURCES} )
