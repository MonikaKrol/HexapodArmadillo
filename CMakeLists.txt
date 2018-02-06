cmake_minimum_required(VERSION 2.8)
project( hex )

set(CMAKE_CXX_STANDARD 11)

set(SOURCES
		src/main.cpp
		src/Robot/Point.h src/Robot/Point.cpp
		src/Robot/leg.h	src/Robot/leg.cpp
		src/Robot/robot.h src/Robot/robot.cpp
		src/Robot/robotcontroler.h src/Robot/robotcontroler.cpp
		src/Robot/robotwalk.h src/Robot/robotwalk.cpp
		src/maestro.h src/maestro.cpp
		src/util.h
		src/TCP/tcpconnector.h src/TCP/tcpconnector.cpp
		src/TCP/tcpacceptor.h src/TCP/tcpacceptor.cpp
		src/TCP/tcpstream.h src/TCP/tcpstream.cpp)

set( GCC_COVERAGE_LINK_FLAGS "-O1 -larmadillo -llapack -lblas" )
set( GCC_COVERAGE_COMPILES_FLAGS "-DARMA_DONT_USE_WRAPPER -std=c++11")
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
set( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINKS_FLAGS}" )
include_directories( ${PROJECT_SOURCE_DIR} ${ARMADILLO_INCLUDE_DIRS})
add_executable( hex ${SOURCES} )
target_link_libraries( hex ${ARMADILLO_LIBRARIES} )
