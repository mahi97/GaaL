cmake_minimum_required(VERSION 3.5)

project(gymsim)
# Simulator Plugin
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)
include(${PROJECT_SOURCE_DIR}/cmake/Utils.cmake)
standard_config()
standard_paths(${PROJECT_SOURCE_DIR} bin lib)

set(libs)
# OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})
list(APPEND libs ${OPENGL_LIBRARIES})

# Qt5
find_package(Qt5 COMPONENTS Core Network Gui REQUIRED)
list(APPEND libs Qt5::Core Qt5::Network Qt5::Gui)

# ODE
find_package(ODE REQUIRED)
include_directories(${ODE_INCLUDE_DIRS})
list(APPEND libs ${ODE_LIBRARIES})
add_definitions(-DdDOUBLE)

# Protobuf
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
list(APPEND libs ${PROTOBUF_LIBRARIES})

file(GLOB PROTO_FILES "${CMAKE_SOURCE_DIR}/proto/*.proto")
protobuf_generate_cpp(PROTO_CPP PROTO_H ${PROTO_FILES})

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/physics
	${CMAKE_SOURCE_DIR}/Utils
)

file(GLOB CPPFILES 
	"${CMAKE_SOURCE_DIR}/Utils/*.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/physics/*.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/physics/*.h"
	"${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)
set(HEADERS
	sslworld.h
)
set(SOURCES
	sslworld.cpp
)
set(srcs
    ${PROTO_CPP}
    ${PROTO_H}
    ${HEADERS}
    ${SOURCES}
    ${CPPFILES}
)

add_library(gymsim SHARED ${srcs})

target_link_libraries(gymsim ${libs})

install(TARGETS gymsim
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)