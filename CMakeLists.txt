cmake_minimum_required(VERSION 3.5)

project(sslsim)
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
find_package(Qt5 COMPONENTS Core REQUIRED)
list(APPEND libs Qt5::Core)

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
#set_property(SOURCE ${PROTO_FILES} PROPERTY SKIP_AUTOMOC ON)
include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/physics
	${CMAKE_SOURCE_DIR}/utils
)

file(GLOB CPPFILES 
	"${CMAKE_SOURCE_DIR}/utils/*.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/physics/*.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/physics/*.h"
	"${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)
set(HEADERS
		sslworld.hpp
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

if(APPLE)
	set(CMAKE_SHARED_LIBRARY_SUFFIX ".so")
endif()

add_library(sslsim SHARED ${srcs})
target_link_libraries(sslsim ${libs})
install(TARGETS sslsim LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)


add_executable(gymsim_test test/main.cpp)
target_link_libraries(gymsim_test sslsim)
install(TARGETS gymsim_test LIBRARY DESTINATION bin ARCHIVE DESTINATION bin)

add_library(sslsim-c-lib SHARED pygrsim/pygrsim_c_wrapper.cpp pygrsim/pygrsim_c_wrapper.h)
target_link_libraries(sslsim-c-lib sslsim)
install(TARGETS sslsim-c-lib LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
