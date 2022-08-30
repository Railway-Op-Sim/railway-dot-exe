cmake_minimum_required( VERSION 3.21 )

set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED True )

set( RAILOS_LIBS "railos" )
set( DATE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/date/include )

project( RAILWAY_OPERATION_SIMULATOR VERSION 2022.08.29 LANGUAGES CXX )

include( ${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake )
conan_basic_setup()

file( GLOB RAILOS_SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cxx )

add_library( ${RAILOS_LIBS} ${RAILOS_SRC_FILES} )

target_include_directories( ${RAILOS_LIBS} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include )
target_include_directories( ${RAILOS_LIBS} PUBLIC ${DATE_INCLUDE_DIR} )
