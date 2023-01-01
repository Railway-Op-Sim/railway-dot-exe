cmake_minimum_required( VERSION 3.21 )

set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED True )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( RAILOS_THIRD_PARTY ${CMAKE_CURRENT_SOURCE_DIR}/external )
set( RAILOS_VERSION UNKNOWN )

if(WIN32)
    execute_process(COMMAND "cmd" " /C date /T" OUTPUT_VARIABLE RAILOS_VERSION)
    string(REGEX REPLACE "(..)/(..)/(....).*" "\\3.\\2.\\1" RAILOS_VERSION ${RAILOS_VERSION})
elseif(UNIX)
    execute_process(COMMAND "date" "+%Y.%m.%d" OUTPUT_VARIABLE RAILOS_VERSION)
    string(STRIP ${RAILOS_VERSION} RAILOS_VERSION)
else()
    message(FATAL_ERROR "Unable to detect date")
endif()

set( RAILOS_LIBS "railos_core" )
set( RAILOS_UI "railos_ui" )
set( RAILOS_APPLICATION_NAME "Railway Operation Simulator" )
set( RAILOS_BINARY_NAME "railos" )

message( STATUS "Building ${RAILOS_APPLICATION_NAME} v${RAILOS_VERSION}" )

project( RAILWAY_OPERATION_SIMULATOR VERSION ${RAILOS_VERSION} LANGUAGES CXX )

file( GLOB RAILOS_SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/core/*.cxx )
file( GLOB RAILOS_UI_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/ui/*.cxx )

add_subdirectory( ${RAILOS_THIRD_PARTY}/boost-cmake )
add_subdirectory( ${RAILOS_THIRD_PARTY}/nana )

set( BOOST_URL "https://boostorg.jfrog.io/artifactory/main/release/1.81.0/source/boost_1_81_0.tar.bz2" )
set( BOOST_LIBS_OPTIONAL
    chrono
    date_time
)

add_library( ${RAILOS_LIBS} ${RAILOS_SRC_FILES} )
add_library( ${RAILOS_UI} ${RAILOS_UI_FILES} )

target_include_directories( ${RAILOS_LIBS} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include )
target_include_directories( ${RAILOS_UI} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include )

set( CMAKE_CXX_FLAGS "-DRAILOS_GRAPHICS_ROOT='\"${CMAKE_CURRENT_SOURCE_DIR}/graphics\"'\
 -DRAILOS_APPLICATION_NAME='\"${RAILOS_APPLICATION_NAME}\"'\
 -DRAILOS_VERSION='\"${RAILOS_VERSION}\"'" 
)

# --------- SETUP NANA ----------------- #
target_compile_options( nana PUBLIC -w )
set( NANA_CMAKE_ENABLE_JPEG ON )

target_link_libraries( ${RAILOS_UI} nana )

add_executable( ${RAILOS_BINARY_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/application.cxx )
target_link_libraries( ${RAILOS_BINARY_NAME} ${RAILOS_LIBS} ${RAILOS_UI} )
target_link_libraries( ${RAILOS_BINARY_NAME} nana )
target_include_directories( ${RAILOS_BINARY_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include )
