cmake_minimum_required( VERSION 3.21 )

set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED True )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

set( RAILOS_LIBS "railos" )

project( RAILWAY_OPERATION_SIMULATOR VERSION 2022.08.29 LANGUAGES CXX )

file( GLOB RAILOS_SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cxx )

add_subdirectory( ${CMAKE_CURRENT_SOURCE_DIR}/external/boost-cmake )
add_subdirectory( ${CMAKE_CURRENT_SOURCE_DIR}/external/nana )

set( BOOST_URL "https://boostorg.jfrog.io/artifactory/main/release/1.81.0/source/boost_1_81_0.tar.bz2" )
set( BOOST_LIBS_OPTIONAL
    chrono
    date_time
)

add_library( ${RAILOS_LIBS} ${RAILOS_SRC_FILES} )

target_include_directories( ${RAILOS_LIBS} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include )
target_include_directories( ${RAILOS_LIBS} PUBLIC ${DATE_INCLUDE_DIR} )
target_compile_options( nana PUBLIC -w )

add_executable( ${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/application.cxx )
target_link_libraries( ${PROJECT_NAME} ${RAILOS_LIBS} )
target_link_libraries( ${PROJECT_NAME} nana )
