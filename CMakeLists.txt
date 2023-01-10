cmake_minimum_required( VERSION 3.21 )

if(WIN32)
    set(CMAKE_CXX_COMPILER g++ )
endif()

set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( RAILOS_THIRD_PARTY ${CMAKE_CURRENT_SOURCE_DIR}/external )
string( TIMESTAMP RAILOS_VERSION "%Y.%m.%d" )

set( RAILOS_LIBS "railos_core" )
set( RAILOS_UI "railos_ui" )
set( RAILOS_BINARY_NAME "railos" )
set( BOOST_SOURCE ${CMAKE_CURRENT_BINARY_DIR}/boost-src )

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
target_include_directories( ${RAILOS_LIBS} PUBLIC ${BOOST_SOURCE} )
message( ${BOOST_SOURCE} )

target_compile_definitions( ${RAILOS_UI} PUBLIC RAILOS_GRAPHICS_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/graphics" )
target_compile_definitions( ${RAILOS_UI} PUBLIC RAILOS_APPLICATION_NAME="Railway_Operation_Simulator" )
target_compile_definitions( ${RAILOS_UI} PUBLIC RAILOS_VERSION="${RAILOS_VERSION}" )

# --------- SETUP NANA ----------------- #
target_compile_options( nana PUBLIC -w )
set( NANA_CMAKE_ENABLE_JPEG ON )

target_link_libraries( ${RAILOS_UI} nana )
target_include_directories( nana PUBLIC ${PROJECT_THIRD_PARTY}/libpng )
target_include_directories( nana PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/external/libpng )
target_include_directories( nana PUBLIC ${PROJECT_THIRD_PARTY}/zlib )

add_executable( ${RAILOS_BINARY_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/application.cxx )
target_link_libraries( ${RAILOS_BINARY_NAME} ${RAILOS_LIBS} ${RAILOS_UI} )
target_link_libraries( ${RAILOS_BINARY_NAME} nana )
target_include_directories( ${RAILOS_BINARY_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include )
