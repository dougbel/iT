
cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project(it_lib)

# Find includes in corresponding build directories
#set(CMAKE_INCLUDE_CURRENT_DIR ON)


# Find PCL library
find_package(PCL 1.8 REQUIRED)
find_package (Eigen3 REQUIRED )
add_definitions(${PCL_DEFINITIONS})


#QHull dependencies
set(QHULL_LIBRARIES /usr/local/lib/libqhullcpp.a /usr/local/lib/libqhullstatic_r.a)


#IBS dependencies
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libs/IBS)


set(QHULL_LIBRARIES /usr/local/lib/libqhullcpp.a /usr/local/lib/libqhullstatic_r.a)

#for reference IT
set(IT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(IT_SOURCES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(IT_HEADERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(IT_SOURCES ${IBS_SOURCES}  ${IT_SOURCES_DIR}/it.cpp ${IT_SOURCES_DIR}/provenancevectors_it.cpp ${IT_SOURCES_DIR}/sampleruniform_it.cpp ${IT_SOURCES_DIR}/samplerweighted_it.cpp)#TODO Agregar los cpp que realizan el calculo del interaction tensor


#message(STATUS "CMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}")
#message(STATUS "CMAKE_CURRENT_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "IBS_DIR=${IBS_DIR}")
message(STATUS "IBS_SOURCES_DIR=${IBS_SOURCES_DIR}")
message(STATUS "IBS_HEADERS_DIR=${IBS_HEADERS_DIR}")
message(STATUS "IBS_SOURCES=${IBS_SOURCES}")
message(STATUS "IT_DIR=${IT_DIR}")
message(STATUS "IT_SOURCES_DIR=${IT_SOURCES_DIR}")
message(STATUS "IT_HEADERS_DIR=${IT_HEADERS_DIR}")
message(STATUS "IT_SOURCES=${IT_SOURCES}")

#include directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${QHULL_INCLUDE_DIR})
include_directories(${IBS_HEADERS_DIR})
include_directories(${IT_HEADERS_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})


#add_library(IT_lib STATIC  ${IT_SOURCES} )

#set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11  CXX_STANDARD_REQUIRED ON  CXX_EXTENSIONS OFF)
#target_link_libraries (${PROJECT_NAME} ${QHULL_LIBRARIES} ${PCL_LIBRARIES})


add_subdirectory(apps)
#add_subdirectory(test)
