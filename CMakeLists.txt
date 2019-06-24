cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project(it_lib)

# Find includes in corresponding build directories
#set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find PCL library
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED )
# From the PCL src files, To solve cuda flags and stuff
include(${CMAKE_SOURCE_DIR}/cmake/find_cuda.cmake)


add_definitions(${PCL_DEFINITIONS})

#IBS dependencies
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libs/IBS)

#QHull dependencies
set(QHULL_LIBRARIES /usr/local/lib/libqhullcpp.a /usr/local/lib/libqhullstatic_r.a)


#for reference IT
set(IT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(IT_SOURCES_DIR ${IT_DIR}/src)


set(IT_HEADERS_DIR ${IT_DIR}/include)
set(IT_SOURCES ${IBS_SOURCES}  ${IT_SOURCES_DIR}/it.cpp 
                                ${IT_SOURCES_DIR}/provenancevectors_it.cpp 
                                ${IT_SOURCES_DIR}/sampleruniform_it.cpp 
                                ${IT_SOURCES_DIR}/samplerweighted_it.cpp
                                ${IT_SOURCES_DIR}/spinner_it.cpp
                                ${IT_SOURCES_DIR}/agglomerator_it.cpp
                                ${IT_SOURCES_DIR}/densifier.cpp   )
                                

                                
set(IT_TESTING_HEADERS_DIR ${IT_HEADERS_DIR}/Testing)
set(IT_TESTING_SOURCES ${IT_SOURCES_DIR}/Testing/it_testing.cpp)



#message(STATUS "CMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}")
#message(STATUS "CMAKE_CURRENT_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "PCL_DEFINITIONS = ${PCL_DEFINITIONS}")
message(STATUS "IBS_DIR         = ${IBS_DIR}")
message(STATUS "IBS_SOURCES_DIR = ${IBS_SOURCES_DIR}")
message(STATUS "IBS_HEADERS_DIR = ${IBS_HEADERS_DIR}")
message(STATUS "IBS_SOURCES     = ${IBS_SOURCES}")
message(STATUS "IT_DIR          = ${IT_DIR}")
message(STATUS "IT_SOURCES_DIR  = ${IT_SOURCES_DIR}")
message(STATUS "IT_HEADERS_DIR  = ${IT_HEADERS_DIR}")
message(STATUS "IT_TESTING_HEADERS_DIR=${IT_TESTING_HEADERS_DIR}")
message(STATUS "IT_SOURCES      = ${IT_SOURCES}")
message(STATUS "CMAKE_BINARY_DIR= ${CMAKE_BINARY_DIR}")



link_directories(${CMAKE_BINARY_DIR})

#include directories
include_directories(  ${PCL_INCLUDE_DIRS}
                      ${QHULL_INCLUDE_DIR}
                      ${IBS_HEADERS_DIR}
                      ${IT_HEADERS_DIR}
                      ${IT_TESTING_HEADERS_DIR}
                      ${EIGEN3_INCLUDE_DIR}
                      ${CUDA_INCLUDE_DIRS}   )


                      
                      
#build libraries and cuda files to use
CUDA_ADD_LIBRARY(dev_array   SHARED   ${IT_TESTING_HEADERS_DIR}/dev_array.h )

TARGET_LINK_LIBRARIES(dev_array   ${CUDA_LIBRARIES} )




#add_library(IT_lib STATIC  ${IT_SOURCES} )
#set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11  CXX_STANDARD_REQUIRED ON  CXX_EXTENSIONS OFF)
#target_link_libraries (${PROJECT_NAME} ${QHULL_LIBRARIES} ${PCL_LIBRARIES})


add_subdirectory(apps)
add_subdirectory(test)
