
set(PROJECT_MAIN_TEST ${PROJECT_NAME}_main_test)


#for compiler
set(THIRD_PARTY_SOURCES ${IBS_DIR}/third_party/StopWatch/StopWatch.cpp)
set(THIRD_PARTY_HEADERS ${IBS_DIR}/third_party/StopWatch/header)

include_directories(${THIRD_PARTY_HEADERS})

#for testing
add_executable(${PROJECT_MAIN_TEST}  ${IT_SOURCES} ${THIRD_PARTY_SOURCES} main_test.cpp)

set_target_properties(${PROJECT_MAIN_TEST} PROPERTIES CXX_STANDARD 11  CXX_STANDARD_REQUIRED ON  CXX_EXTENSIONS OFF)
target_link_libraries (${PROJECT_MAIN_TEST} ${QHULL_LIBRARIES} ${PCL_LIBRARIES} )
