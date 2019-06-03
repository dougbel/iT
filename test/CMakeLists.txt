
project(it_lib)

set(PROJECT_MAIN_TRAINING_TEST ${PROJECT_NAME}_main_training_test)
set(PROJECT_MAIN_TESTING_TEST ${PROJECT_NAME}_main_testing_test)


#for compiler
set(THIRD_PARTY_SOURCES ${IBS_DIR}/third_party/StopWatch/StopWatch.cpp)
set(THIRD_PARTY_HEADERS ${IBS_DIR}/third_party/StopWatch/header)

include_directories(${THIRD_PARTY_HEADERS})

#for testing IT_CALCULATION
add_executable(${PROJECT_MAIN_TRAINING_TEST}  ${IT_SOURCES} ${THIRD_PARTY_SOURCES} main_training_test.cpp)

set_target_properties(${PROJECT_MAIN_TRAINING_TEST} PROPERTIES CXX_STANDARD 11  CXX_STANDARD_REQUIRED ON  CXX_EXTENSIONS OFF)
target_link_libraries (${PROJECT_MAIN_TRAINING_TEST} ${QHULL_LIBRARIES} ${PCL_LIBRARIES} )


#for test IT_TESTING
add_executable(${PROJECT_MAIN_TESTING_TEST}  ${IT_SOURCES} ${THIRD_PARTY_SOURCES} main_testing_test.cpp)

set_target_properties(${PROJECT_MAIN_TESTING_TEST} PROPERTIES CXX_STANDARD 11  CXX_STANDARD_REQUIRED ON  CXX_EXTENSIONS OFF)
target_link_libraries (${PROJECT_MAIN_TESTING_TEST} ${QHULL_LIBRARIES} ${PCL_LIBRARIES} )
