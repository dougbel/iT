
project(it_lib)

set(PROJECT_MAIN_TRAINING ${PROJECT_NAME}_main_training)
set(PROJECT_MAIN_TESTING ${PROJECT_NAME}_main_testing)

#for compiler
set(THIRD_PARTY_SOURCES ${IBS_DIR}/third_party/StopWatch/StopWatch.cpp)
set(THIRD_PARTY_HEADERS ${IBS_DIR}/third_party/StopWatch/header)


include_directories(${THIRD_PARTY_HEADERS})




#TRAINING
add_executable  (${PROJECT_MAIN_TRAINING}  ${IT_SOURCES} ${THIRD_PARTY_SOURCES}  main_training.cpp)
set_target_properties(${PROJECT_MAIN_TRAINING} PROPERTIES CXX_STANDARD 11  CXX_STANDARD_REQUIRED ON  CXX_EXTENSIONS OFF)
target_link_libraries (${PROJECT_MAIN_TRAINING} ${QHULL_LIBRARIES} ${PCL_LIBRARIES} ${ibs_lib})





# #TEST
add_executable  (${PROJECT_MAIN_TESTING}  ${IT_SOURCES} ${IT_TESTING_SOURCES} ${THIRD_PARTY_SOURCES}  main_testing.cpp)
set_target_properties(${PROJECT_MAIN_TESTING} PROPERTIES CXX_STANDARD 11  CXX_STANDARD_REQUIRED ON  CXX_EXTENSIONS OFF)
target_link_libraries (${PROJECT_MAIN_TESTING}  myCuda  dev_array ${QHULL_LIBRARIES}  ${PCL_LIBRARIES} ${ibs_lib})


