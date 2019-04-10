

set(PROJECT_MAIN ${PROJECT_NAME}_main)


#include_directories(${THIRD_PARTY_HEADERS})


add_executable  (${PROJECT_MAIN}  ${IT_SOURCES}  main.cpp)


set_target_properties(${PROJECT_MAIN} PROPERTIES CXX_STANDARD 11  CXX_STANDARD_REQUIRED ON  CXX_EXTENSIONS OFF)
target_link_libraries (${PROJECT_MAIN} ${QHULL_LIBRARIES} ${PCL_LIBRARIES} ${ibs_lib})
