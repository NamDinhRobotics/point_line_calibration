cmake_minimum_required(VERSION 3.26)
project(testl3l)

set(CMAKE_CXX_STANDARD 14)

set(COLMAP_LIBRARIES
        colmap
        ${COLMAP_INTERNAL_LIBRARIES}
        ${COLMAP_EXTERNAL_LIBRARIES}
)
#set directory for PoseLib to "/home/dinhnambkhn/PoseLib/_install"
#find eigen
find_package(Eigen3 REQUIRED)
# find PoseLib in directory "/home/dinhnambkhn/PoseLib/_install"
find_package(PoseLib REQUIRED)
# find openCV
find_package(OpenCV REQUIRED)
#include directory of OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(testl3l main.cpp)
target_link_libraries(testl3l PRIVATE PoseLib::PoseLib)

add_executable(test_robust_p3ll test_robust_p3ll.cpp)
target_link_libraries(test_robust_p3ll PRIVATE PoseLib::PoseLib)

add_executable(test_undistor test_undistor.cpp)
target_link_libraries(test_undistor PRIVATE PoseLib::PoseLib ${OpenCV_LIBS})

add_executable(test_use_only_line use_onlyLine.cpp)
target_link_libraries(test_use_only_line PRIVATE PoseLib::PoseLib ${OpenCV_LIBS})
