cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(KinectCapture)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(kcap KinectCapture.cpp)
target_link_libraries(kcap ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(viewpcd viewPCD.cpp)
target_link_libraries(viewpcd ${PCL_LIBRARIES} ${OpenCV_LIBS})