cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(KinectCapture)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED filesystem program_options)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")

add_executable(kcap KinectCapture.cpp)
target_link_libraries(kcap ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(viewpcd viewPCD.cpp)
target_link_libraries(viewpcd ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} )