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

add_executable(pcd2pic pcd2pic.cpp)
target_link_libraries(pcd2pic ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} )

add_executable(pcd2csv pcd2csv.cpp)
target_link_libraries(pcd2csv ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} )

add_executable(csv2pcd csv2pcd.cpp)
target_link_libraries(csv2pcd ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} )


# add_executable (SimpleOpenNIViewer SimpleOpenNIViewer.cpp)
# target_link_libraries (SimpleOpenNIViewer ${PCL_LIBRARIES})

# add_executable (openni_viewer OpenNIViewer.cpp)
# target_link_libraries (openni_viewer ${PCL_LIBRARIES})

