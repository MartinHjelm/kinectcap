#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
      
int 
main(int argc, char ** argv)
{

    // Load point cloud from file
    std::string fName = "";
    if(argv[1]) fName = std::string(argv[1]);
    else return 0;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile (fName, *cloud);
    
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    
    viewer.showCloud(cloud);
    
    while (!viewer.wasStopped ())
    {
      // viewer.spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return 0;
}