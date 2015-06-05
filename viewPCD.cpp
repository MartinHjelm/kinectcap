#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>     


int 
main(int argc, char ** argv)
{

  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointXYZ PointT2;
  bool pcTypeRGB = true;

  // Load point cloud from file
  std::string fName = "";
  if(argv[1]) 
  {
    fName = std::string(argv[1]);
    if(fName == "-h")
    {
      std::cout << "Usage: viewpcd fileName xyz. xyz argument used if you want to remove color." << std::endl;   
      return -1;
    }
  }
  else 
  {
    std::cout << "Please specify a file to view." << std::endl;
    std::cout << "Usage: viewpcd fileName xyz. xyz argument used if you want to remove color." << std::endl;   
    return -1;
  }
  
  if(argc>2) 
  {
    if(std::string(argv[2])=="xyz") // Get only xyz no color data
      pcTypeRGB = false;
  }

  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT2>::Ptr cloud2 (new pcl::PointCloud<PointT2>);


  pcl::PCDReader reader;
  if(pcTypeRGB)
    reader.read (fName, *cloud);  
  else
    reader.read (fName, *cloud2);

  // View Point Cloud
  pcl::visualization::PCLVisualizer::Ptr viewer (
    new pcl::visualization::PCLVisualizer ("Point Cloud from: "+fName)
  );

  if(pcTypeRGB){
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer->addPointCloud<PointT> (cloud, rgb, "Cloud");
  }
  else 
  {
    viewer->addPointCloud<PointT2> (cloud2,"Cloud");
  }
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud");  
  viewer->setBackgroundColor (0, 0, 0);
  // viewer->addCoordinateSystem (0.25,0);
  viewer->setCameraPosition(0, 0, -3, 0, -1, 0);

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (10000));
  }
  
  return 0;
}