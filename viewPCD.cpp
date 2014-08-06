#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
      
int 
main(int argc, char ** argv)
{

  typedef pcl::PointXYZRGB PointT;

  // Load point cloud from file
  std::string fName = "";
  if(argv[1]) fName = std::string(argv[1]);
  else return 0;
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PCDReader reader;
  reader.read (fName, *cloud);

  // View Point Cloud
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Point Cloud from: "+fName));
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer->addPointCloud<PointT> (cloud, rgb, "Cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud");  
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (0.25);
  viewer->setCameraPosition(0, 0, -3, 0, -1, 0);

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::seconds (1));
  }
  
  return 0;
}