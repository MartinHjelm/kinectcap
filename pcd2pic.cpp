// std
#include <string>
#include <iostream>

// boost 
#include <boost/program_options.hpp>

// Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZ PointT2;
typedef pcl::PointCloud<PointT> PC;
typedef pcl::PointCloud<PointT2> PC2;

  

int main(int argc, char ** argv)
{

    // Load point cloud from file
    std::string fName = "";
    if(argv[1]) 
    {
        fName = std::string(argv[1]);
        if(fName == "-h")
        {
            std::cout << "Usage: pcd2pic fileName xyz. xyz argument used if you want to remove color." << std::endl;   
            return -1;
        }
    }
    else 
    {
        std::cout << "Please specify a file to convert." << std::endl;
        std::cout << "Usage: pcd2pic fileName xyz. xyz argument used if you want to remove color." << std::endl;   
        return -1;
    }


    bool pcTypeRGB = true;

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

    cv::Mat pcImg = cv::Mat::zeros(480,640,CV_8UC3);
    double focalInv = 1000.0/525.0;


    if (pcTypeRGB)
    {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
        for(PC::const_iterator iter=cloud->begin(); iter!=cloud->end(); ++iter)
        {
            double xm = iter->x; double ym = iter->y; double zm = iter->z;
            int xPos = (1000.0 * xm / (zm * focalInv)) + 320;
            int yPos = (1000.0 * ym / (zm * focalInv)) + 240;

            pcImg.at<cv::Vec3b>(yPos,xPos)[0] = iter->b; //BGR 2 RGB
            pcImg.at<cv::Vec3b>(yPos,xPos)[1] = iter->g;
            pcImg.at<cv::Vec3b>(yPos,xPos)[2] = iter->r;
        }
    }
    else 
    {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud2,*cloud2, indices);        
        for(PC2::const_iterator iter=cloud2->begin(); iter!=cloud2->end(); ++iter)
        {
            double xm = iter->x; double ym = iter->y; double zm = iter->z;
            int xPos = (1000.0 * xm / (zm * focalInv)) + 320;
            int yPos = (1000.0 * ym / (zm * focalInv)) + 240;

            pcImg.at<cv::Vec3b>(yPos,xPos)[0] = 255; //BGR 2 RGB
            pcImg.at<cv::Vec3b>(yPos,xPos)[1] = 255;
            pcImg.at<cv::Vec3b>(yPos,xPos)[2] = 255;
        }
    }

    cv::imwrite("test.png", pcImg);

    return 1;
}    
