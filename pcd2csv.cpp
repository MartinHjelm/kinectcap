// std
#include <string>
#include <iostream>

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
typedef pcl::PointCloud<PointT> PC;
const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", "\n");

int writeMat2File(const Eigen::MatrixXf &matrix, const std::string &fileName);


int
writeMat2File(const Eigen::MatrixXf &matrix, const std::string &fileName)
{
  std::ofstream out( fileName.c_str() );

  if (out.is_open())
    out << matrix.format(CSVFormat);
  else
    return 0;

  out.close();
  return 1;
}


int main(int argc, char ** argv)
{
    std::string fNamePCD = "";
    std::string fNamePIC = "";

    // Load point cloud from file
    std::string fName = "";
    if(argc > 2) 
    {
        fNamePCD = std::string(argv[1]);
        fNamePIC = std::string(argv[2]);
        if(fNamePCD == "-h")
        {
            std::cout << "Usage: pcd2csv pcd-FileName pic-FileName" << std::endl;   
            return -1;
        }
    }
    else 
    {
        std::cout << "Please specify a pcd file and its corresponding image to convert to csv." << std::endl;
        std::cout << "Usage: pcd2csv pcd-FileName pic-FileName" << std::endl;   
        return -1;
    }


    // READ PCD AND IM FILES
    cv::Mat im; 
    im = cv::imread(fNamePIC);

    pcl::PCDReader reader;
    PC::Ptr cloud (new PC);
    reader.read(fNamePCD, *cloud); 

    int i = 640*480;
    Eigen::MatrixXf X(307200,8);

    for(int iCol=0; iCol!=480; iCol++)
    {
        for(int iRow=0; iRow!=640; iRow++)
        {   
            int idx = iRow*480 + iCol;
            // img x-y-pos
            X(idx,0) = iRow; 
            X(idx,1) = iCol;
            // img
            X(idx,2) = im.at<cv::Vec3b>(iRow,iCol)[0];
            X(idx,3) = im.at<cv::Vec3b>(iRow,iCol)[1];
            X(idx,4) = im.at<cv::Vec3b>(iRow,iCol)[2];
            // img
            // PointT p_valid = cloudIn->at(idx);
            X(idx,5) = cloud->at(idx).x;
            X(idx,6) = cloud->at(idx).y;
            X(idx,7) = cloud->at(idx).z;
        }
    }

    writeMat2File(X, "X.txt");

    return 1;
}    






