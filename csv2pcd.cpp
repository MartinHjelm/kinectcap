// std
#include <string>
#include <iostream>
#include <sstream>

// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

// boost
#include <boost/algorithm/string.hpp>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PC;

const float bad_point = std::numeric_limits<float>::quiet_NaN();

int main(int argc, char ** argv)
{
    std::string fNameCSV = "";
    std::string token = ";";

    // Check arguments
    if(argc > 1) 
    {
        fNameCSV = std::string(argv[1]);
        if(argc > 2) 
            token = std::string(argv[2]);

        if(fNameCSV == "-h")
        {
            std::cout << "Usage: csv2pcd csv-FileName token" << std::endl;   
            return -1;
        }
    }
    else 
    {
        std::cout << "Please specify a csv file." << std::endl;
        std::cout << "Usage: csv2pcd csv-FileName token" << std::endl;   
        return -1;
    }


    PC::Ptr cloud(new PC);

    // Read csv file to pcd
    std::ifstream file(fNameCSV);
    std::string values;
    while ( file.good() )
    {
        std::getline(file,values);
        std::vector<std::string> elems;
        boost::split(elems, values, boost::is_any_of(token));

        if(elems.size()!=6)
            break;

        // for(int i=0; i!=6; i++)
        //     std::cout << elems[i] << ", ";
        // std::cout << std::endl;


        PointT pt;
        pt.r = std::atoi(elems[0].c_str());
        pt.g = std::atoi(elems[1].c_str());
        pt.b = std::atoi(elems[2].c_str());
        pt.a = 255;
        // uint8_t r = std::atoi(elems[0].c_str());
        // uint8_t g = std::atoi(elems[1].c_str());
        // uint8_t b = std::atoi(elems[2].c_str());
        // int32_t rgb = (r << 16) | (g << 8) | b; 
        // pt.rgba = *(float *)(&rgb); // makes the point red
        
        pt.x = std::atof(elems[3].c_str());
        pt.y = std::atof(elems[4].c_str());
        pt.z = std::atof(elems[5].c_str());

        // std::cout << pt.x << ", " << pt.y << ", "<< pt.z << ", " << std::endl;

        // std::cout << pt << std::endl;

        cloud->push_back(pt);
    }

    cloud->width = 640;
    cloud->height = 480;

    int fileNameLen = fNameCSV.length();
    pcl::io::savePCDFileBinary(fNameCSV.substr(0,fileNameLen-4)+".pcd", *cloud);

    return 1;
}    






