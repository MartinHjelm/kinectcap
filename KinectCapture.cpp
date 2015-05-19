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

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PC;


class KinectCapture
{
  void imageCallback(const PC::ConstPtr& cloud);
  void videoCallback(const boost::shared_ptr<openni_wrapper::Image>& img);
  void setImageNameCounter ();

  boost::shared_ptr<cv::VideoWriter> videoWriter;
  bool capImage, capVideo, saveImg;
  int save_n, skipImageFreq;
  std::string imgCaptureDir;

public:  
  KinectCapture ();
  void run ();
  void setCaptureType ( std::string s );
  void setSkipFreq ( const int &freq );
  void setImageCaptureDir ( std::string s );
  std::string getImageCaptureDir ();
};

  
KinectCapture::KinectCapture() : 
capImage(true), 
capVideo(false), 
saveImg(false), 
save_n(0), 
skipImageFreq(1), 
imgCaptureDir("kcap_images")
{}


void 
KinectCapture::setCaptureType ( std::string s )
{
  if (s == "image") 
    capImage = true;
  else if (s == "video") 
    capVideo = true;    
}


void KinectCapture::setSkipFreq(const int &freq) { skipImageFreq = freq; }

std::string KinectCapture::getImageCaptureDir() { return imgCaptureDir; }



void 
KinectCapture::setImageCaptureDir ( std::string s ) 
{ 
    imgCaptureDir = s; 
   // Create image directory if it does not exist
    boost::filesystem::path dir( imgCaptureDir );
    if ( boost::filesystem::create_directories(dir) ) printf("Saving captures to directory %s\n",s.c_str());  
    // Set counter to the +1 number of images in the directory 
    setImageNameCounter ();  
}


void
KinectCapture::setImageNameCounter()
{
  boost::filesystem::directory_iterator it( getImageCaptureDir() );
  for (; it != boost::filesystem::directory_iterator(); ++it )
  {
    if ( boost::filesystem::is_regular_file( it->status() ) && it->path().extension() == ".png" )
      save_n++;
  }
}



void KinectCapture::imageCallback(const PC::ConstPtr& cloud)
{
  
  // Convert point cloud to an image of the point cloud.
  cv::Mat pcImg(480,640,CV_8UC3);
  Eigen::Vector3i rgbVals;  
  for(int iter_x = 0; iter_x != 640; iter_x++)
  {
    for(int iter_y = 0; iter_y != 480; iter_y++)
    {
      // PointT p = cloud->at(iter_x,iter_y);
      rgbVals = (cloud->at(iter_x,iter_y)).getRGBVector3i();
      pcImg.at<cv::Vec3b>(iter_y,iter_x)[0] = rgbVals[2]; //BGR 2 RGB
      pcImg.at<cv::Vec3b>(iter_y,iter_x)[1] = rgbVals[1];
      pcImg.at<cv::Vec3b>(iter_y,iter_x)[2] = rgbVals[0];
    }
  }

  // Show converted img
  cv::imshow("Result", pcImg);
  cv::waitKey(10);

  // If we are saving
  if ( saveImg )
  {
    std::string fName = std::string("kcap_images/image-") + boost::lexical_cast<std::string>(save_n);
    std::string fEnding = ".png";
    std::cout << "Saved image: " << fName+fEnding ;
    cv::imwrite(fName+fEnding, pcImg);

      // PC rgbpc;
      // rgbpc.width = pc->width;
      // rgbpc.height = pc->height;
      // rgbpc.is_dense = pc->is_dense;
      // for (pcl::PointCloud<pcl::PointXYZRGBA>::const_iterator ii = pc->begin(); ii != pc->end(); ++ii)
      // {
      //   pcl::PointXYZRGB p;
      //   p.x = ii->x;
      //   p.y = ii->y;
      //   p.z = ii->z;
      //   p.r = ii->r;
      //   p.g = ii->g;
      //   p.b = ii->b;
      //   //p.rgb = ii->rgba;
      //   rgbpc.push_back(p);
    
    fEnding = std::string(".pcd");
    pcl::io::savePCDFileBinary(fName+fEnding, *cloud);
    std::cout << " and point cloud: " << fName+fEnding;
    save_n++;
    saveImg = false;
  }
}




void 
KinectCapture::videoCallback(const boost::shared_ptr<openni_wrapper::Image>& img)
{
  if (save_n % skipImageFreq == 0)
  {
    cv::Mat imgFrame = cv::Mat(img->getHeight(),img->getWidth(),CV_8UC3);
    img->fillRGB(imgFrame.cols,imgFrame.rows,imgFrame.data,imgFrame.step);
    cv::cvtColor(imgFrame,imgFrame,CV_RGB2BGR);
    std::string fileName = std::string("kcap_images/image-") + boost::lexical_cast<std::string>(save_n) + std::string(".jpg");
    cv::imwrite(fileName, imgFrame);
  }
  
  save_n++;
}




/*
class gg : pcl::Grabber
{
public:
  gg()
  {
    typedef std::map<std::string, boost::signals2::signal_base*> m_t;
    for (m_t::const_iterator i = signals_.begin(); i != signals_.end(); ++i)
      {
        std::cout << i->first << std::endl;
      }
  }
  };*/

void
KinectCapture::run()
{
   
  cv::Mat pcImg(480,640,CV_8UC3);
  if (capImage)
  {
    cv::imshow("Result", pcImg);
    cv::waitKey(10);
  }
  // create a new grabber for OpenNI devices
  pcl::Grabber* interface = new pcl::OpenNIGrabber();

  // std::cout << interface->getFramesPerSecond() << std::endl;
  boost::signals2::connection c;
  if (capImage)
  {
    boost::function<void (const PC::ConstPtr&)> f = boost::bind(&KinectCapture::imageCallback, this, _1);      
    // connect callback function for desired signal. In this case its a point cloud with color values
    c = interface->registerCallback (f);
  }
  else
  {
    // videoWriter.reset(new cv::VideoWriter("video.avi", CV_FOURCC('m', 'j', 'p', 'g'), 25, cvSize(640, 480), 1));
    boost::function<pcl::OpenNIGrabber::sig_cb_openni_image> f = boost::bind(&KinectCapture::videoCallback, this, _1);
    // connect callback function for desired signal. In this case its a point cloud with color values
    c = interface->registerCallback (f);
  }
  // start receiving point clouds
  interface->start ();
  std::cout << "Press enter key to save current image. q + enter key to exit." << std::endl;


  char s;
  while (true)
  {
    s = std::cin.get();
    //std::cin.ignore();
    // std::cin >> s;
    if ( s == 'q' )
      break;
    saveImg = true;
  }
  std::cout << std::endl << "Exiting..." << std::endl;
  // stop the grabber
  interface->stop ();
}





int main(int argc, char ** argv)
{
  try 
  {    

    // Create command line options
    namespace po = boost::program_options; 
    po::options_description desc("Options"); 
    desc.add_options() 
      (",c", po::value<std::string>(), "Capture type: image(default) or video")
      (",f", po::value<int>(), "If video, frame rate. Default is 5 per second.")
      ("dir,d", po::value<std::string>(), "Directory to save the capture to. Default is: kcap_images")
      ("help,h", "Print help messages");
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).run(),vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

    // Create capture object and do configure
    KinectCapture kc;

    // Image or image sequence capture default is image 
    if(vm.count("c")) 
      kc.setCaptureType(vm["c"].as<std::string>());

    // If video set frame rate skipping
    if(vm.count("f")) 
      kc.setSkipFreq(vm["f"].as<int>());  

    // If set directory for storing capture 
    if(vm.count("dir"))    
      kc.setImageCaptureDir(vm["dir"].as<std::string>());


    // All configured lets run, run, run!!
    kc.run();



    return 1;
  }
  catch ( std::exception &e ) 
  {
    std::cerr << "Exception caught: ";
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch ( ... ) {
    std::cerr << "Caught unknown exception." << std::endl;
    return EXIT_FAILURE;
  }
  
}
