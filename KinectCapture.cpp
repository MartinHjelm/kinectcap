// std
#include <string>

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
  void setSkipFreq ( std::string s );
  void setImageCaptureDir ( std::string s );
  std::string getImageCaptureDir ();
};

  
KinectCapture::KinectCapture() : 
capImage(false), 
capVideo(false), 
saveImg(false), 
save_n(0), 
skipImageFreq(1), 
imgCaptureDir("kcap_images")
{}


void KinectCapture::setCaptureType ( std::string s )
{
  if (s.length() > 1 && s == "--image") 
    capImage = true;
  else if (s.length() > 1 && s == "--video") 
    capVideo = true;    
  else
    capImage = true;
}


void KinectCapture::setSkipFreq (std::string s) { skipImageFreq = std::atoi(s.c_str()); }

std::string KinectCapture::getImageCaptureDir() { return imgCaptureDir; }

void KinectCapture::setImageCaptureDir ( std::string s ) 
{ 
    imgCaptureDir = s; 
   // Create image directory if it does not exist
    boost::filesystem::path dir( imgCaptureDir );
    if ( boost::filesystem::create_directories(dir) ) printf("Mkdir: kcap_images\n");  
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


// Main Fun
int main(int argc, char ** argv)
{
  try 
  {    
    KinectCapture kc;

    // Check args 
    if(argc > 1 && std::string(argv[1])=="--help")
    { 
      printf("Kinectcap usage: kinectcap --[image|video] --[savedir]\n");
      return 0;
    }

    // Image or image sequence capture
    if(argc > 1) 
      kc.setCaptureType( std::string(argv[1]) );
    else 
      kc.setCaptureType( "" );

    
    // If video set frame rate skipping
    if(argv[2]) kc.setSkipFreq(std::string(argv[2]));  

    // Set directory for storing images 
    kc.setImageCaptureDir("kcap_images");

    kc.run();
    return 0;
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
