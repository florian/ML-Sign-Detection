#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
   cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
   // cv::imshow("view",cv_bridge::toCvShare(msg,"bgr8")->image);   
   // cv::imshow("view",cv_ptr->image);
  }

  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  
  int windows_n_rows = 200;
  int windows_n_cols = 200;
  int step = 100; 

  for(int row = 0; row <= cv_ptr->image.rows - windows_n_rows; row += step){
     for(int col = 0; col <= cv_ptr->image.cols - windows_n_cols; col += step){
     //there could be feature evaluator over windows

     //resulting window
     cv::Rect windows(col, row, windows_n_rows, windows_n_cols);
     cv::rectangle(cv_ptr->image,windows, Scalar(255),1,8,0);
     
     cv::imshow("view",cv_ptr->image);
     }


  } 

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image/compressed",1,imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
