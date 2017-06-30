#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

class preprocessor
{
  private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber imageSub;
    image_transport::Publisher imagePub;

  public:
    preprocessor():it(nh){
	imageSub = it.subscribe("camera",1,&preprocessor::imageCallBack,this);
	imagePub = it.advertise("preprocessed",1);
	cv::namedWindow("VIEW");
    }

    ~preprocessor(){
    cv::destroyWindow("VIEW");
    }

    void imageCallBack(const sensor_msgs::ImageConstPtr &msg){
	//convert to cv type for further processing
	cv_bridge::CvImagePtr cv_ptr;
	try{
	  cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	}
	catch(cv_bridge::Exception &e){
	  ROS_ERROR("cv_bridge exception: %s",e.what());
	  return;
	}

	//candidate selecting
	//if(process(cv_ptr->image)){
	  imagePub.publish(cv_ptr->toImageMsg()); 
	//}
    }	
	//function for image processing unfinished
	/*
	bool process(cv::Mat img){
	  if (img.){
	    return 1;
          return 0;
	  }  
	}
	*/
};

int main(int argc, char** argv)
{
  ros::init(argc,argv, "preprocess");
  preprocessor p;
  ros::spin();
  return 0;
}
