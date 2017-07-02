#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

#include<dlib/image_io.h>
#include<dlib/opencv.h>
#include<dlib/image_processing.h>
#include<dlib/gui_widgets.h>
#include<dlib/image_transforms.h>

#include<sys/time.h>

using namespace dlib;
using namespace std;

image_window win;

class detector
{
  private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber imageSub;
    image_transport::Publisher imagePub;
  public:
    detector():it(nh){	
	imageSub = it.subscribe("app/camera/rgb/image_raw",1,&detector::imageCallBack,this);
	imagePub = it.advertise("detction",1);
	cv::namedWindow("VIEW");
    }

    ~detector(){
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
	
	//traffic sign detection
	//load the svm file
	typedef scan_fhog_pyramid<pyramid_down<6>> image_scanner_type;

	object_detector<image_scanner_type> detector;
	deserialize("own_new.svm") >> detector;
	
	cv::Mat imgForDlib = cv_ptr->image.clone();

	array2d<rgb_pixel> img;
	assign_image(img,cv_image<bgr_pixel>(imgForDlib));
	
	struct timeval tp;
	gettimeofday(&tp, NULL);
	long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
	
	pyramid_up(img);
	pyramid_up(img);
	std::vector<dlib::rectangle> signs = detector(img);

	gettimeofday(&tp, NULL);
	long int ms2 = tp.tv_sec * 1000 + tp.tv_usec / 1000;
	
	std::vector<dlib::rectangle> scaled_signs(signs.size());
	
	if(signs.size() != 0){
	 
	  for(int i = 0; i < signs.size(); i++){

	    scaled_signs[i] = dlib::rectangle(signs[i].left()/4.00625,signs[i].top()/4.009375,signs[i].right()/4.00625,signs[i].bottom()/4.009375);
	  }
	
	}	
	//time counting
	cout<<(ms2 - ms)<<endl;

	dlib::cv_image<dlib::bgr_pixel> cimg(imgForDlib);
	win.clear_overlay();
	win.set_image(cimg);
	win.add_overlay(scaled_signs,rgb_pixel(0,255,0));
	
	}	
};

int main(int argc, char** argv)
{
  ros::init(argc,argv, "detection");
  detector d;
  ros::spin();
  return 0;
}
