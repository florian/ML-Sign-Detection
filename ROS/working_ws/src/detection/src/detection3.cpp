#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

#include<dlib/image_io.h>
#include<dlib/opencv.h>
#include<dlib/image_processing.h>
#include<dlib/image_processing/frontal_face_detector.h>
#include<dlib/image_processing/render_face_detections.h>
#include<dlib/gui_widgets.h>
#include<dlib/image_transforms.h>

#include<sys/time.h>

using namespace dlib;
using namespace std;

//image_window win;

class preprocessor
{
  private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber imageSub;
    image_transport::Publisher imagePub;
    float imgScale;
  public:
    preprocessor():it(nh){	
	imageSub = it.subscribe("image_reading",1,&preprocessor::imageCallBack,this);
	imagePub = it.advertise("preprocessed",1);
//////	cv::namedWindow("VIEW");
//	cv::startWindowThread();
    }

   //// ~preprocessor(){
   //// cv::destroyWindow("VIEW");
    //}

    void imageCallBack(const sensor_msgs::ImageConstPtr &msg){
	//convert to cv type for further processing
	cout<<"imageCallBack"<<endl;
	cv_bridge::CvImagePtr cv_ptr;
	try{
	  cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	}
	catch(cv_bridge::Exception &e){
	  ROS_ERROR("cv_bridge exception: %s",e.what());
	  return;
	}
	
	typedef scan_fhog_pyramid<pyramid_down<6>> image_scanner_type;

	object_detector<image_scanner_type> detector;
	deserialize("/root/traffic_sign_ws/src/detection/src/own19.svm") >> detector;
	
	struct timeval tp;
	gettimeofday(&tp, NULL);
	long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
	//candidate finding
	cout<<"candidate selecting!"<<endl;	
	cv::Mat hsv_image;
	cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);
		
	cv::Mat lower_mask;
	cv::Mat upper_mask;
	
	cv::inRange(hsv_image, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), lower_mask);
	cv::inRange(hsv_image, cv::Scalar(160,50,50),cv::Scalar(180, 255, 255), upper_mask);
	
	cv::Mat combination;
	cv::addWeighted(lower_mask, 1.0, upper_mask, 1.0, 0.0, combination);
	cout<<"show image"<<endl;

	

//	cv::imshow("VIEW",combination);
	
	//finding window part
//	cv::Mat grayImage;
//	cv::Mat hsv_channels[3];
//	cv::split(combination, hsv_channels);
	
//	cv::cvtColor(combination, grayImage, cv::COLOR_HSV2GRAY);	
//	cv::Mat imgForCa;
//	cv::threshold(grayImage, imgForCa, 127, 255, 0);	

	//cv::imshow("VIEW",combination);
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(combination, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	cout<<contours.size()<<endl;
	int c = 50;	
	for(size_t i = 0 ; i < contours.size(); i++) {	
		int area = cv::contourArea(contours[i]);
		if(area > 100){
		  cv::Rect rect = cv::boundingRect(contours[i]);
		  float ratio = ((float)rect.width)/((float)rect.height);
		  if(ratio<= 1.5 && ratio >= 0.5){
			  cv::Mat candi = cv::Mat(cv_ptr->image, cv::Rect(rect.x-c, rect.y-c, rect.width+ 2*c, rect.height+ 2*c));
			  
			array2d<rgb_pixel> img;
			assign_image(img,cv_image<bgr_pixel>(candi));
			cout<<"pyramid_up()"<<endl;	
			pyramid_up(img);
			pyramid_up(img);

			long int height;
			long int width;
			height = img.nc();
			width = img.nr();

			

			//calculateing factor
			double row_factor = (double)height/(double)candi.cols;
			double col_factor = (double)width/(double)candi.rows;
			cout<<"signs_detector()"<<endl;
			std::vector<dlib::rectangle> signs = detector(img);
			cout<<"sings_detector() done"<<endl;
			std::vector<dlib::rectangle> scaled_signs(signs.size());

			if(signs.size() != 0){
			 for(int i = 0; i < signs.size(); i++){
			  scaled_signs[i] = dlib::rectangle(signs[i].left()/4.00625,signs[i].top()/4.009375,signs[i].right()/4.00625,signs[i].bottom()/4.009375);
			cout<<"coordinates"<<endl;
			cout<<scaled_signs[i].left()<<endl;
			cout<<scaled_signs[i].top()<<endl;
			cout<<scaled_signs[i].right()<<endl;
			cout<<scaled_signs[i].bottom()<<endl;
			
			//win.clear_overlay();
			//win.set_image(img);
			//win.add_overlay(signs,rgb_pixel(0,255,0));
			} 
			}
		}
            }

	}	
	gettimeofday(&tp, NULL);
	long int ms1 = tp.tv_sec * 1000 + tp.tv_usec / 1000;
	cout<<"performance="<<ms1 - ms<<endl;
/*	
	cv::Mat imgForDlib = cv_ptr->image.clone();
//	dlib::cv_image<dlib::bgr_pixel> cimg(imgForDlib);

//	cv_image(cimg)	
	array2d<rgb_pixel> img;
	assign_image(img,cv_image<bgr_pixel>(cv_ptr->image));
	
	//factor for rescale img after pyramid_up
	double row_factor;
	double col_factor;
	
	//starting-time counting
	struct timeval tp;
	gettimeofday(&tp, NULL);
	long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
	
	pyramid_up(img);
	pyramid_up(img);

	long int height;
	long int width;
	height = img.nc();
	width = img.nr();

	cout<<height<<width<<endl;
	
	std::vector<dlib::rectangle> signs = detector(img);

	cout<<imgForDlib.cols<<endl;
	cout<<imgForDlib.rows<<endl;

	
	//calculateing factor
	row_factor = height/imgForDlib.cols;
	col_factor = width/imgForDlib.rows;

	cout<<row_factor<<endl;
	cout<<col_factor<<endl;

	//ending-time counting
	gettimeofday(&tp, NULL);
	long int ms2 = tp.tv_sec * 1000 + tp.tv_usec / 1000;
	
	std::vector<dlib::rectangle> scaled_signs(signs.size());

	if(signs.size() != 0){
	 for(int i = 0; i < signs.size(); i++){
	  scaled_signs[i] = dlib::rectangle(signs[i].left()/4.00625,signs[i].top()/4.009375,signs[i].right()/4.00625,signs[i].bottom()/4.009375);

	} 
	}
//	image_window hogwin(draw_fhog(detector),"show");	
	cout<<(ms2 - ms)<<endl;

	dlib::cv_image<dlib::bgr_pixel> cimg(imgForDlib);
	win.clear_overlay();
	win.set_image(cimg);
	win.add_overlay(scaled_signs,rgb_pixel(0,255,0));
	*/
	}	
};

int main(int argc, char** argv)
{
  cout<<"main called"<<endl;
  ros::init(argc,argv, "preprocess");
  cout<<"init ed"<<endl;
  preprocessor p;
  ros::spin();
  cout<<"after spin"<<endl;
  return 0;
}
