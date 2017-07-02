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

image_window win;

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
	
	//test for using dlib
	//convert for dlib
/*	frontal_face_detector detector = get_frontal_face_detector();
	shape_predictor pose_model;
	deserialize("shape_predictor_68_face_landmarks.dat") >> pose_model;
	
	dlib::cv_image<dlib::bgr_pixel> cimg(cv_ptr->image);	

	std::vector<rectangle> faces = detector(cimg);
	std::vector<full_object_detection> shapes;

	for(unsigned long i = 0 ; i < faces.size(); ++i)
          shapes.push_back(pose_model(cimg, faces[i]));	
	
	win.clear_overlay();
	win.set_image(cimg);
	win.add_overlay(render_face_detections(shapes));
*/
	//candidate selecting
	//if(process(cv_ptr->image)){
	//imagePub.publish(cv_ptr->toImageMsg()); 
	//}

	//traffic sign detection
	//load the svm file
	typedef scan_fhog_pyramid<pyramid_down<6>> image_scanner_type;
//	image_scanner_type scanner;

//	scanner.set_detection_window_size(40,40);
	
	object_detector<image_scanner_type> detector;
	deserialize("own_new.svm") >> detector;
	
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
	
	}	
};

int main(int argc, char** argv)
{
  ros::init(argc,argv, "preprocess");
  preprocessor p;
  ros::spin();
  return 0;
}
