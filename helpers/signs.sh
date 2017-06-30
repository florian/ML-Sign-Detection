alias record="rosbag record /app/camera/rgb/camera_raw/compressed /tf /rosout"
alias extractx="rosrun image_view extract_images _sec_per_frame:=1 image:=/app/camera/rgb/image_raw _image_transport:=compressed && mkdir images && mv frame*.jpg images"

function extract() {
	rm -rf signs_images
	rosrun image_view extract_images _sec_per_frame:=$1 image:=/app/camera/rgb/image_raw _image_transport:=compressed
	mkdir signs_images
	mv frame*.jpg signs_images
}
