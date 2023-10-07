#include <librealsense2/rs.hpp> //Include Intel RealSense Cross Platform API
#include <iostream>

int main(){
	//Create a Pipeline - this serves as a top-level API for streaming and processing frames
	rs2::error* e;
	rs2::pipeline pipe;

	//Configure and start the pipeline
/*	pipe.start();

	//Block program until frames arrive
	rs2::frameset frames = pipe.wait_for_frames();

	//Try to get a frame of a depth image
	rs2::depth_frame depth = frames.get_depth_frame();

	//Get the dept frame's dimensions
	float width = depth.get_width();
	float height = depth.get_height();

	// Query the distance from the camera to the object in the center of the image
	float dist_to_center = depth.get_distance(width / 2, height / 2);

	// Print the distance
	std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";
*/	return 0;
}
