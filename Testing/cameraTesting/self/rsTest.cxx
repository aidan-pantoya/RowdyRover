#include <librealsense2/rs.hpp> //Include Intel RealSense Cross Platform API
#include <iostream>

int main(){
	rs2::context ctx;
	auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
	if (list.size() == 0) 
    		throw std::runtime_error("No device detected. Is it plugged in?");
	rs2::device dev = list.front();
}
