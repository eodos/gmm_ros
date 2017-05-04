#include "ros/ros.h"

#include "aplink.hpp"

#include "BufferedAsyncSerial.h"
#include <boost/chrono/chrono.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gmm_aplink");
	ros::NodeHandle n;

	return 0;
}