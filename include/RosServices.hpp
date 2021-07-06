#ifndef ROS_SERVICES_HPP
#define ROS_SERVICES_HPP

#include <iostream>
#include <cctype>
#include <chrono> 
#include <thread>

#include "ros/ros.h"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <curl/curl.h>

#include "human_vision_exchange/FindFaceVectorsFacenet.h"
#include "human_vision_exchange/FaceDescriptionVectorFacenet.h"

#include <API_Functions.hpp>

namespace RS
{
	bool computeFaceVectors(human_vision_exchange::FindFaceVectorsFacenet::Request  &req, human_vision_exchange::FindFaceVectorsFacenet::Response &res);
}

#endif
