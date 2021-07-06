#include "ros/ros.h"
#include "ros/package.h"

#include <curl/curl.h>

#include <RosServices.hpp>
#include <SystemFunctions.hpp>
#include <Paths.hpp>

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "human_facenet_face_recognition");
	ros::NodeHandle node("/human_facenet_face_recognition/");
	
	// set path to image directory
	std::string pathToOutputDirectory;
	pathToOutputDirectory = SF::getPathToPackageDirectory() + "output/";
	P::setPathToOutputDirectory(pathToOutputDirectory);
	
	// init curl
	curl_global_init(CURL_GLOBAL_ALL);

	ros::ServiceServer faceVectorService = node.advertiseService("findFaceVectors", RS::computeFaceVectors);
		
	ros::spin();
    return 0;
}
