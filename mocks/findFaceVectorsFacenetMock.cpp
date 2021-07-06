#include <vector>
#include <string>

#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/Image.h"

#include "human_vision_exchange/FindFaceVectorsFacenet.h"
#include "human_vision_exchange/FaceDescriptionVectorFacenet.h"

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ros/package.h"

std::string getPathToImage(int argc, char **argv);
cv_bridge::CvImage readImage(std::string imageName);
std::vector< std::vector<double>> callFindFaceVector(ros::ServiceClient& client, std::vector<cv_bridge::CvImage>& faces);
void displayDescriptions(std::vector<std::vector<double> > descriptionsVector);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "find_face_vector_facenet_mock");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<human_vision_exchange::FindFaceVectorsFacenet>("/human_facenet_face_recognition/findFaceVectors");

	// read first image
    cv_bridge::CvImage firstImage;
    firstImage = readImage("face0.png");
    
	// read second image
    cv_bridge::CvImage secondImage;
    secondImage = readImage("face1.png");
    
	// read third image
    cv_bridge::CvImage thirdImage;
    thirdImage = readImage("face52_0.png");
	
	// send brooklyn images
	std::vector<cv_bridge::CvImage> faces;
	faces.push_back(firstImage);
	faces.push_back(secondImage);
	faces.push_back(thirdImage);
	std::vector< std::vector<double>> brooklynVectors;
	brooklynVectors = callFindFaceVector(client, faces);
	
	// send brooklyn and random images
/*	std::vector<cv_bridge::CvImage> faces2;
	faces2.push_back(thirdImage);
	faces2.push_back(secondImage);
	std::vector< std::vector<double>> randomVectors;
	randomVectors = callFindFaceVector(client, faces2);  */
	
	/*
	
	brooklynVectors.push_back(randomVectors.at(0));
	brooklynVectors.push_back(randomVectors.at(1)); */
	
	displayDescriptions(brooklynVectors); 

	return 0;
}

std::string getPathToImage(std::string imageName)
{
	std::string pathToImage;
	pathToImage = ros::package::getPath("human_facenet_face_recognition") + "/images/" + imageName;
	
	return pathToImage;
}

cv_bridge::CvImage readImage(std::string imageName)
{
	std::string pathToImage;
	pathToImage = getPathToImage(imageName);
	
#if CV_MAJOR_VERSION < 4
    cv::Mat image = cv::imread(pathToImage.c_str(), CV_LOAD_IMAGE_COLOR); 
#else
    cv::Mat image = cv::imread(pathToImage.c_str(), cv::IMREAD_COLOR); 
#endif 

    cv::waitKey(30);
    cv_bridge::CvImage cvImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image);	
    
    return cvImage;
}

std::vector< std::vector<double>> callFindFaceVector(ros::ServiceClient& client, std::vector<cv_bridge::CvImage>& faces)
{
	std::vector< std::vector<double>> faceDescriptions;
	
	human_vision_exchange::FindFaceVectorsFacenet callMsg;
	callMsg.request.faces.resize(faces.size());
	
	for(size_t i = 0; i < faces.size(); i++)
	{
		faces.at(i).toImageMsg(callMsg.request.faces[i]);
	}
	
	client.call(callMsg);
	
	for(size_t i = 0; i < callMsg.response.faceVectors.faces.size(); i++)
	{
		std::vector<double> description;
		
		for(size_t j = 0; j < callMsg.response.faceVectors.faces[i].points.size(); j++)
		{
			description.push_back(callMsg.response.faceVectors.faces[i].points[j]);
		}
		
		faceDescriptions.push_back(description);
	}
	
	return faceDescriptions;
}

void displayDescriptions(std::vector<std::vector<double> > descriptionsVector)
{
	if(descriptionsVector.size() > 0)
	{
		for(size_t i = 0; i < descriptionsVector.at(0).size(); i++)
		{
			std::cout<<i<<":\t"<<std::setprecision(2);
			for(size_t j = 0; j < descriptionsVector.size(); j++)
			{
				std::cout<<descriptionsVector.at(j).at(i)<<"\t";
			}
			std::cout<<std::endl;
		}
	}
}
