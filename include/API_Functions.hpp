#ifndef API_FUNCTIONS_HPP
#define API_FUNCTIONS_HPP

#include <iostream>
#include <cctype>

#include "ros/ros.h"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <curl/curl.h>

#include "human_vision_exchange/FindFaceVectorsFacenet.h"
#include "human_vision_exchange/FaceDescriptionVectorFacenet.h"
#include "human_vision_exchange/FaceDescriptionFacenet.h"

#include <Paths.hpp>

#define FACEVECTORELEMENTSAMOUNT 512

namespace AF
{
	std::vector<std::string> saveImagesAndReturnPaths(human_vision_exchange::FindFaceVectorsFacenet::Request& req, std::string name = "image");
	double findNextDouble(FILE* file);
	void prepareAndPerformCurlPost(FILE* responseFile, std::vector<std::string>& paths);
	void fillupVectors(std::vector<std::vector<double>>& faceVectors, FILE* responseFile);
	void cleanupTemporaryImages(std::vector<std::string>& paths);
	void fillupFindFaceVectorsFacenetResponse(std::vector<std::vector<double>>& faceVectors, human_vision_exchange::FindFaceVectorsFacenet::Response& res);
	
	std::vector<double> readFaceVector(FILE* file);
	char checkLastChar(FILE* file);
	void returnToPreviousChar(FILE* file);
	std::vector<std::vector<double>> readFaceVectorsFromOnePicture(FILE* file);
	bool checkIfPreviousAndActualCharsAreSquareClosingBrackets(FILE* file);
	std::vector<std::vector<double>> readFaceVectorsFromResponseFile(FILE* file);
	bool findTwoLeftSquareBrackets(FILE* file); 
}

#endif
