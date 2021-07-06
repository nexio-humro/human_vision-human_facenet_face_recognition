#include <RosServices.hpp>

namespace RS
{
	bool computeFaceVectors(human_vision_exchange::FindFaceVectorsFacenet::Request  &req, human_vision_exchange::FindFaceVectorsFacenet::Response &res)
	{	
		std::cout<<"RS::computeFaceVectors(): req.faces.size() = "<<req.faces.size()<<std::endl;
		
		FILE *responseFile = NULL;	
		std::string responseFilePath = P::getPathToOutputDirectory() + "response";
		responseFile = fopen(responseFilePath.c_str(), "w+");
		
		std::vector<std::string> pathsToImages;
		pathsToImages = AF::saveImagesAndReturnPaths(req, "image");
		
		std::this_thread::sleep_for (std::chrono::milliseconds(500));
	
		AF::prepareAndPerformCurlPost(responseFile, pathsToImages);
	
		// set pointer to begin of file
		int seekResponse;
		seekResponse = fseek(responseFile, 0, SEEK_SET);
		
		std::vector<std::vector<double>> faceVectors;
//		faceVectors.resize(pathsToImages.size());
		
//		AF::fillupVectors(faceVectors, responseFile);
		faceVectors = AF::readFaceVectorsFromResponseFile(responseFile);
		AF::fillupFindFaceVectorsFacenetResponse(faceVectors, res);
		
		// display vectors
		if(false)
		{
			for(size_t i = 0; i < faceVectors.size(); i++)
			{
				std::cout<<i<<" vector:"<<std::endl;
				
				for(size_t j = 0; j < faceVectors.at(i).size(); j++)
				{
					std::cout<<j<<" = "<<faceVectors.at(i).at(j)<<std::endl;
				}
			}
		}

		fclose(responseFile);
		
		AF::cleanupTemporaryImages(pathsToImages);
		
		return true;
	}
}
