#include <API_Functions.hpp>

namespace AF
{
	std::vector<std::string> saveImagesAndReturnPaths(human_vision_exchange::FindFaceVectorsFacenet::Request& req, std::string name)
	{
		std::vector<std::string> pathsToImages;
		
		for(size_t i = 0; i < req.faces.size(); i++)
		{
//			std::cout<<"image "<<i<<" encoding = "<<req.faces[i].encoding<<std::endl;
			
			cv_bridge::CvImagePtr cv_ptr;
			
			// check encoding
			if(req.faces[i].encoding == "rgb8")
			{
				cv_ptr = cv_bridge::toCvCopy(req.faces[i], sensor_msgs::image_encodings::RGB8);
			}
			else
			{
				cv_ptr = cv_bridge::toCvCopy(req.faces[i], sensor_msgs::image_encodings::BGR8);
			}
			
			cv::Mat faceImage = cv_ptr->image;
			
			std::string actualPath = P::getPathToOutputDirectory() + name + std::to_string(i) + ".png";
			pathsToImages.push_back(actualPath);
			
			cv::imwrite(actualPath, faceImage);
		}
		
		return pathsToImages;
	}
	
	double findNextDouble(FILE* file)
	{
//		static int wordCounter = 0;
		
		double result;
		
		char number[100];
		bool numberStarted = false;
		int c;
		int counter =0;
		
		while ((c = getc(file)) != EOF)
		{
//			std::cout<<"findNextDouble(): c = "<<(char)c<<std::endl;
			if(numberStarted == false)
			{			
				if( (isdigit(c) != 0) || (c == '-') )
				{
					numberStarted = true;
					number[counter++] = (char)c;
				}
			}
			else
			{			
				if( (isdigit(c) != 0) || (c == '-') || (c == '.') || (c == 'e') )
				{
					number[counter++] = (char)c;
				}
				else
				{
					break;
				}
			}
		}
		number[counter] = '\0';
		
		result = atof(number);
		
//		std::cout<<wordCounter++<<": number = "<<number<<", result = "<<result<<std::endl;
		
		return result;
	}
	
	void prepareAndPerformCurlPost(FILE* responseFile, std::vector<std::string>& paths)
	{
		std::cout<<"prepareAndPerformCurlPost()"<<std::endl;
		
		// get a curl handle 
		CURL *curl;
		curl = curl_easy_init();
		
		if(curl) 
		{
			curl_easy_setopt(curl, CURLOPT_URL, "http://127.0.0.1:5000/get_vector");
			
			curl_mime *multipart = curl_mime_init(curl);
			
			curl_mimepart *part;
			for(size_t i = 0; i < paths.size(); i++)
			{
				std::cout<<"mimepart : "<<i<<std::endl;
				part = curl_mime_addpart(multipart);
				curl_mime_name(part, "files[]");
				curl_mime_filedata(part, paths.at(i).c_str());
			}  
			
			// write response to file
			curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void*)responseFile);
		
			// set the form info
			curl_easy_setopt(curl, CURLOPT_MIMEPOST, multipart);

			// send post request
			curl_easy_perform(curl); 

			// free the post data again 
			curl_mime_free(multipart); 
		}
		curl_global_cleanup();
	}
	
	void fillupVectors(std::vector<std::vector<double>>& faceVectors, FILE* responseFile)
	{
		for(size_t i = 0; i < faceVectors.size(); i++)
		{
			std::vector<double> faceVector;
			
			std::cout << std::setprecision(22);
			
			for(size_t j = 0; j < 512; j++)
			{
				double doubleVariable;
				doubleVariable = AF::findNextDouble(responseFile);
				
				faceVector.push_back(doubleVariable);
			}
			
			faceVectors.at(i) = faceVector;
		} 
	}
	
	void cleanupTemporaryImages(std::vector<std::string>& paths)
	{
		for(size_t i = 0; i < paths.size(); i++)
		{
			remove(paths.at(i).c_str());
		}
	}
	
	void fillupFindFaceVectorsFacenetResponse(std::vector<std::vector<double>>& faceVectors, human_vision_exchange::FindFaceVectorsFacenet::Response& res)
	{
		res.faceVectors.faces.resize(faceVectors.size());
		for(size_t i = 0; i < faceVectors.size(); i++)
		{
			human_vision_exchange::FaceDescriptionFacenet fd;
			for(size_t j = 0; j < faceVectors.at(i).size(); j++)
			{
				fd.points[j] = faceVectors.at(i).at(j);
			}
			res.faceVectors.faces[i] = fd;
		}
	}
	
	std::vector<double> readFaceVector(FILE* file)
	{
		std::vector<double> faceVector;
		faceVector.resize(FACEVECTORELEMENTSAMOUNT);
		size_t numberAmount = 0;
			
		std::cout << std::setprecision(22);
		
		for(size_t j = 0; j < FACEVECTORELEMENTSAMOUNT; j++)
		{
			double doubleVariable;
			doubleVariable = AF::findNextDouble(file);
			
//			std::cout<<j<<" number = "<<doubleVariable<<std::endl;
			
			// check break sign
			char c = AF::checkLastChar(file);
			
			if(c == ',')
			{
				faceVector.at(j) = doubleVariable;
				numberAmount++;
			}
			else
			{
				if(c == ']')
				{
					faceVector.at(j) = doubleVariable;
					numberAmount++;
					break;
				}
			}
		}
		
		if(numberAmount != FACEVECTORELEMENTSAMOUNT)
		{
			std::cout<<"readFaceVector(): wrong amount of numbers = "<<numberAmount<<std::endl;
		}
		
		return faceVector;
	}
	
	char checkLastChar(FILE* file)
	{
		char result;
		
		AF::returnToPreviousChar(file);
		result = getc(file);
		
		return result;
	}
	
	void returnToPreviousChar(FILE* file)
	{
		long actualPosition = ftell(file);
		
		if(actualPosition > 0)
		{
			fseek(file, (actualPosition - 1), SEEK_SET);
		}
	}
	
	std::vector<std::vector<double>> readFaceVectorsFromOnePicture(FILE* file)
	{
		std::vector<std::vector<double>> result;
		
		// get face vector until to closing square bracket meet
		while( checkIfPreviousAndActualCharsAreSquareClosingBrackets(file) == false)
		{
			std::vector<double> actualFaceVector;
			actualFaceVector = readFaceVector(file);
			result.push_back(actualFaceVector);
		}	
		
		return result;
	}
	
	bool checkIfPreviousAndActualCharsAreSquareClosingBrackets(FILE* file)
	{
		bool result = false;
		
		char previous;
		previous = AF::checkLastChar(file);
		
		if(previous == ']')
		{
			char actual = getc(file);
			
			if(actual == ']')
			{
				result = true;
			}
		}
		
		return result;
	}
	
	std::vector<std::vector<double>> readFaceVectorsFromResponseFile(FILE* file)
	{
		std::vector<std::vector<double>> result;
		
		// find beginning of image face vectors in file, should be preceded by two left square brackets [[
		while(AF::findTwoLeftSquareBrackets(file) == true)
		{
			std::vector<std::vector<double>> imageFaceVectors;
			imageFaceVectors = AF::readFaceVectorsFromOnePicture(file);
			
//			std::cout<<"readFaceVectorsFromResponseFile(): before insertion: result.size() = "<<result.size()<<", imageFaceVectors.size() = "<<imageFaceVectors.size()<<std::endl;
			
			// insert vectors
			result.insert(result.end(), imageFaceVectors.begin(), imageFaceVectors.end());
			
//			std::cout<<"readFaceVectorsFromResponseFile(): after insertion: result.size() = "<<result.size()<<std::endl;
		}
		
		return result;
	}
	
	bool findTwoLeftSquareBrackets(FILE* file)
	{
		int previousChar = 0;
		int actualChar = 0;
		bool twoBracketsFound = false;
		
		while( ((actualChar = getc(file)) != EOF) )
		{
			if( ((char)previousChar == '[') && ((char)actualChar == '[') )
			{
				twoBracketsFound = true;
				break;
			}
			
			previousChar = actualChar;
		}
		
		return twoBracketsFound;
	}
}
