#include <catch.hpp>

#include <SystemFunctions.hpp>
#include <API_Functions.hpp>

TEST_CASE("readFaceVector()", "[API_Functions]")
{	
	std::string pathToFile;
	pathToFile = SF::getPathToPackageDirectory() + "examples/response";
	
	FILE *responseFile = NULL;	
	responseFile = fopen(pathToFile.c_str(), "r");
	
	if(responseFile == NULL)
	{
		std::cout<<"error with file opening"<<std::endl;
	}
	
	fseek(responseFile, 0, SEEK_SET);
	
	// display file
	if(false)
	{
		char c;
		while ((c = getc(responseFile)) != EOF)
		{
			std::cout<<(char)c;
		}
		std::cout<<std::endl;
	}
		
	std::vector<double> receivedFaceVector;
	
	size_t vectorsAmount = 8;
	for(size_t i = 0; i < vectorsAmount; i++)
	{
		receivedFaceVector = AF::readFaceVector(responseFile);
			
		REQUIRE(receivedFaceVector.at(receivedFaceVector.size()-1) != 0);
		REQUIRE(AF::checkLastChar(responseFile) == ']');
	}
	
	receivedFaceVector = AF::readFaceVector(responseFile);
		
	REQUIRE(receivedFaceVector.at(receivedFaceVector.size()-1) == 0);
	REQUIRE(receivedFaceVector.at(0) == 0);
	REQUIRE(AF::checkLastChar(responseFile) == ']');
	
	fclose(responseFile);
}

TEST_CASE("readFaceVectorsFromOnePicture()", "[API_Functions]")
{	
	std::string pathToFile;
	pathToFile = SF::getPathToPackageDirectory() + "examples/response";
	
	FILE *responseFile = NULL;	
	responseFile = fopen(pathToFile.c_str(), "r");
	
	if(responseFile == NULL)
	{
		std::cout<<"error with file opening"<<std::endl;
	}
	
	fseek(responseFile, 0, SEEK_SET);
	
	std::vector<std::vector<double>> receivedFaceVectors;
	
	size_t imagesAmount = 8;
	for(size_t i = 0; i < imagesAmount; i++)
	{
		std::cout<<i<<" image"<<std::endl;
		receivedFaceVectors = AF::readFaceVectorsFromOnePicture(responseFile);
			
		REQUIRE(receivedFaceVectors.size() == 1);
	}
	
	receivedFaceVectors = AF::readFaceVectorsFromOnePicture(responseFile);
		
	REQUIRE(receivedFaceVectors.size() == 0);
	
	fclose(responseFile);
}

TEST_CASE("readFaceVectorsFromResponseFile()", "[API_Functions]")
{	
	std::string pathToFile;
	pathToFile = SF::getPathToPackageDirectory() + "examples/response";
	
	FILE *responseFile = NULL;	
	responseFile = fopen(pathToFile.c_str(), "r");
	
	if(responseFile == NULL)
	{
		std::cout<<"error with file opening"<<std::endl;
	}
	
	fseek(responseFile, 0, SEEK_SET);
	
	std::vector<std::vector<double>> receivedFaceVectors;
	
	receivedFaceVectors = AF::readFaceVectorsFromResponseFile(responseFile);		
	REQUIRE(receivedFaceVectors.size() == 8);
	
	receivedFaceVectors = AF::readFaceVectorsFromResponseFile(responseFile);
	REQUIRE(receivedFaceVectors.size() == 0);
	
	fclose(responseFile);
}
