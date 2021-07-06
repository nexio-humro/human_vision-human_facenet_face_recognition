#include "Paths.hpp"

namespace
{
	std::string pathToOutputDirectory = "output";
}

namespace P
{
	void setPathToOutputDirectory(std::string path)
	{
		pathToOutputDirectory = path;
	}
	
	const std::string getPathToOutputDirectory()
	{
		return pathToOutputDirectory;
	}
}
