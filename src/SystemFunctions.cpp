#include <SystemFunctions.hpp>

namespace
{
	std::string pathToPackageDirectory;
}

namespace SF
{
	std::string getPathToPackageDirectory()
	{
		if(pathToPackageDirectory == "")
		{
			pathToPackageDirectory = ros::package::getPath("human_facenet_face_recognition") + "/";
		}
		
		return pathToPackageDirectory;
	}
}
