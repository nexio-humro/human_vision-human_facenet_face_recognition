#ifndef SYSTEM_FUNCTIONS_HPP
#define SYSTEM_FUNCTIONS_HPP

#include <stdio.h>
#include <unistd.h>
#include <iostream>

#include "ros/ros.h"
#include "ros/package.h"

namespace SF
{
	std::string getPathToPackageDirectory();
}

#endif
