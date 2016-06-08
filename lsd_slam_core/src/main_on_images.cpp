/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "LiveSLAMWrapper.h"

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "SlamSystem.h"
#include "ImageSLAMWrapper.h"

#include "GlobalMapping/KeyFrameGraph.h"

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

#include "IOWrapper/ROS/ROSOutput3DWrapper.h"
#include "IOWrapper/ROS/rosReconfigure.h"

#include "util/Undistorter.h"
#include <ros/package.h>

#include "opencv2/opencv.hpp"

std::string &ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
        return s;
}
std::string &rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
        return s;
}
std::string &trim(std::string &s) {
        return ltrim(rtrim(s));
}
int getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
	printf("Opendir\n");
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        printf("  failed\n");
		return -1;
    }

    while ((dirp = readdir(dp)) != NULL) {
    	std::string name = std::string(dirp->d_name);

    	if(name != "." && name != "..")
    		files.push_back(name);
    }
    closedir(dp);


    std::sort(files.begin(), files.end());

    if(dir.at( dir.length() - 1 ) != '/') dir = dir+"/";
	for(unsigned int i=0;i<files.size();i++)
	{
		if(files[i].at(0) != '/')
			files[i] = dir + files[i];
	}

    return files.size();
}

int getFile (std::string source, std::vector<std::string> &files)
{
	std::ifstream f(source.c_str());

	if(f.good() && f.is_open())
	{
		while(!f.eof())
		{
			std::string l;
			std::getline(f,l);

			l = trim(l);

			if(l == "" || l[0] == '#')
				continue;

			files.push_back(l);
		}

		f.close();

		size_t sp = source.find_last_of('/');
		std::string prefix;
		if(sp == std::string::npos)
			prefix = "";
		else
			prefix = source.substr(0,sp);

		for(unsigned int i=0;i<files.size();i++)
		{
			if(files[i].at(0) != '/')
				files[i] = prefix + "/" + files[i];
		}

		return (int)files.size();
	}
	else
	{
		f.close();
		return -1;
	}

}

using namespace lsd_slam;

void splitParams(const std::string& src, const std::string& sep, std::vector<std::string>& dst)
{
	boost::split(dst, src, boost::is_any_of(sep));
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "LSD_SLAM");

	dynamic_reconfigure::Server<lsd_slam_core::LSDParamsConfig> srv(ros::NodeHandle("~"));
	srv.setCallback(dynConfCb);

	dynamic_reconfigure::Server<lsd_slam_core::LSDDebugParamsConfig> srvDebug(ros::NodeHandle("~Debug"));
	srvDebug.setCallback(dynConfCbDebug);

	packagePath = ros::package::getPath("lsd_slam_core")+"/";

	// get camera calibration in form of an undistorter object.
	std::string calibFilesStr;
	if (!ros::param::get("~calib", calibFilesStr))
	{
		printf("need camera calibration files! (set using _calib:=FILE0,...,FILEN)\n");
		exit(1);
	}
	ros::param::del("~calib");
	std::vector<std::string> calibFiles;
	splitParams(calibFilesStr, ", ", calibFiles);

	// open image files: first try to open as file.
	std::string foldersStr;
	if (!ros::param::get("~folders", foldersStr))
	{
		printf("need image folders! (set using _folders:=FOLDER0,...,FOLDERN)\n");
		exit(1);
	}
	ros::param::del("~folders");
	std::vector<std::string> folders;
	splitParams(foldersStr, ", ", folders);
	
	if (calibFiles.size() != folders.size())
	{
		printf("calibrations files count(%d) and image folders count(%d) are not equal!", (int)calibFiles.size(), (int)folders.size());
		exit(1);
	}
	
	srand(42);
	
	// get HZ
	int hz = 0;
	if(!ros::param::get("~hz", hz))
		hz = 0;
	ros::param::del("~hz");
	printf("HZ = %d\n", hz);

	//initialize wrappers
	int frameIDOffset = 0;
	std::vector<SlamSystemWrapper> wrappers(calibFiles.size());
	for (size_t i = 0; i < calibFiles.size(); ++i)
	{
		Undistorter* undistorter = Undistorter::getUndistorterForFile(calibFiles[i].c_str());
		if (!undistorter)
		{
			printf("undisorter calib file: %s has wrong format!\n", calibFiles[i].c_str());
			exit(1);
		}
		
		std::vector<std::string> files;
		if (getdir(folders[i], files) >= 0)
		{
			printf("found %d image files in folder %s!\n", (int)files.size(), folders[i].c_str());
			wrappers[i].init(i/* + 1*/, frameIDOffset, undistorter, hz);
			frameIDOffset += (int)files.size();
			wrappers[i].setImages(folders[i], files);
		}
		else
		{
			printf("folder %s is empty!\n", folders[i].c_str());
		}
	}

	//algorithm start
	ros::Rate r(hz);

	bool isRunning = true;
	while (isRunning)
	{
		isRunning = false;
		for (auto& wrapper : wrappers)
		{
			isRunning |= wrapper.processNextImage();

			if (hz != 0)
				r.sleep();

			if (fullResetRequested)
				wrapper.reset();

			ros::spinOnce();

			if (!ros::ok())
				break;
		}
	}

	return 0;
}
