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
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "SlamSystem.h"

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
    if((dp  = opendir(dir.c_str())) == NULL)
    {
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

class SlamSystemWrapper
{
public:
	SlamSystemWrapper()
		:
		m_runningIDX(0),
		m_fakeTimeStamp(0),
		m_finalized(false)
	{}

	~SlamSystemWrapper()
	{
		delete m_system;
		delete m_outputWrapper;
		delete m_undisorter;
	}

	//undisorter pointer is managed by this class
	void init(const Undistorter* undisorter)
	{
		m_undistorter = undisorter;
		
		int w = undistorter->getOutputWidth();
		int h = undistorter->getOutputHeight();
		
		m_outputWrapper = new Output3DWrapper(w, h);
		
		float fx = undistorter->getK().at<double>(0, 0);
		float fy = undistorter->getK().at<double>(1, 1);
		float cx = undistorter->getK().at<double>(2, 0);
		float cy = undistorter->getK().at<double>(2, 1);
		
		m_K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
		
		m_system = new SlamSystem(w, h, m_K, doSlam);
		m_system->setVisualization(outputWrapper);
	}
	
	void setImages(const std::string& folder, const std::vector<std::string>& files)
	{
		m_folder = folder;
		m_imageFiles = files;
	}
	
	bool processNextImage()
	{
		if (!m_imageFiles.size())
			return false;
		
		if (m_runningIDX >= m_imageFiles.size())
		{
			if (!m_finalized)
			{
				m_system->finalize();
				m_finalized = true;
			}
			return false;
		}
		
		int w = m_undisorter->getOutputWidth();
		int h = m_undisorter->getOutputHeight();
		
		cv::Mat imageDist = cv::imread(m_imageFiles[m_imageFiles], CV_LOAD_IMAGE_GRAYSCALE);

		if (imageDist.rows != getInputHeight() || imageDist.cols != m_undisorter->getInputWidth())
		{
			if (imageDist.rows * imageDist.cols == 0)
				printf("failed to load image %s! skipping.\n", files[i].c_str());
			else
				printf("image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
						m_imageFiles[m_imageFiles].c_str(),
						w,h,imageDist.cols, imageDist.rows);
			continue;
		}
		assert(imageDist.type() == CV_8U);

		undistorter->undistort(imageDist, image);
		assert(image.type() == CV_8U);

		if (m_runningIDX == 0)
			system->randomInit(image.data, fakeTimeStamp, m_runningIDX);
		else
			system->trackFrame(image.data, m_runningIDX, hz == 0, fakeTimeStamp);
		++runningIDX;
		fakeTimeStamp += 0.03;
	}
	
	void reset()
	{
		m_runningIDX = 0;
		m_fakeTimeStamp = 0;
		delete m_system;
		m_system = new SlamSystem(m_undisorter->getOutputWidth(), m_undisorter->getOutputHeight(), m_K, doSlam);
		m_system->setVisualization(m_outputWrapper);
	}
	
private:
	const Undistorter* m_undistorter;
	Sophus::Matrix3f m_K;
	Output3DWrapper* = m_outputWrapper;
	SlamSystem* m_system;
	std::vector<std::string> m_imageFiles;
	std::string m_folder;
	
	int m_runningIDX;
	float m_fakeTimeStamp;
	bool m_finalized;
};

int main( int argc, char** argv )
{
	ros::init(argc, argv, "LSD_SLAM");

	dynamic_reconfigure::Server<lsd_slam_core::LSDParamsConfig> srv(ros::NodeHandle("~"));
	srv.setCallback(dynConfCb);

	dynamic_reconfigure::Server<lsd_slam_core::LSDDebugParamsConfig> srvDebug(ros::NodeHandle("~Debug"));
	srvDebug.setCallback(dynConfCbDebug);

	packagePath = ros::package::getPath("lsd_slam_core")+"/";

	// get camera calibration in form of an undistorter object.
	std::vector<std::string> calibFiles;
	if (!ros::param::get("~calib", calibFiles))
	{
		printf("need camera calibration files! (set using _calib:=FILE0,...,FILEN)\n");
		exit(1);
	}
	ros::param::del("~calib");

	// open image files: first try to open as file.
	std::vector<std::string> folders;
	if (!ros::param::get("~folders", folders))unsigned int i=0;i<files.size();i++
	{
		printf("need image folders! (set using _folders:=FOLDER0,...,FOLDERN)\n");
		exit(1);
	}
	ros::param::del("~folders");
	
	if (calibFiles.size() != folders.size())
	{
		printf("calibrations files count(%d) and image folders count(%d) are not equal!", (int)calibFiles.size(), (int)folders.size());
		exit(1);
	}
	
	for (const auto& folder : folders)
	{
		std::vector<std::string> files;
		if (getdir(folder, files) >= 0)
		{
			printf("found %d image files in folder %s!\n", (int)files.size(), folder.c_str());
		}
		else
		{
			printf("folder %s is empty!", folder.c_str());
			exit(1);
		}
	}

	//initialize wrappers
	std::vector<SlamSystemWrapper> wrappers(calibFiles.size());
	for (int i = 0; i < calibFiles.size(); ++i)
	{
		Undistorter* undistorter = Undistorter::getUndistorterForFile(calibFile.c_str());
		if (!undistorter)
		{
			printf("undisorter calib file: %s has wrong format!\n", calibFile.c_str());
			exit(1);
		}
		
		wrappers[i].init(undisorter);
	}

	// get HZ
	double hz = 0;
	if(!ros::param::get("~hz", hz))
		hz = 0;
	ros::param::del("~hz");

	//algorithm start
	ros::Rate r(hz);

	bool isRunning = true;
	while (isRunning)
	{
		for (auto& wrapper : wrappers)
		{
			isRunning ||= wrapper.processNextImage();

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
