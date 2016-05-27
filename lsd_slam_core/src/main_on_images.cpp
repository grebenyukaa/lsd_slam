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
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	SlamSystemWrapper()
		:
		m_runningIDX(0),
		m_fakeTimeStamp(0),
		m_finalized(false),
		m_hz(0)
	{}

	~SlamSystemWrapper()
	{
		delete m_system;
		delete m_outputWrapper;
		delete m_undistorter;
	}

	//undisorter pointer is managed by this class
	void init(const std::string& wnd_name, const Undistorter* undistorter, const double hz)
	{
		m_hz = hz;
		m_undistorter = undistorter;
		m_wnd_name = wnd_name;
		
		int w = undistorter->getOutputWidth();
		int h = undistorter->getOutputHeight();
		
		m_outputWrapper = new ROSOutput3DWrapper(w, h);
		
		float fx = undistorter->getK().at<double>(0, 0);
		float fy = undistorter->getK().at<double>(1, 1);
		float cx = undistorter->getK().at<double>(2, 0);
		float cy = undistorter->getK().at<double>(2, 1);
		
		m_K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
		
		m_system = new SlamSystem(w, h, m_K, wnd_name, doSlam);
		m_system->setVisualization(m_outputWrapper);
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
		
		if (m_runningIDX >= (int)m_imageFiles.size())
		{
			if (!m_finalized)
			{
				m_system->finalize();
				m_finalized = true;
			}
			return false;
		}
		
		int w = m_undistorter->getOutputWidth();
		int h = m_undistorter->getOutputHeight();
		
		cv::Mat imageDist = cv::imread(m_imageFiles[m_runningIDX], CV_LOAD_IMAGE_GRAYSCALE);

		if (imageDist.rows != m_undistorter->getInputHeight() || imageDist.cols != m_undistorter->getInputWidth())
		{
			if (imageDist.rows * imageDist.cols == 0)
				printf("failed to load image %s! skipping.\n", m_imageFiles[m_runningIDX].c_str());
			else
				printf("image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
						m_imageFiles[m_runningIDX].c_str(),
						w,h,imageDist.cols, imageDist.rows);
			return true;
		}
		assert(imageDist.type() == CV_8U);

		cv::Mat image = cv::Mat(h, w, CV_8U);
		m_undistorter->undistort(imageDist, image);
		assert(image.type() == CV_8U);

		if (m_runningIDX == 0)
			m_system->randomInit(image.data, m_fakeTimeStamp, m_runningIDX);
		else
			m_system->trackFrame(image.data, m_runningIDX, m_hz == 0, m_fakeTimeStamp);
		++m_runningIDX;
		m_fakeTimeStamp += 0.03;
		
		return true;
	}
	
	void reset()
	{
		m_runningIDX = 0;
		m_fakeTimeStamp = 0;
		m_hz = 0;
		delete m_system;
		m_system = new SlamSystem(m_undistorter->getOutputWidth(), m_undistorter->getOutputHeight(), m_K, m_wnd_name, doSlam);
		m_system->setVisualization(m_outputWrapper);
	}
	
private:
	const Undistorter* m_undistorter;
	Sophus::Matrix3f m_K;
	Output3DWrapper* m_outputWrapper;
	SlamSystem* m_system;
	std::vector<std::string> m_imageFiles;
	std::string m_folder;
	
	int m_runningIDX;
	float m_fakeTimeStamp;
	bool m_finalized;
	double m_hz;
	std::string m_wnd_name;
};

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
	
	// get HZ
	double hz = 0;
	if(!ros::param::get("~hz", hz))
		hz = 0;
	ros::param::del("~hz");

	//initialize wrappers
	std::vector<SlamSystemWrapper> wrappers(calibFiles.size());
	for (size_t i = 0; i < calibFiles.size(); ++i)
	{
		Undistorter* undistorter = Undistorter::getUndistorterForFile(calibFiles[i].c_str());
		if (!undistorter)
		{
			printf("undisorter calib file: %s has wrong format!\n", calibFiles[i].c_str());
			exit(1);
		}
		
		std::ostringstream oss;
		oss << "Vehicle " << i;
		wrappers[i].init(oss.str(), undistorter, hz);
		
		std::vector<std::string> files;
		if (getdir(folders[i], files) >= 0)
		{
			printf("found %d image files in folder %s!\n", (int)files.size(), folders[i].c_str());
			wrappers[i].setImages(folders[i], files);
		}
		else
		{
			printf("folder %s is empty!", folders[i].c_str());
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
