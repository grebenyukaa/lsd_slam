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
* along with dvo. If not, see <http://www.gnu.org/licenses/>.
*/

#include <qapplication.h>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>

#include "ros/ros.h"
#include "settings.h"
#include "PointCloudViewer.h"

#include <dynamic_reconfigure/server.h>
#include "lsd_slam_viewer/LSDSLAMViewerParamsConfig.h"

#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "lsd_slam_viewer/keyframeMsg.h"
#include "MultiAgentPointCloudViewer.h"

#include "rosbag/bag.h"
#include "rosbag/query.h"
#include "rosbag/view.h"

MultiAgentPointCloudViewer* mvwr = nullptr;

void dynConfCb(lsd_slam_viewer::LSDSLAMViewerParamsConfig &config, uint32_t level)
{

	pointTesselation = config.pointTesselation;
	lineTesselation = config.lineTesselation;

	keepInMemory = config.keepInMemory;
	showKFCameras = config.showKFCameras;
	showKFPointclouds = config.showKFPointclouds;
	showConstraints = config.showConstraints;
	showCurrentCamera = config.showCurrentCamera;
	showCurrentPointcloud = config.showCurrentPointcloud;


	scaledDepthVarTH = exp10( config.scaledDepthVarTH );
	absDepthVarTH = exp10( config.absDepthVarTH );
	minNearSupport = config.minNearSupport;
	sparsifyFactor = config.sparsifyFactor;
	cutFirstNKf = config.cutFirstNKf;

	saveAllVideo = config.saveAllVideo;

}

void frameCb(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	printf("  frame cb, agentId: %d\n", (int)msg->agentId);	
	if(msg->time > lastFrameTime) return;
	mvwr->addFrameMsg(msg);
}

void graphCb(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	printf("  graph cb, agentId: %d\n", (int)msg->agentId);	
	mvwr->addGraphMsg(msg);
}

void rosThreadLoop( int argc, char** argv )
{
	printf("Started ROS thread\n");

	//glutInit(&argc, argv);

	ros::init(argc, argv, "viewer");
	ROS_INFO("lsd_slam_viewer started");

	dynamic_reconfigure::Server<lsd_slam_viewer::LSDSLAMViewerParamsConfig> srv;
	srv.setCallback(dynConfCb);


	ros::NodeHandle nh;

	ros::Subscriber liveFrames_sub = nh.subscribe(nh.resolveName("lsd_slam/liveframes"),1, frameCb);
	ros::Subscriber keyFrames_sub = nh.subscribe(nh.resolveName("lsd_slam/keyframes"),20, frameCb);
	ros::Subscriber graph_sub       = nh.subscribe(nh.resolveName("lsd_slam/graph"),10, graphCb);

	ros::spin();

	ros::shutdown();

	printf("Exiting ROS thread\n");

	exit(1);
}


void rosFileLoop( int argc, char** argv )
{
	ros::init(argc, argv, "viewer");
	dynamic_reconfigure::Server<lsd_slam_viewer::LSDSLAMViewerParamsConfig> srv;
	srv.setCallback(dynConfCb);

	rosbag::Bag bag;
	bag.open(argv[2], rosbag::bagmode::Read);

	std::vector<std::string> topics;
	topics.push_back(std::string("/lsd_slam/liveframes"));
	topics.push_back(std::string("/lsd_slam/keyframes"));
	topics.push_back(std::string("/lsd_slam/graph"));

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	 //for(rosbag::MessageInstance const m = view.begin(); m < view.end(); ++m)
	 BOOST_FOREACH(rosbag::MessageInstance const m, view)
	 {

		 if(m.getTopic() == "/lsd_slam/liveframes" || m.getTopic() == "/lsd_slam/keyframes")
			 frameCb(m.instantiate<lsd_slam_viewer::keyframeMsg>());


		 if(m.getTopic() == "/lsd_slam/graph")
			 graphCb(m.instantiate<lsd_slam_viewer::keyframeGraphMsg>());
	 }

	ros::spin();

	ros::shutdown();

	printf("Exiting ROS thread\n");

	exit(1);
}

int main(int argc, char** argv)
{
	if (argc < 2)
	{
		printf("Usage: %s NAgents\n", argv[0]);
		exit(1);
	}
	int nAgents = atoi(argv[1]);
	
	printf("Started QApplication thread\n");
	// Read command lines arguments.
	QApplication application(argc,argv);
	
	mvwr = new MultiAgentPointCloudViewer(nAgents);
	mvwr->init();

	boost::thread rosThread;

	if (argc > 2)
	{
		rosThread = boost::thread(rosFileLoop, argc, argv);
	}
	else
	{
		// start ROS thread
		rosThread = boost::thread(rosThreadLoop, argc, argv);
	}
	
	application.exec();

	printf("Shutting down... \n");
	ros::shutdown();
	rosThread.join();
	
	delete mvwr;
	
	printf("Done. \n");
}
