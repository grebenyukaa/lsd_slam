#pragma once

#include <QGLViewer/qglviewer.h>
#include <tbb/concurrent_unordered_map.h>

#include "lsd_slam_viewer/keyframeMsg.h"
#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "PointCloudViewer.h"

class MultiAgentPointCloudViewer
{
public:
    MultiAgentPointCloudViewer(int nAgents);
	
	void init();
	
	void addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr msg);
	void addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg);
   
private:
	tbb::concurrent_unordered_map<int, PointCloudViewer*> m_viewers;
};