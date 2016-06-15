#pragma once

#include <QGLViewer/qglviewer.h>
#include <tbb/concurrent_unordered_map.h>

#include "lsd_slam_viewer/keyframeMsg.h"
#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "PointCloudViewer.h"

#include <map>

class MultiAgentPointCloudViewer
{
public:
    MultiAgentPointCloudViewer(int nAgents);
	
	void init();
	
	void addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr msg);
	void addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg);
	inline PointCloudViewer* getViewer(const int i) const { return m_viewers.at(i); }
	inline int getVwrCount() const { return (int)m_viewers.size(); }
   
private:
	bool findEqualAndAdjustAlignTransform(int refId, int refAgentId);
	void countLogNormResidual(int agentId, int frameId, int refAgentId, int refFrameId, float& avg, float& max);

	tbb::concurrent_unordered_map<int, PointCloudViewer*> m_viewers;
	std::map<int, std::map<int, std::map<int, std::list<int>>>> m_equal_kfs;
};