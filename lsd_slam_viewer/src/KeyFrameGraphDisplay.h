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



#ifndef KEYFRAMEGRAPHDISPLAY_H_
#define KEYFRAMEGRAPHDISPLAY_H_


#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "lsd_slam_viewer/keyframeMsg.h"
#include "boost/thread.hpp"
#include <list>

#include "KeyFrameDisplay.h"

class PointCloudViewer;

struct GraphConstraint
{
	int from;
	int to;
	float err;
};

struct GraphConstraintPt
{
	int fromAgentId;
	int from;
	int toAgentId;
	int to;
	float err;
	
	GraphConstraintPt()
		:
		fromAgentId(-1),
		from(-1),
		toAgentId(-1),
		to(-1)
	{}
};

struct GraphFramePose
{
	int id;
	float camToWorld[7];
};


class KeyFrameGraphDisplay {
public:
	KeyFrameGraphDisplay(int agentId, PointCloudViewer* vwr);
	virtual ~KeyFrameGraphDisplay();

	void draw();

	void addMsg(lsd_slam_viewer::keyframeMsgConstPtr msg);
	void addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg);
	
	void addConstraint(int fromAgentId, int fromId, int toAgentId, int toId, float err = 0.0);
	void addGraph(int pivot, int other_pivot, const KeyFrameGraphDisplay* graph);
	int findEqualKF(const Sophus::Sim3f& alignTransform, const KeyFrameDisplay& kf, double& dist, double& angleCos, float ClosenessTH = 1.0f, float KFDistWeight = 4.0f);

	inline const std::map<int, KeyFrameDisplay>& getKeyFramesByID() const { return keyframesByID; }
	inline const std::list<GraphConstraintPt>& getConstraints() const { return constraints; }
	inline void setCamToWorld(int frameId, const Sophus::Sim3f& ctw)
	{ 
		dataMutex.lock();
		memcpy(keyframesByID[frameId].camToWorld.data(), ctw.data(), 7*sizeof(float));
		dataMutex.unlock();
	};
	inline const Sophus::Sim3f& getCamToWorld(int frameId) { return keyframesByID[frameId].camToWorld; };

	bool flushPointcloud;
	bool printNumbers;
	
	inline int getAgentId() const { return agentId; }
	
private:
	int agentId;
	PointCloudViewer* vwr;
	std::map<int, KeyFrameDisplay> keyframesByID;
	//std::list<KeyFrameDisplayPtr> keyframes;
	std::list<GraphConstraintPt> constraints;
	
	boost::mutex dataMutex;
};

#endif /* KEYFRAMEGRAPHDISPLAY_H_ */
