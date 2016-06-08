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


#include "KeyFrameGraphDisplay.h"
#include "KeyFrameDisplay.h"
#include "settings.h"
#include <sstream>
#include <fstream>

#include "ros/package.h"

KeyFrameGraphDisplay::KeyFrameGraphDisplay()
{
	flushPointcloud = false;
	printNumbers = false;
}

KeyFrameGraphDisplay::~KeyFrameGraphDisplay()
{}


void KeyFrameGraphDisplay::draw()
{
	dataMutex.lock();
	numRefreshedAlready = 0;

	// draw keyframes
	float color[3] = {0,0,1};
	for (auto frame_kv : keyframesByID)
	{
		if(showKFCameras)
			frame_kv.second->drawCam(lineTesselation, color);

		if((showKFPointclouds && (int)frame_kv.first > cutFirstNKf) || frame_kv.first == (int)keyframesByID.size() - 1)
			frame_kv.second->drawPC(pointTesselation, 1);
	}

	if(flushPointcloud)
	{
		printf("Flushing Pointcloud to %s!\n", (ros::package::getPath("lsd_slam_viewer")+"/pc_tmp.ply").c_str());
		std::ofstream f((ros::package::getPath("lsd_slam_viewer")+"/pc_tmp.ply").c_str());
		int numpts = 0;
		for (auto frame_kv : keyframesByID)
		{
			if((int)frame_kv.first > cutFirstNKf)
				numpts += frame_kv.second->flushPC(&f);
		}
		f.flush();
		f.close();

		std::ofstream f2((ros::package::getPath("lsd_slam_viewer")+"/pc.ply").c_str());
		f2 << std::string("ply\n");
		f2 << std::string("format binary_little_endian 1.0\n");
		f2 << std::string("element vertex ") << numpts << std::string("\n");
		f2 << std::string("property float x\n");
		f2 << std::string("property float y\n");
		f2 << std::string("property float z\n");
		f2 << std::string("property float intensity\n");
		f2 << std::string("end_header\n");

		std::ifstream f3((ros::package::getPath("lsd_slam_viewer")+"/pc_tmp.ply").c_str());
		while(!f3.eof()) f2.put(f3.get());

		f2.close();
		f3.close();

		system(("rm "+ros::package::getPath("lsd_slam_viewer")+"/pc_tmp.ply").c_str());
		flushPointcloud = false;
		printf("Done Flushing Pointcloud with %d points!\n", numpts);
	}

	if(printNumbers)
	{
		int totalPoint = 0;
		int visPoints = 0;
		for (auto frame_kv : keyframesByID)
		{
			totalPoint += frame_kv.second->totalPoints;
			visPoints += frame_kv.second->displayedPoints;
		}

		printf("Have %d points, %d keyframes, %d constraints. Displaying %d points.\n",
				totalPoint, (int)keyframesByID.size(), (int)constraints.size(), visPoints);
		printNumbers = false;
	}

	if(showConstraints)
	{
		// draw constraints
		glLineWidth(lineTesselation);
		glBegin(GL_LINES);
		for (auto constraint : constraints)
		{
			if(constraint.from == 0 || constraint.to == 0)
				continue;

			double colorScalar = std::max(0.0, std::min(1.0, constraint.err / 0.05));
			glColor3f(colorScalar, 1 - colorScalar, 0);

			Sophus::Vector3f t = constraint.from->camToWorld.translation();
			glVertex3f((GLfloat)t[0], (GLfloat)t[1], (GLfloat)t[2]);

			t = constraint.to->camToWorld.translation();
			glVertex3f((GLfloat)t[0], (GLfloat)t[1], (GLfloat)t[2]);
		}
		glEnd();
	}

	dataMutex.unlock();
}

int KeyFrameGraphDisplay::findEqualKF(const KeyFrameDisplayPtr& kf, float ClosenessTH, float KFDistWeight)
{
	float fowX, fowY;
	fowX = 2 * atanf((float)((kf->getWidth() / kf->getFx() / 2.0f)));
	fowY = 2 * atanf((float)((kf->getHeight() / kf->getFy() / 2.0f)));
	
	//from solver
	float distanceTH = ClosenessTH * 15 / (KFDistWeight*KFDistWeight);
	float angleTH = 1.0 - 0.25 * ClosenessTH;
	bool checkBothScales = true;
		
	// basically the maximal angle-difference in viewing direction is angleTH*(average FoV).
	// e.g. if the FoV is 130°, then it is angleTH*130°.
	float cosAngleTH = cosf(angleTH*0.5f*(fowX + fowY));

	Eigen::Vector3f pos = kf->camToWorld.translation();
	Eigen::Vector3f viewingDir = kf->camToWorld.rotationMatrix().rightCols<1>();

	float iDepth = kf->getPointDenseStruct()->idepth;

	float distFacReciprocal = 1;
	if(checkBothScales)
		distFacReciprocal = iDepth / kf->camToWorld.scale();

	// for each frame, calculate the rough score, consisting of pose, scale and angle overlap.
	int idKF = -1;
	dataMutex.lock();
	for (const auto& kf_kv : keyframesByID)
	{
		const KeyFrameDisplayPtr& curKF = kf_kv.second;
		Eigen::Vector3f otherPos = curKF->camToWorld.translation();
		float curIDepth = curKF->getPointDenseStruct()->idepth;

		// get distance between the frames, scaled to fit the potential reference frame.
		float distFac = curIDepth / curKF->camToWorld.scale();
		if (checkBothScales && distFacReciprocal < distFac) distFac = distFacReciprocal;
		Eigen::Vector3f dist = (pos - otherPos) * distFac;
		float dNorm2 = dist.dot(dist);
		if (dNorm2 > distanceTH) continue;

		Eigen::Vector3f otherViewingDir = curKF->camToWorld.rotationMatrix().rightCols<1>();
		float dirDotProd = otherViewingDir.dot(viewingDir);
		if (dirDotProd < cosAngleTH) continue;

		idKF = kf_kv.first;
		break;
	}
	dataMutex.unlock();

	return idKF;
}

void KeyFrameGraphDisplay::addMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	dataMutex.lock();
	if(keyframesByID.count(msg->id) == 0)
	{
		keyframesByID[msg->id] = std::make_shared<KeyFrameDisplay>();
		//keyframes.push_back(disp);

	//	printf("added new KF, now there are %d!\n", (int)keyframes.size());
	}

	keyframesByID[msg->id]->setFrom(msg);
	dataMutex.unlock();
}

void KeyFrameGraphDisplay::addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	dataMutex.lock();

	constraints.resize(msg->numConstraints);
	assert(msg->constraintsData.size() == sizeof(GraphConstraint)*msg->numConstraints);
	GraphConstraint* constraintsIn = (GraphConstraint*)msg->constraintsData.data();
	for(int i = 0; i < (int)msg->numConstraints; i++)
	{
		GraphConstraintPt new_constraint;
		new_constraint.err = constraintsIn[i].err;
		new_constraint.from = 0;
		new_constraint.to = 0;

		if(keyframesByID.count(constraintsIn[i].from) != 0)
			new_constraint.from = keyframesByID[constraintsIn[i].from].get();
//		else
//			printf("ERROR: graph update contains constraints for %d -> %d, but I dont have a frame %d!\n",
//					constraintsIn[i].from,
//					constraintsIn[i].to,
//					constraintsIn[i].from);


		if(keyframesByID.count(constraintsIn[i].to) != 0)
			new_constraint.to = keyframesByID[constraintsIn[i].to].get();
//		else
//			printf("ERROR: graph update contains constraints for %d -> %d, but I dont have a frame %d!\n",
//					constraintsIn[i].from,
//					constraintsIn[i].to,
//					constraintsIn[i].to);
		constraints.push_back(new_constraint);
	}

	GraphFramePose* graphPoses = (GraphFramePose*)msg->frameData.data();
	int numGraphPoses = msg->numFrames;
	assert(msg->frameData.size() == sizeof(GraphFramePose)*msg->numFrames);

	for(int i=0;i<numGraphPoses;i++)
	{
		if(keyframesByID.count(graphPoses[i].id) == 0)
		{
		//	printf("ERROR: graph update contains pose for frame %d, but I dont have a frame %d!\n", graphPoses[i].id, graphPoses[i].id);
		}
		else
			memcpy(keyframesByID[graphPoses[i].id]->camToWorld.data(), graphPoses[i].camToWorld, 7*sizeof(float));
	}

	dataMutex.unlock();

//	printf("graph update: %d constraints, %d poses\n", msg->numConstraints, msg->numFrames);
}

void KeyFrameGraphDisplay::addConstraint(const KeyFrameDisplayPtr& from, const KeyFrameDisplayPtr& to, float err)
{
	//dataMutex.lock();
	
	GraphConstraintPt new_constraint;
	new_constraint.err = err;
	new_constraint.from = from.get();
	new_constraint.to = to.get();
	constraints.push_back(new_constraint);
	
	//dataMutex.unlock();
}

void KeyFrameGraphDisplay::addGraph(int pivot, int other_pivot, const KeyFrameGraphDisplay* graph)
{
	dataMutex.lock();
	
	//Sophus::Sim3f transformToOtherPivot = (keyframesByID.at(pivot)->camToWorld.inverse() * graph->getKeyFramesByID().at(other_pivot)->camToWorld).inverse();
	
	for (auto frame_kv : graph->getKeyFramesByID())
	{
		keyframesByID[frame_kv.first] = frame_kv.second;
		//keyframesByID[frame_kv.first]->camToWorld = transformToOtherPivot * keyframesByID[frame_kv.first]->camToWorld;
	}
	
	const auto& other_constraints = graph->getConstraints();
	std::copy(other_constraints.cbegin(), other_constraints.cend(), constraints.end());
	
	addConstraint(keyframesByID.at(pivot), graph->getKeyFramesByID().at(other_pivot));
	addConstraint(graph->getKeyFramesByID().at(other_pivot), keyframesByID.at(pivot)); //???
	
	dataMutex.unlock();
}