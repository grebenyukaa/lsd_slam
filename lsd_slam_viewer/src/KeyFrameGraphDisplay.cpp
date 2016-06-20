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
#include "MultiAgentPointCloudViewer.h"
#include "PointCloudViewer.h"

KeyFrameGraphDisplay::KeyFrameGraphDisplay(int agentId, PointCloudViewer* vwr)
	:
	agentId(agentId),
	vwr(vwr)
{
	flushPointcloud = true;
	printNumbers = false;
}

KeyFrameGraphDisplay::~KeyFrameGraphDisplay()
{}

void KeyFrameGraphDisplay::draw()
{
	dataMutex.lock();
	numRefreshedAlready = 0;

	// draw keyframes
	int idx = 0;
	for (auto frame_kv : keyframesByID)
	{
		auto& frame = frame_kv.second;
		int frameAgentId = frame.getAgentId();
		
		PointCloudViewer* frame_vwr = vwr->getParent()->getViewer(frameAgentId);
		const Sophus::Sim3f& alignTransform = frameAgentId != agentId ? frame_vwr->getAlignTransform() : Sophus::Sim3f();
			
		if (showKFCameras)
			frame.drawCam(alignTransform, lineTesselation, frame_vwr->getColor());

		if ((showKFPointclouds && idx > cutFirstNKf) || idx == (int)keyframesByID.size() - 1)
			frame.drawPC(alignTransform, pointTesselation, 1);
		
		++idx;
	}

	// if(flushPointcloud)
	// {
	// 	printf("Flushing Pointcloud to %s!\n", (ros::package::getPath("lsd_slam_viewer")+"/pc_tmp.ply").c_str());
	// 	std::ofstream f((ros::package::getPath("lsd_slam_viewer")+"/pc_tmp.ply").c_str());
	// 	int numpts = 0;
	// 	int idx = 0;
	// 	for (auto frame_kv : keyframesByID)
	// 	{
	// 		auto& frame = frame_kv.second;
	// 		int frameAgentId = frame.getAgentId();
	// 		const Sophus::Sim3f& alignTransform = frameAgentId != agentId ? vwr->getParent()->getViewer(frameAgentId)->getAlignTransform() : Sophus::Sim3f();
			
	// 		if (idx > cutFirstNKf)
	// 			numpts += frame.flushPC(alignTransform, &f);
	// 		++idx;
	// 	}
	// 	f.flush();
	// 	f.close();

	// 	std::ofstream f2((ros::package::getPath("lsd_slam_viewer")+"/pc.ply").c_str());
	// 	f2 << std::string("ply\n");
	// 	f2 << std::string("format binary_little_endian 1.0\n");
	// 	f2 << std::string("element vertex ") << numpts << std::string("\n");
	// 	f2 << std::string("property float x\n");
	// 	f2 << std::string("property float y\n");
	// 	f2 << std::string("property float z\n");
	// 	f2 << std::string("property float intensity\n");
	// 	f2 << std::string("end_header\n");

	// 	std::ifstream f3((ros::package::getPath("lsd_slam_viewer")+"/pc_tmp.ply").c_str());
	// 	while(!f3.eof()) f2.put(f3.get());

	// 	f2.close();
	// 	f3.close();

	// 	system(("rm "+ros::package::getPath("lsd_slam_viewer")+"/pc_tmp.ply").c_str());
	// 	flushPointcloud = false;
	// 	printf("Done Flushing Pointcloud with %d points!\n", numpts);
	// }

	if (flushPointcloud)
	{
		std::ostringstream oss;
		oss << "pc" << agentId << ".txt";
		std::ofstream ofs((ros::package::getPath("lsd_slam_viewer")+oss.str().c_str()).c_str());
		
		for (auto frame_kv : keyframesByID)
		{
			auto& frame = frame_kv.second;
			int frameAgentId = frame.getAgentId();
			const Sophus::Sim3f& alignTransform = frameAgentId != agentId ? vwr->getParent()->getViewer(frameAgentId)->getAlignTransform() : Sophus::Sim3f();
			
			if (idx < cutFirstNKf)
			{
				++idx;
				continue;
			}
			
			ofs << frame.id << std::endl;
			ofs << frame.getAgentId()<< std::endl;
			ofs << frame.camToWorld.rotationMatrix().data()[0] << " " << frame.camToWorld.rotationMatrix().data()[1] << " " << frame.camToWorld.rotationMatrix().data()[2] << " " << frame.camToWorld.translation()[0] << std::endl;
			ofs << frame.camToWorld.rotationMatrix().data()[3] << " " << frame.camToWorld.rotationMatrix().data()[4] << " " << frame.camToWorld.rotationMatrix().data()[5] << " " << frame.camToWorld.translation()[1] << std::endl;
			ofs << frame.camToWorld.rotationMatrix().data()[6] << " " << frame.camToWorld.rotationMatrix().data()[7] << " " << frame.camToWorld.rotationMatrix().data()[8] << " " << frame.camToWorld.translation()[2] << std::endl;
			ofs << 0 << " " << 0 << " " << 0 << " " << frame.camToWorld.scale() << std::endl;
			ofs.flush();
		}
		
		ofs.close();
	}

	if(printNumbers)
	{
		int totalPoint = 0;
		int visPoints = 0;
		for (auto frame_kv : keyframesByID)
		{
			totalPoint += frame_kv.second.totalPoints;
			visPoints += frame_kv.second.displayedPoints;
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
			if(constraint.from == -1 || constraint.to == -1)
				continue;
				
			//printf("[anim] show constr from (%d, %d) to (%d, %d)\n", constraint.fromAgentId, constraint.from, constraint.toAgentId, constraint.to);
			
			const Sophus::Sim3f& alignTransformFrom = constraint.fromAgentId != agentId ? 
				vwr->getParent()->getViewer(constraint.fromAgentId)->getAlignTransform() : Sophus::Sim3f();
			const Sophus::Sim3f& alignTransformTo = constraint.toAgentId != agentId ? 
				vwr->getParent()->getViewer(constraint.toAgentId)->getAlignTransform() : Sophus::Sim3f();

			const KeyFrameDisplay& from = vwr->getParent()->getViewer(constraint.fromAgentId)->getKFGraphDisplay()->getKeyFramesByID().at(constraint.from);
			const KeyFrameDisplay& to = vwr->getParent()->getViewer(constraint.toAgentId)->getKFGraphDisplay()->getKeyFramesByID().at(constraint.to);

			double colorScalar = std::max(0.0, std::min(1.0, constraint.err / 0.05));
			glColor3f(colorScalar, 1 - colorScalar, 0);

			Sophus::Vector3f t = (alignTransformFrom * from.camToWorld).translation();
			glVertex3f((GLfloat)t[0], (GLfloat)t[1], (GLfloat)t[2]);

			t = (alignTransformTo * to.camToWorld).translation();
			glVertex3f((GLfloat)t[0], (GLfloat)t[1], (GLfloat)t[2]);
		}
		glEnd();
	}

	dataMutex.unlock();
}

int KeyFrameGraphDisplay::findEqualKF(const Sophus::Sim3f& alignTransform, const KeyFrameDisplay& kf, double& dist, double& angleCos, float ClosenessTH, float KFDistWeight)
{
	float fowX, fowY;
	fowX = 2 * atanf((float)((kf.getWidth() / kf.getFx() / 2.0f)));
	fowY = 2 * atanf((float)((kf.getHeight() / kf.getFy() / 2.0f)));
	
	//from solver
	float distanceTH = ClosenessTH * 15 / (KFDistWeight*KFDistWeight);
	float angleTH = 1.0 - 0.25 * ClosenessTH;
	bool checkBothScales = true;
		
	// basically the maximal angle-difference in viewing direction is angleTH*(average FoV).
	// e.g. if the FoV is 130°, then it is angleTH*130°.
	float cosAngleTH = cosf(angleTH*0.5f*(fowX + fowY));

	Sophus::Sim3f ctw = alignTransform * kf.camToWorld;
	Eigen::Vector3f pos = ctw.translation();
	Eigen::Vector3f viewingDir = ctw.rotationMatrix().rightCols<1>();

	float iDepth = kf.getMeanIDepth();

	float distFacReciprocal = 1;
	if(checkBothScales)
		distFacReciprocal = iDepth / ctw.scale();

	// for each frame, calculate the rough score, consisting of pose, scale and angle overlap.
	double minDist = std::numeric_limits<double>::max();
	double maxAngleCos = 0;
	int idBestCos = -1;
	int idBestDist = -1;
	int idBestKF = -1;
	dataMutex.lock();
	for (const auto& kf_kv : keyframesByID)
	{
		if (kf_kv.second.getAgentId() == kf.getAgentId()) continue;
		
		const KeyFrameDisplay& curKF = kf_kv.second;
		Eigen::Vector3f otherPos = curKF.camToWorld.translation();
		float curIDepth = curKF.getMeanIDepth();

		// get distance between the frames, scaled to fit the potential reference frame.
		float distFac = curIDepth / curKF.camToWorld.scale();
		if (checkBothScales && distFacReciprocal < distFac) distFac = distFacReciprocal;
		Eigen::Vector3f dist = (pos - otherPos) * distFac;
		float dNorm2 = dist.dot(dist);
		if (dNorm2 > distanceTH) continue;

		Eigen::Vector3f otherViewingDir = curKF.camToWorld.rotationMatrix().rightCols<1>();
		float dirDotProd = otherViewingDir.dot(viewingDir);
		if (dirDotProd < cosAngleTH) continue;
		
		bool betterDist = dNorm2 < minDist;
		bool betterCos = dirDotProd > maxAngleCos;
		if (betterDist && betterCos)
		{
			idBestDist = kf_kv.first;
			minDist = dNorm2;
			idBestCos = kf_kv.first;
			maxAngleCos = dirDotProd;
			idBestKF = kf_kv.first;
		}
		else
		{
			if (betterCos)
			{
				idBestCos = kf_kv.first;
				maxAngleCos = dirDotProd;
			}
			else
			{
				idBestDist = kf_kv.first;
				minDist = dNorm2;
			}
		}
	}
	dataMutex.unlock();

	dist = minDist;
	angleCos = maxAngleCos;
	return idBestKF >= 0 ? idBestKF : idBestDist;
}

void KeyFrameGraphDisplay::addMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	dataMutex.lock();
	if (keyframesByID.find(msg->id) == keyframesByID.cend())
	{
		keyframesByID[msg->id] = KeyFrameDisplay(agentId);
		//keyframes.push_back(disp);

	//	printf("added new KF, now there are %d!\n", (int)keyframes.size());
	}

	keyframesByID[msg->id].setFrom(msg);
	dataMutex.unlock();
}

void KeyFrameGraphDisplay::addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	dataMutex.lock();

	assert(msg->constraintsData.size() == sizeof(GraphConstraint)*msg->numConstraints);
	GraphConstraint* constraintsIn = (GraphConstraint*)msg->constraintsData.data();
	for(int i = 0; i < (int)msg->numConstraints; i++)
	{
		GraphConstraintPt new_constraint;
		new_constraint.err = constraintsIn[i].err;
		new_constraint.from = -1;
		new_constraint.to = -1;

		if(keyframesByID.count(constraintsIn[i].from) != 0)
		{
			new_constraint.from = constraintsIn[i].from;
			new_constraint.fromAgentId = msg->agentId;
		}
//		else
//			printf("ERROR: graph update contains constraints for %d -> %d, but I dont have a frame %d!\n",
//					constraintsIn[i].from,
//					constraintsIn[i].to,
//					constraintsIn[i].from);


		if(keyframesByID.count(constraintsIn[i].to) != 0)
		{
			new_constraint.to = constraintsIn[i].to;
			new_constraint.toAgentId = msg->agentId;
		}
//		else
//			printf("ERROR: graph update contains constraints for %d -> %d, but I dont have a frame %d!\n",
//					constraintsIn[i].from,
//					constraintsIn[i].to,
//					constraintsIn[i].to);
		//printf("[vwr %d] add constraint from (%d, %d) to (%d, %d)\n", agentId, new_constraint.fromAgentId, new_constraint.from, new_constraint.toAgentId, new_constraint.to);
		constraints.push_back(new_constraint);
	}

	GraphFramePose* graphPoses = (GraphFramePose*)msg->frameData.data();
	int numGraphPoses = msg->numFrames;
	assert(msg->frameData.size() == sizeof(GraphFramePose)*msg->numFrames);

	for(int i = 0; i < numGraphPoses; i++)
	{
		if(keyframesByID.count(graphPoses[i].id) == 0)
		{
		//	printf("ERROR: graph update contains pose for frame %d, but I dont have a frame %d!\n", graphPoses[i].id, graphPoses[i].id);
		}
		else
			memcpy(keyframesByID[graphPoses[i].id].camToWorld.data(), graphPoses[i].camToWorld, 7*sizeof(float));
	}

	dataMutex.unlock();

//	printf("graph update: %d constraints, %d poses\n", msg->numConstraints, msg->numFrames);
}

void KeyFrameGraphDisplay::addConstraint(int fromAgentId, int fromId, int toAgentId, int toId, float err)
{
	//dataMutex.lock();
	
	GraphConstraintPt new_constraint;
	new_constraint.err = err;
	new_constraint.from = fromId;
	new_constraint.fromAgentId = fromAgentId;
	new_constraint.to = toId;
	new_constraint.toAgentId = toAgentId;
	constraints.push_back(new_constraint);
	
	//dataMutex.unlock();
}

void KeyFrameGraphDisplay::addGraph(int pivot, int other_pivot, const KeyFrameGraphDisplay* graph)
{
	dataMutex.lock();
	
	for (auto frame_kv : graph->getKeyFramesByID())
	{
		keyframesByID[frame_kv.first] = KeyFrameDisplay(frame_kv.second);
	}
	
	std::copy(graph->getConstraints().cbegin(), graph->getConstraints().cend(), constraints.begin());
	
	addConstraint(agentId, pivot, graph->getAgentId(), other_pivot);
	addConstraint(graph->getAgentId(), other_pivot, agentId, pivot);
	
	dataMutex.unlock();
}