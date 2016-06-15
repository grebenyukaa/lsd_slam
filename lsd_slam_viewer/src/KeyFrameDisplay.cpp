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

#define GL_GLEXT_PROTOTYPES 1

#include "KeyFrameDisplay.h"
#include <stdio.h>
#include "settings.h"

#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "opencv2/opencv.hpp"

#include "ros/package.h"

KeyFrameDisplay::KeyFrameDisplay()
{
	init(-1);
}

KeyFrameDisplay::KeyFrameDisplay(int agentId)
{
	init(agentId);
}

KeyFrameDisplay::KeyFrameDisplay(const KeyFrameDisplay& other)
	:
	id(other.id),
	time(other.time),
	totalPoints(other.totalPoints),
	displayedPoints(other.displayedPoints),
	agentId(other.agentId),
	fx(other.fx),
	fy(other.fy),
	cx(other.cx),
	cy(other.cy),
	fxi(other.fxi),
	fyi(other.fyi),
	cxi(other.cxi),
	cyi(other.cyi),
	width(other.width),
	height(other.height),
	my_scaledTH(other.my_scaledTH),
	my_absTH(other.my_absTH),
	my_scale(other.my_scale),
	my_minNearSupport(other.my_minNearSupport),
	my_sparsifyFactor(other.my_sparsifyFactor),
	meanIDepth(other.meanIDepth),
	vertexBufferId(other.vertexBufferId),
	vertexBufferNumPoints(other.vertexBufferNumPoints),
	vertexBufferIdValid(other.vertexBufferIdValid),
	glBuffersValid(other.glBuffersValid)
{
	memcpy(camToWorld.data(), other.camToWorld.data(), 7*sizeof(float));
	originalInput = other.originalInput;
}

void KeyFrameDisplay::init(int aid)
{
	agentId = aid;
	meanIDepth = 0;
	
	originalInput.resize(0);
	id = 0;
	vertexBufferIdValid = false;
	glBuffersValid = false;

	camToWorld = Sophus::Sim3f();
	width=height=0;

	my_scaledTH = my_absTH = 0;

	totalPoints = displayedPoints = 0;
}

KeyFrameDisplay::~KeyFrameDisplay()
{
	if(vertexBufferIdValid)
	{
		glDeleteBuffers(1, &vertexBufferId);
		vertexBufferIdValid = false;
	}
}

void KeyFrameDisplay::setFrom(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	// copy over campose.
	memcpy(camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));

	fx = msg->fx;
	fy = msg->fy;
	cx = msg->cx;
	cy = msg->cy;

	fxi = 1/fx;
	fyi = 1/fy;
	cxi = -cx / fx;
	cyi = -cy / fy;

	width = msg->width;
	height = msg->height;
	id = msg->id;
	agentId = msg->agentId;
	time = msg->time;

	originalInput.resize(0);

	if(msg->pointcloud.size() != width*height*sizeof(InputPointDense))
	{
		if(msg->pointcloud.size() != 0)
		{
			printf("WARNING: PC with points, but number of points not right! (is %zu, should be %lu*%dx%d=%lu)\n",
					msg->pointcloud.size(), sizeof(InputPointDense), width, height, width*height*sizeof(InputPointDense));
		}
	}
	else
	{
		originalInput.resize(width*height);
		memcpy(originalInput.data(), msg->pointcloud.data(), width*height*sizeof(InputPointDense));
		//std::copy_n((uint8_t*)msg->pointcloud.data(), sizeof(InputPointDense)*width*height, (uint8_t*)&originalInput[0]);
	}

	meanIDepth = 0;
	for (const auto sidepth : originalInput)
		meanIDepth += sidepth.idepth;
	meanIDepth /= width * height;

	glBuffersValid = false;
}

void KeyFrameDisplay::refreshPC(const Sophus::Sim3f& alignTransform)
{
	Sophus::Sim3f curCamToWorld = alignTransform * camToWorld;
//	minNearSupport = 9;
	bool paramsStillGood = my_scaledTH == scaledDepthVarTH &&
			my_absTH == absDepthVarTH &&
			my_scale*1.2 > curCamToWorld.scale() &&
			my_scale < curCamToWorld.scale()*1.2 &&
			my_minNearSupport == minNearSupport &&
			my_sparsifyFactor == sparsifyFactor;

	if(glBuffersValid && (paramsStillGood || numRefreshedAlready > 10)) return;
	numRefreshedAlready++;

	glBuffersValid = true;

	// delete old vertex buffer
	if(vertexBufferIdValid)
	{
		glDeleteBuffers(1, &vertexBufferId);
		vertexBufferIdValid = false;
	}

	// if there are no vertices, done!
	if(originalInput.size() == 0)
	{
		return;
	}

	// make data
	MyVertex* tmpBuffer = new MyVertex[width*height];

	my_scaledTH =scaledDepthVarTH;
	my_absTH = absDepthVarTH;
	my_scale = curCamToWorld.scale();
	my_minNearSupport = minNearSupport;
	my_sparsifyFactor = sparsifyFactor;
	// data is directly in ros message, in correct format.
	vertexBufferNumPoints = 0;

	int total = 0, displayed = 0;
	for(int y=1;y<height-1;y++)
		for(int x=1;x<width-1;x++)
		{
			if(originalInput[x+y*width].idepth <= 0) continue;
			total++;


			if(my_sparsifyFactor > 1 && rand()%my_sparsifyFactor != 0) continue;

			float depth = 1 / originalInput[x+y*width].idepth;
			float depth4 = depth*depth; depth4*= depth4;


			if(originalInput[x+y*width].idepth_var * depth4 > my_scaledTH)
				continue;

			if(originalInput[x+y*width].idepth_var * depth4 * my_scale*my_scale > my_absTH)
				continue;

			if(my_minNearSupport > 1)
			{
				int nearSupport = 0;
				for(int dx=-1;dx<2;dx++)
					for(int dy=-1;dy<2;dy++)
					{
						int idx = x+dx+(y+dy)*width;
						if(originalInput[idx].idepth > 0)
						{
							float diff = originalInput[idx].idepth - 1.0f / depth;
							if(diff*diff < 2*originalInput[x+y*width].idepth_var)
								nearSupport++;
						}
					}

				if(nearSupport < my_minNearSupport)
					continue;
			}

			tmpBuffer[vertexBufferNumPoints].point[0] = (x*fxi + cxi) * depth;
			tmpBuffer[vertexBufferNumPoints].point[1] = (y*fyi + cyi) * depth;
			tmpBuffer[vertexBufferNumPoints].point[2] = depth;

			tmpBuffer[vertexBufferNumPoints].color[3] = 100;
			tmpBuffer[vertexBufferNumPoints].color[2] = originalInput[x+y*width].color[0];
			tmpBuffer[vertexBufferNumPoints].color[1] = originalInput[x+y*width].color[1];
			tmpBuffer[vertexBufferNumPoints].color[0] = originalInput[x+y*width].color[2];

			vertexBufferNumPoints++;
			displayed++;
		}
	totalPoints = total;
	displayedPoints = displayed;

	// create new ones, static
	vertexBufferId=0;
	glGenBuffers(1, &vertexBufferId);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId);         // for vertex coordinates
	glBufferData(GL_ARRAY_BUFFER, sizeof(MyVertex) * vertexBufferNumPoints, tmpBuffer, GL_STATIC_DRAW);
	vertexBufferIdValid = true;

	if (!keepInMemory)
	{
		auto tmp = std::vector<InputPointDense>();
		originalInput.swap(tmp);
	}

	delete[] tmpBuffer;
}

void KeyFrameDisplay::drawCam(const Sophus::Sim3f& alignTransform, float lineWidth, const float* color)
{
	if(width == 0)
		return;

	glPushMatrix();

		Sophus::Sim3f tr = alignTransform * camToWorld;
		Sophus::Matrix4f m = tr.matrix();
		glMultMatrixf((GLfloat*)m.data());

		if(color == 0)
			glColor3f(1,0,0);
		else
			glColor3f(color[0],color[1],color[2]);

		glLineWidth(lineWidth);
		glBegin(GL_LINES);
		glVertex3f(0,0,0);
		glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);
		glVertex3f(0,0,0);
		glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);
		glVertex3f(0,0,0);
		glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);
		glVertex3f(0,0,0);
		glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);

		glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);
		glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);

		glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);
		glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);

		glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);
		glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);

		glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);
		glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);

		glEnd();
	glPopMatrix();
}

int KeyFrameDisplay::flushPC(const Sophus::Sim3f& alignTransform, std::ofstream* f)
{
	MyVertex* tmpBuffer = new MyVertex[width*height];
	int num = 0;
	for(int y=1;y<height-1;y++)
		for(int x=1;x<width-1;x++)
		{
			if(originalInput[x+y*width].idepth <= 0) continue;

			if(my_sparsifyFactor > 1 && rand()%my_sparsifyFactor != 0) continue;

			float depth = 1 / originalInput[x+y*width].idepth;
			float depth4 = depth*depth; depth4*= depth4;

			if(originalInput[x+y*width].idepth_var * depth4 > my_scaledTH)
				continue;

			if(originalInput[x+y*width].idepth_var * depth4 * my_scale*my_scale > my_absTH)
				continue;

			if(my_minNearSupport > 1)
			{
				int nearSupport = 0;
				for(int dx=-1;dx<2;dx++)
					for(int dy=-1;dy<2;dy++)
					{
						int idx = x+dx+(y+dy)*width;
						if(originalInput[idx].idepth > 0)
						{
							float diff = originalInput[idx].idepth - 1.0f / depth;
							if(diff*diff < 2*originalInput[x+y*width].idepth_var)
								nearSupport++;
						}
					}

				if(nearSupport < my_minNearSupport)
					continue;
			}

			Sophus::Sim3f tr = alignTransform * camToWorld;
			Sophus::Vector3f pt = tr * (Sophus::Vector3f((x*fxi + cxi), (y*fyi + cyi), 1) * depth);
			tmpBuffer[num].point[0] = pt[0];
			tmpBuffer[num].point[1] = pt[1];
			tmpBuffer[num].point[2] = pt[2];



			tmpBuffer[num].color[3] = 100;
			tmpBuffer[num].color[2] = originalInput[x+y*width].color[0];
			tmpBuffer[num].color[1] = originalInput[x+y*width].color[1];
			tmpBuffer[num].color[0] = originalInput[x+y*width].color[2];

			num++;
		}

	for(int i=0;i<num;i++)
	{
		f->write((const char *)tmpBuffer[i].point,3*sizeof(float));
		float color = tmpBuffer[i].color[0] / 255.0;
		f->write((const char *)&color,sizeof(float));
	}
	//	*f << tmpBuffer[i].point[0] << " " << tmpBuffer[i].point[1] << " " << tmpBuffer[i].point[2] << " " << (tmpBuffer[i].color[0] / 255.0) << "\n";

	delete tmpBuffer;

	printf("Done flushing frame %d (%d points)!\n", this->id, num);
	return num;
}

void KeyFrameDisplay::drawPC(const Sophus::Sim3f& alignTransform, float pointSize, float alpha)
{
	refreshPC(alignTransform);

	if(!vertexBufferIdValid)
	{
		return;
	}

	GLfloat LightColor[] = {1, 1, 1, 1};
	if(alpha < 1)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		LightColor[0] = LightColor[1] = 0;
		glEnable(GL_LIGHTING);
		glDisable(GL_LIGHT1);

		glLightfv (GL_LIGHT0, GL_AMBIENT, LightColor);
	}
	else
	{
		glDisable(GL_LIGHTING);
	}


	glPushMatrix();

		Sophus::Sim3f tr = alignTransform * camToWorld;
		Sophus::Matrix4f m = tr.matrix();
		glMultMatrixf((GLfloat*)m.data());

		glPointSize(pointSize);

		glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId);

		glVertexPointer(3, GL_FLOAT, sizeof(MyVertex), 0);
		glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(MyVertex), (const void*) (3*sizeof(float)));

		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);

		glDrawArrays(GL_POINTS, 0, vertexBufferNumPoints);

		glDisableClientState(GL_COLOR_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();

	if(alpha < 1)
	{
		glDisable(GL_BLEND);
		glDisable(GL_LIGHTING);
		LightColor[2] = LightColor[1] = LightColor[0] = 1;
		glLightfv (GL_LIGHT0, GL_AMBIENT_AND_DIFFUSE, LightColor);
	}
}

