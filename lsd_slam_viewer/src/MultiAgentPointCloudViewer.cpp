#include <QString>

#include "MultiAgentPointCloudViewer.h"
#include "KeyFrameDisplay.h"
#include "KeyFrameGraphDisplay.h"

MultiAgentPointCloudViewer::MultiAgentPointCloudViewer(int nAgents)
{
	for (int i = 0; i <= nAgents; ++i)
	{
		m_viewers[i] = new PointCloudViewer(i);
	}
}

void MultiAgentPointCloudViewer::addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	m_viewers[msg->agentId]->addFrameMsg(msg);
	if (msg->agentId == 1)
	{
		m_viewers[0]->addFrameMsg(msg);
		return;
	}
	
	if (msg->isKeyframe)
	{
		printf("Is KF!\n");
		const KeyFrameDisplayPtr& ref_kf = m_viewers[msg->agentId]->getKFGraphDisplay()->getKeyFramesByID().at(msg->id);
		/*for (int i = 1; i < m_viewers.size(); ++i)
		{
			if (i == (int)msg->agentId)
				continue;*/
			int equal_kf_id = m_viewers[/*i*/0]->getKFGraphDisplay()->findEqualKF(ref_kf);
			if (equal_kf_id > 0)
			{
				printf("Found equal KF #%d!, ref KF #%d, ref agentID #%d\n", equal_kf_id, ref_kf->id, msg->agentId);
				m_viewers[0]->getKFGraphDisplay()->addGraph(equal_kf_id, ref_kf->id, m_viewers[msg->agentId]->getKFGraphDisplay());
				m_viewers[0]->resetAnimation(0, 0);
			}
		/*
		}*/
	}
}

void MultiAgentPointCloudViewer::addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	m_viewers[msg->agentId]->addGraphMsg(msg);
	if (msg->agentId == 1)
	{
		m_viewers[0]->addGraphMsg(msg);
	}
}

void MultiAgentPointCloudViewer::init()
{
	for (int i = 0; i < (int)m_viewers.size(); ++i)
	{
		QString wnd_title = i != 0 ? QString("PointCloud Viewer #%1").arg(i) : QString("Global PointCloud");
		
		#if QT_VERSION < 0x040000
			// Set the viewer as the application main widget.
			//application.setMainWidget(viewer);
			assert("Not implemented!" && 0);
		#else
			m_viewers[i]->setWindowTitle(wnd_title);
		#endif

		// Make the viewer window visible on screen.
		m_viewers[i]->show();
	}
}