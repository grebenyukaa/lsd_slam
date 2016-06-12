#include <QString>

#include "MultiAgentPointCloudViewer.h"
#include "KeyFrameDisplay.h"
#include "KeyFrameGraphDisplay.h"

MultiAgentPointCloudViewer::MultiAgentPointCloudViewer(int nAgents)
{
	for (int i = 0; i <= nAgents; ++i)
	{
		m_viewers[i] = new PointCloudViewer(i, this);
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
		//printf("Is KF!\n");
		PointCloudViewer* ref_vwr = m_viewers.at(msg->agentId);
		
		if (ref_vwr->getKFCount() <= cutFirstNKf) return; 
			
		const KeyFrameDisplayPtr& ref_kf = ref_vwr->getKFGraphDisplay()->getKeyFramesByID().at(msg->id);
		
		double minDist;
		double maxAngleCos;
		int equal_kf_id = m_viewers[0]->findEqualKF(ref_kf, minDist, maxAngleCos);
		if (equal_kf_id > 0)
		{
			printf("agent #%d: found equal KF #%d, ref KF #%d, ref agentID #%d\n", msg->agentId, equal_kf_id, ref_kf->id, ref_kf->getAgentId());
			const KeyFrameDisplayPtr& pivot_kf = m_viewers[0]->getKFGraphDisplay()->getKeyFramesByID().at(equal_kf_id);
			
			bool fDist = false;
			bool fAngle = false;
			if (minDist < ref_vwr->getMinAlignDist())
			{
				ref_vwr->setMinAlignDist(minDist);
				fDist = true;
			}
			if (maxAngleCos > ref_vwr->getMaxAlignCos())
			{
				ref_vwr->setMaxAlignCos(maxAngleCos);
				fAngle = true;
			}
			if (fDist && fAngle)
			{
				printf("Found better align transform: dist %f cos(angle) %f\n", minDist, maxAngleCos);
				ref_vwr->setAlignTransform(pivot_kf->camToWorld * ref_kf->camToWorld.inverse());
			}
						
			m_viewers[0]->getKFGraphDisplay()->addGraph(equal_kf_id, ref_kf->id, m_viewers[msg->agentId]->getKFGraphDisplay());
			m_viewers[0]->resetAnimation(0, cutFirstNKf);
		}
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