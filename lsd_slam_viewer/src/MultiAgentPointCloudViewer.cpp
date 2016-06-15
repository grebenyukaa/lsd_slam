#include <QString>
#include <random>

#include "MultiAgentPointCloudViewer.h"
#include "KeyFrameDisplay.h"
#include "KeyFrameGraphDisplay.h"

float randf(std::mt19937& gen)
{
	return std::uniform_real_distribution<float>(0, 1)(gen);
} 

void getRandomRGBColor(std::mt19937& gen, int i, int n, float* rgb)
{
	int H = (int)round((i / (float)n) * 6);
	float S = 0.9 + randf(gen) / 10;
	float L = 0.5 + randf(gen) / 10;

	//printf("H %d S %f L %f\n", H * 60, S, L);

	float c = (1 - fabs(2*L - 1)) * S;
	float x = c * (1 - abs((H % 2) - 1));
	float m = L - c/2;
	
	if (H < 60)  { rgb[0] = c + m; rgb[1] = x + m; rgb[2] = 0 + m; return; }
	if (H < 120) { rgb[0] = x + m; rgb[1] = c + m; rgb[2] = 0 + m; return; }
	if (H < 180) { rgb[0] = 0 + m; rgb[1] = c + m; rgb[2] = x + m; return; }
	if (H < 240) { rgb[0] = 0 + m; rgb[1] = x + m; rgb[2] = c + m; return; }
	if (H < 300) { rgb[0] = x + m; rgb[1] = 0 + m; rgb[2] = c + m; return; }
	if (H < 360) { rgb[0] = c + m; rgb[1] = 0 + m; rgb[2] = x + m; return; }
	
	assert("Should not happen!" && 0);
	rgb[0] = m; rgb[1] = m; rgb[2] = m;
}

MultiAgentPointCloudViewer::MultiAgentPointCloudViewer(int nAgents)
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<float> dis(0, 1);
	
	for (int i = 0; i <= nAgents; ++i)
	{
		float color[3];
		getRandomRGBColor(gen, i, nAgents, color);
		printf("agent #%d (%f, %f, %f)\n", i, color[0], color[1], color[2]);
		m_viewers[i] = new PointCloudViewer(i, this, color);
	}
}

void MultiAgentPointCloudViewer::countLogNormResidual(int agentId, int frameId, int refAgentId, int refFrameId, float& avg, float& max)
{
	printf("Count avg log norm\n");
	
	PointCloudViewer* vwr = m_viewers[agentId];
	PointCloudViewer* ref_vwr = m_viewers[refAgentId];
	const KeyFrameDisplay& pivot_kf = vwr->getKFGraphDisplay()->getKeyFramesByID().at(frameId);
	
	avg = 0;
	max = 0;
	int count = 0;
	for (auto refId : m_equal_kfs[agentId][refAgentId][frameId])
	{
		const KeyFrameDisplay& ref_kf = ref_vwr->getKFGraphDisplay()->getKeyFramesByID().at(refId);
		Sophus::Sim3f alignedRefKF = ref_vwr->getAlignTransform() * ref_kf.camToWorld;
		Sophus::Sim3f residual = pivot_kf.camToWorld * alignedRefKF.inverse();
		float val = residual.log().norm();
		if (val > max)
			max = val;
		avg += val;
		++count;
	}
	avg /= count;
	
	printf("  avg log norm: %f\n", avg);
	printf("  max log norm: %f\n", max);
}

bool MultiAgentPointCloudViewer::findEqualAndAdjustAlignTransform(int refId, int refAgentId)
{
	PointCloudViewer* ref_vwr = m_viewers[refAgentId];
	const KeyFrameDisplay& ref_kf = ref_vwr->getKFGraphDisplay()->getKeyFramesByID().at(refId);
	
	for (int i = 1; i < (int)m_viewers.size(); ++i)
	{
		double minDist;
		double maxAngleCos;
		int equal_kf_id = m_viewers[i]->findEqualKF(ref_kf, minDist, maxAngleCos);
		
		if (equal_kf_id > 0)
		{
			printf("agent #%d: found equal KF #%d, ref KF #%d, ref agentID #%d\n", refAgentId, equal_kf_id, ref_kf.id, ref_kf.getAgentId());
			const KeyFrameDisplay& pivot_kf = m_viewers[0]->getKFGraphDisplay()->getKeyFramesByID().at(equal_kf_id);
			
			bool fDist = false;
			bool fAngle = false;
			
			if (minDist < 0.1 && maxAngleCos > 0.98)
			{
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
					Sophus::Sim3f alignTransform = pivot_kf.camToWorld * ref_kf.camToWorld.inverse();
					//Sophus::Sim3f alignTransform = (ref_kf.camToWorld.inverse() * pivot_kf.camToWorld).inverse()
					//alignTransform.setScale(1);
					ref_vwr->setAlignTransform(alignTransform * ref_vwr->getAlignTransform());
					
					m_equal_kfs[0][refAgentId][equal_kf_id].push_back(ref_kf.id);
					return true;
				}
				
				//Check align
				Sophus::Sim3f alignedRefKF = ref_vwr->getAlignTransform() * ref_kf.camToWorld;
				Sophus::Sim3f residual = pivot_kf.camToWorld * alignedRefKF.inverse();
				double logNorm = residual.log().norm();
				printf("Residual log norm: %f\n", logNorm);
				
				float avgResidual;
				float maxResidual;
				countLogNormResidual(0, equal_kf_id, ref_kf.getAgentId(), ref_kf.id, avgResidual, maxResidual);
				
				if (maxResidual > 0.1 || avgResidual > 0.01)
				{
					printf("Adjusting ref ctw\n");
					ref_vwr->setAlignTransform(residual * ref_vwr->getAlignTransform());
					//m_viewers[0]->getKFGraphDisplay()->setCamToWorld(ref_kf.id, ref_vwr->getAlignTransform().inverse() * pivot_kf.camToWorld);
				}
			}
		}
	}
	return false;
}

void MultiAgentPointCloudViewer::addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	m_viewers[msg->agentId]->addFrameMsg(msg);
	if (msg->agentId == 1)
	{
		m_viewers[0]->addFrameMsg(msg);
		// if (msg->isKeyframe)
		// 	if (msg->id > cutFirstNKf)
		// 		retrackOtherAgents(msg->id);
		return;
	}
	
	if (msg->isKeyframe)
	{
		//printf("Is KF!\n");
		PointCloudViewer* ref_vwr = m_viewers.at(msg->agentId);
		
		if (ref_vwr->getKFCount() <= cutFirstNKf) return;
		
		//for (auto ref_kf_kv : ref_vwr->getKFGraphDisplay()->getKeyFramesByID())
		{
			const KeyFrameDisplay& ref_kf = /*ref_kf_kv.second;*/ref_vwr->getKFGraphDisplay()->getKeyFramesByID().at(msg->id);
			
			for (int i = 1; i < (int)m_viewers.size(); ++i)
			{
				double minDist;
				double maxAngleCos;
				int equal_kf_id = m_viewers[i]->findEqualKF(ref_kf, minDist, maxAngleCos);
				
				if (equal_kf_id > 0)
				{
					printf("agent #%d: found equal KF #%d, ref KF #%d, ref agentID #%d\n", msg->agentId, equal_kf_id, ref_kf.id, ref_kf.getAgentId());
					const KeyFrameDisplay& pivot_kf = m_viewers[0]->getKFGraphDisplay()->getKeyFramesByID().at(equal_kf_id);
					
					bool fDist = false;
					bool fAngle = false;
					
					if (minDist < 0.1 && maxAngleCos > 0.98)
					{
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
							Sophus::Sim3f alignTransform = pivot_kf.camToWorld * ref_kf.camToWorld.inverse();
							//Sophus::Sim3f alignTransform = (ref_kf.camToWorld.inverse() * pivot_kf.camToWorld).inverse()
							//alignTransform.setScale(1);
							ref_vwr->setAlignTransform(alignTransform * ref_vwr->getAlignTransform());
							
							m_equal_kfs[0][msg->agentId][equal_kf_id].push_back(ref_kf.id);
						}
						
						//Check align
						Sophus::Sim3f alignedRefKF = ref_vwr->getAlignTransform() * ref_kf.camToWorld;
						Sophus::Sim3f residual = pivot_kf.camToWorld * alignedRefKF.inverse();
						double logNorm = residual.log().norm();
						printf("Residual log norm: %f\n", logNorm);
						
						float avgResidual;
						float maxResidual;
						countLogNormResidual(0, equal_kf_id, ref_kf.getAgentId(), ref_kf.id, avgResidual, maxResidual);
						
						if (maxResidual > 0.1 || avgResidual > 0.01)
						{
							printf("Adjusting ref ctw\n");
							ref_vwr->setAlignTransform(residual * ref_vwr->getAlignTransform());
							//m_viewers[0]->getKFGraphDisplay()->setCamToWorld(ref_kf.id, ref_vwr->getAlignTransform().inverse() * pivot_kf.camToWorld);
						}
						
						m_viewers[0]->getKFGraphDisplay()->addGraph(equal_kf_id, ref_kf.id, m_viewers[msg->agentId]->getKFGraphDisplay());
						m_viewers[0]->resetAnimation(0, cutFirstNKf);
					}
				}
			}
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