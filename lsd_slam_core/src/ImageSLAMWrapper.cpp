#include "ImageSLAMWrapper.h"
#include "SlamSystem.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "GlobalMapping/GlobalKeyFrameGraph.h"
#include "util/Undistorter.h"
#include "IOWrapper/ROS/ROSOutput3DWrapper.h"

#include "opencv2/opencv.hpp"

namespace lsd_slam
{
    SlamSystemWrapper::SlamSystemWrapper()
        :
        m_undistorter(nullptr),
        m_outputWrapper(nullptr),
        m_system(nullptr),
        m_runningIDX(0),
        m_fakeTimeStamp(0),
        m_finalized(false),
        m_hz(0)
    {}

    SlamSystemWrapper::~SlamSystemWrapper()
    {
        delete m_system;
        delete m_outputWrapper;
        delete m_undistorter;
    }

    //undisorter pointer is managed by this class
    void SlamSystemWrapper::init(const int agentId, const int idOffset, const Undistorter* undistorter, const int hz)
    {
        m_hz = hz;
        m_undistorter = undistorter;
        m_agentId = agentId;
        m_idOffset = idOffset;
        
        int w = undistorter->getOutputWidth();
        int h = undistorter->getOutputHeight();
        
        m_outputWrapper = new ROSOutput3DWrapper(w, h);
        
        float fx = undistorter->getK().at<double>(0, 0);
        float fy = undistorter->getK().at<double>(1, 1);
        float cx = undistorter->getK().at<double>(2, 0);
        float cy = undistorter->getK().at<double>(2, 1);
        
        m_K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
        
        m_system = new SlamSystem(w, h, m_K, agentId, doSlam);
        m_system->setVisualization(m_outputWrapper);
    }
        
    void SlamSystemWrapper::setImages(const std::string& folder, const std::vector<std::string>& files)
    {
        m_folder = folder;
        m_imageFiles = files;
    }
        
    bool SlamSystemWrapper::processNextImage()
    {
        if (!m_imageFiles.size())
            return false;
        
        if (m_runningIDX >= (int)m_imageFiles.size())
        {
            if (!m_finalized)
            {
                m_system->finalize();
                m_finalized = true;
            }
            return false;
        }
        
        int w = m_undistorter->getOutputWidth();
        int h = m_undistorter->getOutputHeight();
        
        cv::Mat imageDist = cv::imread(m_imageFiles[m_runningIDX], CV_LOAD_IMAGE_GRAYSCALE);

        if (imageDist.rows != m_undistorter->getInputHeight() || imageDist.cols != m_undistorter->getInputWidth())
        {
            if (imageDist.rows * imageDist.cols == 0)
                printf("failed to load image %s! skipping.\n", m_imageFiles[m_runningIDX].c_str());
            else
                printf("image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
                        m_imageFiles[m_runningIDX].c_str(),
                        w,h,imageDist.cols, imageDist.rows);
            return true;
        }
        assert(imageDist.type() == CV_8U);

        cv::Mat image = cv::Mat(h, w, CV_8U);
        m_undistorter->undistort(imageDist, image);
        assert(image.type() == CV_8U);

        if (m_runningIDX == 0)
            m_system->randomInit(image.data, m_fakeTimeStamp, m_runningIDX + m_idOffset);
        else
            m_system->trackFrame(image.data, m_runningIDX + m_idOffset, m_hz == 0, m_fakeTimeStamp);
        ++m_runningIDX;
        m_fakeTimeStamp += 0.03;
        
        return true;
    }
    
    void SlamSystemWrapper::reset()
    {
        m_runningIDX = 0;
        m_fakeTimeStamp = 0;
        delete m_system;
        //TODO: global graph. Remove frames with corresponding agent id?
        m_system = new SlamSystem(m_undistorter->getOutputWidth(), m_undistorter->getOutputHeight(), m_K, m_agentId, doSlam);
        m_system->setVisualization(m_outputWrapper);
    }
}