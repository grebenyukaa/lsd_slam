#pragma once

#include <vector>
#include <string>
#include "util/SophusUtil.h"

namespace cv
{
    class Mat;
}

namespace lsd_slam
{
    class Output3DWrapper;
    class SlamSystem;
    class Undistorter;
    
    class SlamSystemWrapper
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        SlamSystemWrapper();
        ~SlamSystemWrapper();
        
        //undisorter pointer is managed by this class
        void init(const int agentId, const int idOffset, const Undistorter* undistorter, const double hz);
        void setImages(const std::string& folder, const std::vector<std::string>& files);
        bool processNextImage();
        void reset();
        
    private:
        const Undistorter* m_undistorter;
        Sophus::Matrix3f m_K;
        Output3DWrapper* m_outputWrapper;
        SlamSystem* m_system;
        std::vector<std::string> m_imageFiles;
        std::string m_folder;
        
        int m_runningIDX;
        float m_fakeTimeStamp;
        bool m_finalized;
        double m_hz;
        int m_agentId;
        int m_idOffset;
    };
}