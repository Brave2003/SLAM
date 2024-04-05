#ifndef MYSLAM_SAVEFILE_H
#define MYSLAM_SAVEFILE_H

#include "myslam/common_include.h"
#include "frame.h"
#include "myslam/config.h"

namespace myslam{
    class SaveFile{
        public:
            static bool SE32TUM(const std::string &filepath, const std::string &filename, const std::vector<Sophus::SE3d>& poses, const std::vector<double>& timestamps);

            static std::vector<double> ReadTimeStampsFile(const std::string &filepath);

            static void SaveTrajectoryFile(const std::string& filepath, std::vector< Frame::Ptr> poses);
            static void SaveTrajectoryFile(const std::string& filepath, std::vector< SE3> poses);

        private:
            SaveFile() = delete;
    };
};

#endif