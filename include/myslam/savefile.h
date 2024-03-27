#ifndef MYSLAM_SAVEFILE_H
#define MYSLAM_SAVEFILE_H

#include "myslam/common_include.h"


namespace myslam{
    class SaveFile{
        public:
            static bool SE32TUM(const std::string &filepath, const std::string &filename, const std::vector<Sophus::SE3d>& poses, const std::vector<double>& timestamps);

            static std::vector<double> ReadTimeStampsFile(const std::string &filepath);

        private:
            SaveFile() = delete;
    }
}

#endif