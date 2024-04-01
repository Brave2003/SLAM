#include <iostream>
#include <fstream>
#include <unistd.h>
#include "myslam/common_include.h"
#include "myslam/savefile.h"


namespace myslam
{   
    bool SaveFile::SE32TUM(const std::string &filepath, const std::string &filename, const std::vector<Sophus::SE3d> &poses, const std::vector<double> &timestamps)
    {
        // 检查文件是否存在
        std::string file_ = filepath + "/" + filename;
        std::ofstream file(file_, std::ofstream::out);

        if (!file.is_open())
        {
            LOG(ERROR) << "Could not open the tum data file for writing: " << filename;
            return false;
        }

        for (size_t i = 0; i < poses.size(); ++i)
        {
            const Sophus::SE3d &pose = poses[i];
            const double timestamp = timestamps[i];

            // 提取旋转四元数和平移向量
            Eigen::Quaterniond quaternion(pose.rotationMatrix());
            Eigen::Vector3d translation = pose.translation();

            // 写入TUM格式的数据到文件
            file << timestamp << " " << quaternion.w() << " " << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << " "
                 << translation(0) << " " << translation(1) << " " << translation(2) << std::endl;
        }

        file.close();
        return true;
    };

    std::vector<double> SaveFile::ReadTimeStampsFile(const std::string &filepath)
    {
        std::ifstream file(filepath+"/"+"times.txt");         // 打开文件用于读取

        if (!file.is_open())
        {
            LOG(ERROR) << "Could not open the timestamps file : " << filepath << std::endl;
            return {}; // 返回错误代码
        }

        std::string line;
        std::vector<double> timeStamps={};
        while (std::getline(file, line))
        { // 使用getline读取每一行
            // 在这里处理每一行的内容

            double value = std::stod(line);
            timeStamps.push_back(value);
        }

        file.close(); // 关闭文件

        return timeStamps; // 程序正常结束
    };
}
