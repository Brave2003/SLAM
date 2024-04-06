#include <iostream>
#include <fstream>
#include <unistd.h>
#include "myslam/savefile.h"



namespace myslam
{   
    bool SaveFile::SE32TUM(const std::string &filepath, const std::string &filename, const std::vector<Sophus::SE3d> &poses, const std::vector<double> &timestamps)
    {
        // 检查文件是否存在
        std::string file_ = filepath + "/" + filename;
        std::ofstream file(file_);

        if (!file.is_open())
        {
            LOG(ERROR) << "Could not open the tum data file for writing: " << filename;
            return false;
        }

        for (size_t i = 0; i < poses.size(); ++i)
        {
            const Sophus::SE3d &pose = poses[i].inverse();
            const double timestamp = timestamps[i];

            // 提取旋转四元数和平移向量
            Eigen::Quaterniond quaternion(pose.rotationMatrix());
            Eigen::Vector3d translation = pose.translation();

            // 写入TUM格式的数据到文件
            file << timestamp << " "
                 << translation(0) << " "
                 << translation(1) << " "
                 << translation(2) << " "
                 << quaternion.x() << " "
                 << quaternion.y() << " "
                 << quaternion.z() << " "
                 << quaternion.w()
                 << std::endl;
        }

        file.close();
        LOG(INFO) << "Finish write trajectory file";
        return true;
    };

    std::vector<double> SaveFile::ReadTimeStampsFile(const std::string &filename)
    {
        std::ifstream file_(filename);         // 打开文件用于读取

        if (!file_.is_open())
        {
            LOG(ERROR) << "Could not open the timestamps file : " << filename << std::endl;
            return {}; // 返回错误代码
        }

        std::string line;
        std::vector<double> timeStamps={};
        while (std::getline(file_, line))
        { // 使用getline读取每一行
            // 在这里处理每一行的内容

            double value = std::stod(line);
            timeStamps.push_back(value);
        }

        file_.close(); // 关闭文件

        return timeStamps; // 程序正常结束
    };

    void SaveFile::SaveTrajectoryFile(const std::string& filepath, std::vector< Frame::Ptr> keyframes)
    {
        std::vector<Sophus::SE3d> poses;
        for(const auto& frame: keyframes){
            poses.push_back(frame->Pose());
        }

        if (SaveFile::SE32TUM(filepath,
                Config::Get<std::string>("trajectory_file_name"),
                poses,
                SaveFile::ReadTimeStampsFile(Config::Get<std::string>("dataset_dir") + "/" + "times.txt")))
        {
            LOG(INFO) << "Save trajectory file to : " << filepath;
        }
        else
            LOG(ERROR) << "Couldn't save trajectory file to: " << filepath;
    }
    void SaveFile::SaveTrajectoryFile(const std::string& filepath, std::vector< SE3> poses)
    {



        if (SaveFile::SE32TUM(filepath,
                              Config::Get<std::string>("trajectory_file_name"),
                              poses,
                              SaveFile::ReadTimeStampsFile(Config::Get<std::string>("dataset_dir") + "/" + "times.txt")))
        {
            LOG(INFO) << "Save trajectory file to : " << filepath;
        }
        else
            LOG(ERROR) << "Couldn't save trajectory file to: " << filepath;
    }
}
