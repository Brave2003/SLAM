#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"

DEFINE_string(config_file, "/home/brave/Desktop/SLAM/config/default.yaml", "config file path");

int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    myslam::VisualOdometry::Ptr vo(
        new myslam::VisualOdometry(FLAGS_config_file));
    vo->Init();
    vo->Run();

    return 0;
}

