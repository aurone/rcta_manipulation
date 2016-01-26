#include <string>
#include <ros/ros.h>
#include <pr2_vfh_database/VFHPoseEstimationTrainer.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hdt_train_object_pose_estimation");
    ros::NodeHandle nh;

    ros::NodeHandle ph("~");

    std::string database_file;
    if (!ph.getParam("database_filename", database_file)) {
        ROS_ERROR("Failed to retrieve 'database_file' param");
        return 1;
    }

    VFHPoseEstimationTrainer trainer;

    if (!trainer.initialize(database_file)) {
        return 2;   
    }

    const std::string features_filename = "training_features.h5";
    const std::string kdtree_indices_filename = "training_kdtree.idx";
    if (!trainer.train(features_filename, kdtree_indices_filename)) {
        return 3;
    }

    return 0;
}
