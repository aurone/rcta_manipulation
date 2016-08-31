// standard includes
#include <string>

// system includes
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

/// \brief Provides a node that listens to gazebo model states and publishes
///     pose information and tf for a model
class GroundTruthPublisher
{
public:

    GroundTruthPublisher();

    int run();

private:

    ros::NodeHandle m_nh;
    ros::NodeHandle m_ph;
    ros::Publisher m_pose_pub;
    ros::Subscriber m_model_states_sub;

    std::string m_model_name;
    ros::Duration m_publish_duration;
    std::string m_map_frame;
    std::string m_odom_frame;
    std::string m_base_frame;
    ros::Time m_last_publish_time;
    tf::TransformBroadcaster m_broadcaster;
    tf::TransformListener m_listener;

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
};

GroundTruthPublisher::GroundTruthPublisher() :
    m_ph("~"),
    m_last_publish_time(0, 0)
{
    m_pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
    m_model_states_sub = m_nh.subscribe("gazebo/model_states", 1, &GroundTruthPublisher::modelStatesCallback, this);
}

int GroundTruthPublisher::run()
{
    if (!m_ph.hasParam("model_name")) {
        return 1;
    }
    m_ph.getParam("model_name", m_model_name);

    double publish_rate;
    m_ph.param("publish_rate", publish_rate, 50.0);

    m_publish_duration = ros::Duration(1.0 / publish_rate);

    m_ph.param<std::string>("map_frame", m_map_frame, "map");
    m_ph.param<std::string>("odom_frame", m_odom_frame, "odom");
    m_ph.param<std::string>("base_frame", m_base_frame, "base_link");

    ROS_INFO("model name: %s", m_model_name.c_str());
    ROS_INFO("map frame: %s", m_map_frame.c_str());
    ROS_INFO("odom frame: %s", m_odom_frame.c_str());
    ROS_INFO("base frame: %s", m_base_frame.c_str());
    ROS_INFO("publish duration: %0.3f", m_publish_duration.toSec());

    ros::spin();
    return 0;
}

void GroundTruthPublisher::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    auto nit = std::find(msg->name.begin(), msg->name.end(), m_model_name);
    if (nit == msg->name.end()) {
        return;
    }

    size_t midx = std::distance(msg->name.begin(), nit);

    const geometry_msgs::Pose& pose = msg->pose[midx];

    ros::Time deadline = m_last_publish_time + m_publish_duration;
    ros::Time now = ros::Time::now();
    if (now > deadline) {
        geometry_msgs::PoseStamped p;
        p.header.frame_id = m_map_frame;
        p.header.stamp = now;
        p.pose = pose;
        m_pose_pub.publish(p);

        // look up the transform from odom to map
        Eigen::Affine3d T_odom_base(Eigen::Affine3d::Identity());
        if (m_odom_frame != m_base_frame) {
            if (!m_listener.waitForTransform(m_odom_frame, m_base_frame, now, m_publish_duration)) {
                return;
            }
            try {
                tf::StampedTransform tin_stamped;
                m_listener.lookupTransform(m_odom_frame, m_base_frame, now, tin_stamped);
                tf::transformTFToEigen(tin_stamped, T_odom_base);
            } catch (const tf::TransformException& ex) {
                ROS_ERROR("failed to lookup transform (%s)", ex.what());
                return; // don't publish tf
            }
        }

        Eigen::Affine3d T_map_base;
        tf::poseMsgToEigen(pose, T_map_base);

        Eigen::Affine3d T_map_odom = T_map_base * T_odom_base.inverse();
        tf::Transform tout;
        tf::transformEigenToTF(T_map_odom, tout);
        tf::StampedTransform tout_stamped(tout, now, m_map_frame, m_odom_frame);

        m_broadcaster.sendTransform(tout_stamped);
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ground_truth_publisher");
    GroundTruthPublisher g;
    return g.run();
}
