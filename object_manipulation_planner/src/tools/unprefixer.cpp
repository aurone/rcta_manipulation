#include <memory>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>

std::unique_ptr<tf::TransformBroadcaster> g_broadcaster;
ros::Publisher g_static_pub;

std::vector<std::pair<std::string, std::string>> g_seen;

void LogStuff(const std::string& a, const std::string& b)
{
    auto p = std::make_pair(a, b);
    if (std::find(begin(g_seen), end(g_seen), p) == end(g_seen)) {
        ROS_INFO("New transform %s -> %s", a.c_str(), b.c_str());
        g_seen.push_back(p);
    }
}

bool substitute(std::string& s, const std::string& a, const std::string& b)
{
    // WARNING: don't pass a = b or the case where b substring of a
    auto pos = std::string::npos;
    auto changed = false;
    while ((pos = s.find(a)) != std::string::npos) {
        s.replace(pos, a.size(), b);
        changed = true;
    }
    return changed;
}

void TransformCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    // for all frames in the system
    //   if the frame has a prefix
    //     publish an identity transform from the prefixed link to a fictional link without the prefix
    for (auto& transform : msg->transforms) {
        auto sub = transform.child_frame_id;
        if (substitute(sub, "roman4/", "")) {
            // bad...dealing with map -> /roman4/map?
            if (sub.find("map") != std::string::npos) continue;

            tf::StampedTransform tf;
            tf.frame_id_ = transform.child_frame_id;
            tf.child_frame_id_ = sub;
            tf.stamp_ = transform.header.stamp;
            LogStuff(tf.frame_id_, tf.child_frame_id_);
            tf.setIdentity();
            ROS_DEBUG("Broadcast transform from %s to %s", tf.frame_id_.c_str(), tf.child_frame_id_.c_str());
            g_broadcaster->sendTransform(tf);
        }
    }
}

void StaticTransformCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    // for all frames in the system
    //   if the frame has a prefix
    //     publish an identity transform from the prefixed link to a fictional link without the prefix
    tf2_msgs::TFMessage static_transforms;
    static_transforms.transforms.reserve(msg->transforms.size());

    for (auto& transform : msg->transforms) {
        auto sub = transform.child_frame_id;
        if (substitute(sub, "roman4/", "")) {
            // bad...dealing with map -> /roman4/map?
            if (sub.find("map") != std::string::npos) continue;

            geometry_msgs::TransformStamped tf;
            tf.header.frame_id = transform.child_frame_id;
            tf.header.stamp = transform.header.stamp;


            tf.child_frame_id = sub;
            tf.transform.rotation.w = 1.0;
            LogStuff(tf.header.frame_id, tf.child_frame_id);
            ROS_DEBUG("Add transform from %s to %s", tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
            static_transforms.transforms.push_back(tf);
        }
    }

    g_static_pub.publish(static_transforms);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "unprefixer");
    ros::NodeHandle nh;

    g_broadcaster.reset(new tf::TransformBroadcaster);

    auto sub = nh.subscribe("/tf", 10, TransformCallback);

    auto static_sub = nh.subscribe("/tf_static", 10, StaticTransformCallback);
    g_static_pub = nh.advertise<tf2_msgs::TFMessage>("/tf_static", 1, true);

    ros::spin();

    return 0;
}
