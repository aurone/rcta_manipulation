#include <ros/ros.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <tf/message_filter.h>
#include <XmlRpc.h>

class OctomapOccupancyMapUpdater :
    public occupancy_map_monitor::OccupancyMapUpdater
{
public:

    std::string octomap_topic;
    ros::NodeHandle nh;

    std::unique_ptr<message_filters::Subscriber<octomap_msgs::Octomap>> octomap_sub;
    std::unique_ptr<tf::MessageFilter<octomap_msgs::Octomap>> octomap_filter;
    boost::shared_ptr<tf::Transformer> tf;

    OctomapOccupancyMapUpdater() :
        occupancy_map_monitor::OccupancyMapUpdater("OctomapOccupancyMapUpdater")
    { }

    bool setParams(XmlRpc::XmlRpcValue& params) override
    {
        try {
            if (!params.hasMember("octomap_topic")) {
                return false;
            }

            this->octomap_topic = (const std::string&)params["octomap_topic"];
        } catch (XmlRpc::XmlRpcException& ex) {
            ROS_ERROR("XmlRpc Exception: %s", ex.getMessage().c_str());
            return false;
        }

        return true;
    }

    bool initialize() override
    {
        this->tf = this->monitor_->getTFClient();
        if (!this->tf) {
            ROS_ERROR("TF Monitor is null");
            return false;
        }
        return true;
    }

    void start() override
    {
        this->octomap_sub.reset(
                new message_filters::Subscriber<octomap_msgs::Octomap>(
                        this->nh,
                        this->octomap_topic,
                        1));
        if (this->tf && !this->monitor_->getMapFrame().empty()) {
            this->octomap_filter.reset(new tf::MessageFilter<octomap_msgs::Octomap>(
                    *this->octomap_sub,
                    *this->tf,
                    this->monitor_->getMapFrame(),
                    5));
            this->octomap_filter->registerCallback(
                    boost::bind(
                            &OctomapOccupancyMapUpdater::OctomapCallback,
                            this,
                            _1));
        } else {
            this->octomap_sub->registerCallback(
                    boost::bind(
                            &OctomapOccupancyMapUpdater::OctomapCallback,
                            this,
                            _1));
        }
    }

    void stop() override
    {
        this->octomap_filter.reset();
        this->octomap_sub.reset();
    }

    auto excludeShape(const shapes::ShapeConstPtr& shape) ->
        occupancy_map_monitor::ShapeHandle override
    {
        return occupancy_map_monitor::ShapeHandle(0);
    }

    void forgetShape(occupancy_map_monitor::ShapeHandle handle) override
    {
    }

    void OctomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
    {
        auto* tree = octomap_msgs::binaryMsgToMap(*msg);
        if (!tree || !dynamic_cast<octomap::OcTree*>(tree)) {
            delete tree;
            return;
        }
        std::unique_ptr<octomap::OcTree> ot((octomap::OcTree*)tree);

        this->tree_->lockWrite();

        this->tree_->clear();
        for (auto it = ot->begin_leafs(); it != ot->end_leafs(); ++it) {
            octomap::OcTreeKey key;
            if (it.getDepth() != ot->getTreeDepth()) {
                key = it.getIndexKey();
            } else {
                key = it.getKey();
            }
            bool occupied = it->getOccupancy() >= ot->getOccupancyThres();
            bool lazy_eval = false;
            this->tree_->updateNode(key, occupied, lazy_eval);
        }

        this->tree_->unlockWrite();
        this->tree_->triggerUpdateCallback();
    }
};

#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(
    OctomapOccupancyMapUpdater, occupancy_map_monitor::OccupancyMapUpdater);

