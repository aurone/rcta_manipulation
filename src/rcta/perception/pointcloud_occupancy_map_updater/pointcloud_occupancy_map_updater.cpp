/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Jon Binney, Ioan Sucan */

#include <cmath>
#include <cstdint>
#include <chrono>

#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <XmlRpcException.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>
#include <moveit/point_containment_filter/shape_mask.h>

namespace occupancy_map_monitor {

class PointCloudOccupancyMapUpdater : public OccupancyMapUpdater {
public:

    PointCloudOccupancyMapUpdater();
    virtual ~PointCloudOccupancyMapUpdater();

    bool setParams(XmlRpc::XmlRpcValue& params) override;

    bool initialize() override;
    void start() override;
    void stop() override;
    ShapeHandle excludeShape(const shapes::ShapeConstPtr& shape) override;
    void forgetShape(ShapeHandle handle) override;

private:

    ros::NodeHandle root_nh_;
    ros::NodeHandle private_nh_;
    boost::shared_ptr<tf::Transformer> tf_;

    /* params */
    std::string point_cloud_topic_;
    double scale_;
    double padding_;
    double max_range_;
    unsigned int point_subsample_;
    std::string filtered_cloud_topic_;
    ros::Publisher filtered_cloud_publisher_;

    message_filters::Subscriber<sensor_msgs::PointCloud2>* point_cloud_subscriber_;
    tf::MessageFilter<sensor_msgs::PointCloud2>* point_cloud_filter_;

    // used to store all cells in the map which a given ray passes through during raycasting.
    // we cache this here because it dynamically pre-allocates a lot of memory in its contsructor
    octomap::KeyRay key_ray_;

    boost::scoped_ptr<point_containment_filter::ShapeMask> shape_mask_;
    std::vector<int> mask_;

    void updateMask(
        const sensor_msgs::PointCloud2& cloud,
        const Eigen::Vector3d& sensor_origin,
        std::vector<int>& mask);

    bool getShapeTransform(ShapeHandle h, Eigen::Affine3d& transform) const;
    void cloudMsgCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    void stopHelper();

    bool updateTransformCache(
        const std::string& target_frame,
        const ros::Time& target_time);
};

PointCloudOccupancyMapUpdater::PointCloudOccupancyMapUpdater()
    : OccupancyMapUpdater("PointCloudUpdater")
    , private_nh_("~")
    , scale_(1.0)
    , padding_(0.0)
    , max_range_(std::numeric_limits<double>::infinity())
    , point_subsample_(1)
    , point_cloud_subscriber_(NULL)
    , point_cloud_filter_(NULL)
{
}

PointCloudOccupancyMapUpdater::~PointCloudOccupancyMapUpdater()
{
    stopHelper();
}

bool PointCloudOccupancyMapUpdater::setParams(XmlRpc::XmlRpcValue& params) {
    try {
        if (!params.hasMember("point_cloud_topic")) {
            return false;
        }
        point_cloud_topic_ = static_cast<const std::string&>(params["point_cloud_topic"]);

        readXmlParam(params, "max_range", &max_range_);
        readXmlParam(params, "padding_offset", &padding_);
        readXmlParam(params, "padding_scale", &scale_);
        readXmlParam(params, "point_subsample", &point_subsample_);
        if (params.hasMember("filtered_cloud_topic")) {
            filtered_cloud_topic_ = static_cast<const std::string&>(params["filtered_cloud_topic"]);
        }
    } catch (XmlRpc::XmlRpcException& ex) {
        ROS_ERROR("XmlRpc Exception: %s", ex.getMessage().c_str());
        return false;
    }

    return true;
}

bool PointCloudOccupancyMapUpdater::initialize() {
    tf_ = monitor_->getTFClient();
    shape_mask_.reset(new point_containment_filter::ShapeMask());
    shape_mask_->setTransformCallback(boost::bind(&PointCloudOccupancyMapUpdater::getShapeTransform, this, _1, _2));
    if (!filtered_cloud_topic_.empty()) {
        filtered_cloud_publisher_ = private_nh_.advertise<sensor_msgs::PointCloud2>(filtered_cloud_topic_, 10, false);
    }
    return true;
}

void PointCloudOccupancyMapUpdater::start() {
    if (point_cloud_subscriber_) {
        return;
    }
    /* subscribe to point cloud topic using tf filter*/
    point_cloud_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(root_nh_, point_cloud_topic_, 5);
    if (tf_ && !monitor_->getMapFrame().empty()) {
        point_cloud_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(*point_cloud_subscriber_, *tf_, monitor_->getMapFrame(), 5);
        point_cloud_filter_->registerCallback(boost::bind(&PointCloudOccupancyMapUpdater::cloudMsgCallback, this, _1));
        ROS_INFO("Listening to '%s' using message filter with target frame '%s'", point_cloud_topic_.c_str(), point_cloud_filter_->getTargetFramesString().c_str());
    } else {
        point_cloud_subscriber_->registerCallback(boost::bind(&PointCloudOccupancyMapUpdater::cloudMsgCallback, this, _1));
        ROS_INFO("Listening to '%s'", point_cloud_topic_.c_str());
    }
}

void PointCloudOccupancyMapUpdater::stopHelper() {
    delete point_cloud_filter_;
    delete point_cloud_subscriber_;
}

void PointCloudOccupancyMapUpdater::stop() {
    stopHelper();
    point_cloud_filter_ = NULL;
    point_cloud_subscriber_ = NULL;
}

ShapeHandle PointCloudOccupancyMapUpdater::excludeShape(
    const shapes::ShapeConstPtr& shape)
{
    ShapeHandle h = 0;
    if (shape_mask_) {
        h = shape_mask_->addShape(shape, scale_, padding_);
    } else {
        ROS_ERROR("Shape filter not yet initialized!");
    }
    return h;
}

void PointCloudOccupancyMapUpdater::forgetShape(ShapeHandle handle) {
    if (shape_mask_) {
        shape_mask_->removeShape(handle);
    }
}

bool PointCloudOccupancyMapUpdater::getShapeTransform(
    ShapeHandle h,
    Eigen::Affine3d& transform) const
{
    auto it = transform_cache_.find(h);
    if (it == transform_cache_.end()) {
        ROS_ERROR("Internal error. Shape filter handle %u not found", h);
        return false;
    }
    transform = it->second;
    return true;
}

void PointCloudOccupancyMapUpdater::updateMask(
    const sensor_msgs::PointCloud2& cloud,
    const Eigen::Vector3d& sensor_origin,
    std::vector<int>& mask)
{
}

void PointCloudOccupancyMapUpdater::cloudMsgCallback(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    ROS_DEBUG("Received a new point cloud message");
    ros::WallTime start = ros::WallTime::now();

    if (monitor_->getMapFrame().empty()) {
        monitor_->setMapFrame(cloud_msg->header.frame_id);
    }

    using namespace std::chrono;
    using display_time_unit = duration<double, std::milli>;

    time_point<high_resolution_clock> before, after;

    before = high_resolution_clock::now();
    // get transform for cloud into map frame
    tf::StampedTransform map_H_sensor;
    if (monitor_->getMapFrame() == cloud_msg->header.frame_id) {
        map_H_sensor.setIdentity();
    } else {
        if (tf_) {
            try {
                tf_->lookupTransform(monitor_->getMapFrame(), cloud_msg->header.frame_id, cloud_msg->header.stamp, map_H_sensor);
            } catch (tf::TransformException& ex) {
                ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << "; quitting callback");
                return;
            }
        } else {
            return;
        }
    }
    after = high_resolution_clock::now();
    ROS_DEBUG("  Lookup cloud transform took %lf seconds", display_time_unit(after - before).count());

    // compute sensor origin in map frame
    const tf::Vector3& sensor_origin_tf = map_H_sensor.getOrigin();
    octomap::point3d sensor_origin(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());
    Eigen::Vector3d sensor_origin_eigen(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());

    before = high_resolution_clock::now();
    if (!updateTransformCache(cloud_msg->header.frame_id, cloud_msg->header.stamp)) {
        ROS_ERROR_THROTTLE(1, "Transform cache was not updated. Self-filtering may fail.");
        return;
    }
    after = high_resolution_clock::now();
    ROS_DEBUG("  Update transform cache took %lf seconds", display_time_unit(after - before).count());

    before = high_resolution_clock::now();
    // mask out points on the robot
    shape_mask_->maskContainment(*cloud_msg, sensor_origin_eigen, 0.0, max_range_, mask_);
    updateMask(*cloud_msg, sensor_origin_eigen, mask_);
    after = high_resolution_clock::now();
    ROS_DEBUG("  Update mask took %lf seconds", display_time_unit(after - before).count());

    octomap::KeySet free_cells, occupied_cells, model_cells, clip_cells;
    boost::scoped_ptr<sensor_msgs::PointCloud2> filtered_cloud;

    // We only use these iterators if we are creating a filtered_cloud for
    // publishing. We cannot default construct these, so we use scoped_ptr's
    // to defer construction
    boost::scoped_ptr<sensor_msgs::PointCloud2Iterator<float> > iter_filtered_x;
    boost::scoped_ptr<sensor_msgs::PointCloud2Iterator<float> > iter_filtered_y;
    boost::scoped_ptr<sensor_msgs::PointCloud2Iterator<float> > iter_filtered_z;

    if (!filtered_cloud_topic_.empty()) {
        filtered_cloud.reset(new sensor_msgs::PointCloud2());
        filtered_cloud->header = cloud_msg->header;
        sensor_msgs::PointCloud2Modifier pcd_modifier(*filtered_cloud);
        pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
        pcd_modifier.resize(cloud_msg->width * cloud_msg->height);

        // we have created a filtered_out, so we can create the iterators now
        iter_filtered_x.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "x"));
        iter_filtered_y.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "y"));
        iter_filtered_z.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "z"));
    }
    size_t filtered_cloud_size = 0;

    before = high_resolution_clock::now();
    tree_->lockRead();
    after = high_resolution_clock::now();
    ROS_DEBUG("  Read-locking took %lf seconds", display_time_unit(after - before).count());

   try {
        before = high_resolution_clock::now();
        // do ray tracing to find which cells this point cloud indicates should
        // be free, and which it indicates should be occupied
        for (std::uint32_t row = 0; row < cloud_msg->height; row += point_subsample_) {
            std::uint32_t row_c = row * cloud_msg->width;
            sensor_msgs::PointCloud2ConstIterator<float> pt_iter(*cloud_msg, "x");
            // set iterator to point at start of the current row
            pt_iter += row_c;

            for (std::uint32_t col = 0;
                 col < cloud_msg->width;
                 col += point_subsample_, pt_iter += point_subsample_)
            {
                // if (mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP)
                //  continue;

                // check for NaN
                if (isnan(pt_iter[0]) || isnan(pt_iter[1]) || isnan(pt_iter[2])) {
                    continue;
                }

                // transform to map frame
                tf::Vector3 point_tf = map_H_sensor * tf::Vector3(pt_iter[0], pt_iter[1], pt_iter[2]);

                // occupied cell at ray endpoint if ray is shorter than max range and this point isn't on a part of the robot
                if (mask_[row_c + col] == point_containment_filter::ShapeMask::INSIDE) {
                    model_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
                } else if (mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP) {
                    clip_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
                } else {
                    occupied_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
                    // build list of valid points if we want to publish them
                    if (filtered_cloud) {
                        **iter_filtered_x = pt_iter[0];
                        **iter_filtered_y = pt_iter[1];
                        **iter_filtered_z = pt_iter[2];
                        ++filtered_cloud_size;
                        ++*iter_filtered_x;
                        ++*iter_filtered_y;
                        ++*iter_filtered_z;
                    }
                }
            }
        }
        after = high_resolution_clock::now();
        ROS_DEBUG("  Ray tracing took %lf seconds", display_time_unit(after - before).count());

        before = high_resolution_clock::now();
        // compute the free cells along each ray that ends at an occupied cell
        for (auto it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it) {
            if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_)) {
                free_cells.insert(key_ray_.begin(), key_ray_.end());
            }
        }

        // compute the free cells along each ray that ends at a model cell
        for (auto it = model_cells.begin(), end = model_cells.end(); it != end; ++it) {
            if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_)) {
                free_cells.insert(key_ray_.begin(), key_ray_.end());
            }
        }

        // compute the free cells along each ray that ends at a clipped cell
        for (auto it = clip_cells.begin(), end = clip_cells.end(); it != end; ++it) {
            if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_)) {
                free_cells.insert(key_ray_.begin(), key_ray_.end());
            }
        }
        after = high_resolution_clock::now();
        ROS_DEBUG("  Gathering cell types took %lf seconds", display_time_unit(after - before).count());
    } catch (...) {
        tree_->unlockRead();
        return;
    }

    tree_->unlockRead();

    before = high_resolution_clock::now();
    // cells that overlap with the model are not occupied
    for (auto it = model_cells.begin(), end = model_cells.end(); it != end; ++it) {
        occupied_cells.erase(*it);
    }
    after = high_resolution_clock::now();
    ROS_DEBUG("  Filtering model cells took %lf seconds", display_time_unit(after - before).count());

    // occupied cells are not free
    before = high_resolution_clock::now();
    for (auto it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it) {
        free_cells.erase(*it);
    }
    after = high_resolution_clock::now();
    ROS_DEBUG("  Reinsert obstacle cells took %lf seconds", display_time_unit(after - before).count());

    before = high_resolution_clock::now();
    tree_->lockWrite();
    after = high_resolution_clock::now();
    ROS_DEBUG("  Write-locking took %lf seconds", display_time_unit(after - before).count());

    before = high_resolution_clock::now();
    try {
        // mark free cells only if not seen occupied in this cloud
        for (auto it = free_cells.begin(), end = free_cells.end(); it != end; ++it) {
            tree_->updateNode(*it, false);
        }

        // now mark all occupied cells
        for (auto it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it) {
            tree_->updateNode(*it, true);
        }

        // set the logodds to the minimum for the cells that are part of the model
        const float lg = tree_->getClampingThresMinLog() - tree_->getClampingThresMaxLog();
        for (auto it = model_cells.begin(), end = model_cells.end(); it != end; ++it) {
            tree_->updateNode(*it, lg);
        }
    } catch (...) {
        ROS_ERROR("Internal error while updating octree");
    }
    after = high_resolution_clock::now();
    ROS_DEBUG("  Update of octree took %lf seconds", display_time_unit(after - before).count());

    tree_->unlockWrite();
    ROS_DEBUG("Processed point cloud in %lf ms", (ros::WallTime::now() - start).toSec() * 1000.0);
    tree_->triggerUpdateCallback();

    if (filtered_cloud) {
        sensor_msgs::PointCloud2Modifier pcd_modifier(*filtered_cloud);
        pcd_modifier.resize(filtered_cloud_size);
        filtered_cloud_publisher_.publish(*filtered_cloud);
    }
}

bool PointCloudOccupancyMapUpdater::updateTransformCache(
    const std::string& target_frame,
    const ros::Time& target_time)
{
    if (transform_provider_callback_) {
        if (!transform_provider_callback_(target_frame, target_time, transform_cache_)) {
            ROS_WARN_THROTTLE(1, "Failed to update all transforms to the target time");
        } else {
            ROS_INFO_THROTTLE(1, "Updated all transforms");
        }
    } else {
        ROS_WARN_THROTTLE(1, "No callback provided for updating the transform cache for octomap updaters");
    }
    return true;
}

}

#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(
        occupancy_map_monitor::PointCloudOccupancyMapUpdater,
        occupancy_map_monitor::OccupancyMapUpdater)
