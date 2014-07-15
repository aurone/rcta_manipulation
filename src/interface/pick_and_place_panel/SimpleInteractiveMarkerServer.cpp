#include "SimpleInteractiveMarkerServer.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <ros/ros.h>
#include "SimpleInteractiveMarker.h"

SimpleInteractiveMarkerServer::SimpleInteractiveMarkerServer(const std::string &name) :
    server_(name)
{

}

void SimpleInteractiveMarkerServer::insert(const std::shared_ptr<SimpleInteractiveMarker> &sim)
{
    ROS_INFO("Inserting marker '%s'", sim->name().c_str());
    markers_.insert(std::make_pair(sim->name(), sim));
    server_.insert(sim->interactive_marker(),
            std::bind(&SimpleInteractiveMarkerServer::process_feedback, this, std::placeholders::_1));
}

SimpleInteractiveMarkerPtr SimpleInteractiveMarkerServer::get(const std::string &marker)
{
    auto mit = markers_.find(marker);
    if (mit == markers_.end()) {
        return std::shared_ptr<SimpleInteractiveMarker>();
    }
    else {
        return mit->second;
    }
}

void SimpleInteractiveMarkerServer::update(
    const std::string &marker,
    const visualization_msgs::InteractiveMarker &int_marker)
{
    auto mit = markers_.find(marker);
    if (mit != markers_.end()) {
        ROS_INFO("Updating marker '%s'", marker.c_str());
        mit->second->interactive_marker(int_marker);
        server_.insert(int_marker);
    }
    else {
        ROS_WARN("Can not update non-existant marker '%s'", marker.c_str());
    }
}

void SimpleInteractiveMarkerServer::remove(const std::string &marker)
{
    if (!server_.erase(marker)) {
        ROS_WARN("No marker named '%s' to remove from the server", marker.c_str());
    }
    else {
        ROS_INFO("Erased marker '%s'", marker.c_str());
    }
}

void SimpleInteractiveMarkerServer::clear()
{
    ROS_INFO("Clearing all interactive markers");
    markers_.clear();
    server_.clear();
}

void SimpleInteractiveMarkerServer::flush()
{
    ROS_INFO("Flushing changes to interactive marker server");
    server_.applyChanges();
}

void SimpleInteractiveMarkerServer::print_interactive_marker_feedback(
    const visualization_msgs::InteractiveMarkerFeedback& feedback_msg) const
{
    ROS_INFO("Processing feedback for interactive marker %s", feedback_msg.marker_name.c_str());
    ROS_INFO("  header:");
    ROS_INFO("    seq: %u", feedback_msg.header.seq);
    ROS_INFO("    stamp: %s", boost::posix_time::to_simple_string(feedback_msg.header.stamp.toBoost()).c_str());
    ROS_INFO("    frame_id: %s", feedback_msg.header.frame_id.c_str());
    ROS_INFO("  client_id: %s", feedback_msg.client_id.c_str());
    ROS_INFO("  marker_name: %s", feedback_msg.marker_name.c_str());
    ROS_INFO("  control_name: %s", feedback_msg.control_name.c_str());
    ROS_INFO("  event_type: %d", (int)feedback_msg.event_type);
    ROS_INFO("  pose:");
    ROS_INFO("    position:");
    ROS_INFO("      x: %0.3f", feedback_msg.pose.position.x);
    ROS_INFO("      y: %0.3f", feedback_msg.pose.position.y);
    ROS_INFO("      z: %0.3f", feedback_msg.pose.position.z);
    ROS_INFO("    orientation:");
    ROS_INFO("      w: %0.3f", feedback_msg.pose.orientation.w);
    ROS_INFO("      x: %0.3f", feedback_msg.pose.orientation.x);
    ROS_INFO("      y: %0.3f", feedback_msg.pose.orientation.y);
    ROS_INFO("      z: %0.3f", feedback_msg.pose.orientation.z);
    ROS_INFO("  menu_entry_id: %u", feedback_msg.menu_entry_id);
    ROS_INFO("  mouse_point:");
    ROS_INFO("    x:", feedback_msg.mouse_point.x);
    ROS_INFO("    y:", feedback_msg.mouse_point.y);
    ROS_INFO("    z:", feedback_msg.mouse_point.z);
    ROS_INFO("  mouse_point_valid: %s", feedback_msg.mouse_point_valid ? "true" : "false");
}

void SimpleInteractiveMarkerServer::process_feedback(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr &feedback)
{
    if (!feedback) {
        return;
    }

    auto mit = markers_.find(feedback->marker_name);
    if (mit == markers_.end()) {
        ROS_WARN("Received feedback for marker %s not handled by this simple interactive marker server", feedback->marker_name.c_str());
        return;
    }

    SimpleInteractiveMarker::Feedback f;
    f.header = feedback->header;
    f.client_id = feedback->client_id;
    f.control_name = feedback->control_name;

    SimpleInteractiveMarkerPtr marker = mit->second;
    switch (feedback->event_type)
    {
    case visualization_msgs::InteractiveMarkerFeedback::KEEP_ALIVE:
        marker->on_keep_alive(f);
        break;
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        marker->on_pose_update(f, feedback->pose);
        break;
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
        marker->on_menu_select(f, feedback->menu_entry_id);
        break;
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        marker->on_button_click(f);
        break;
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        marker->on_mouse_down();
        break;
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        marker->on_mouse_up();
        break;
    default:
        ROS_WARN("Unrecognized InteractiveMarkerFeedback event type");
        return;
    }

    flush();
    fb_cb_();
}
