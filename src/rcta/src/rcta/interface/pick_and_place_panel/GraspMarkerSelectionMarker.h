#ifndef SelectableInteractiveMarker_h
#define SelectableInteractiveMarker_h

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <ros/ros.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>

#include "SimpleInteractiveMarker.h"

class GraspMarkerSelection;

class GraspMarkerSelectionMarker : public SimpleInteractiveMarker
{
public:

    GraspMarkerSelectionMarker(
        SimpleInteractiveMarkerServer &server,
        const visualization_msgs::InteractiveMarker &int_marker,
        const std::shared_ptr<GraspMarkerSelection> &grasp_selection);

    virtual ~GraspMarkerSelectionMarker();

    virtual void on_button_click(const Feedback &feedback);

    bool is_selected_pregrasp() const;
    bool is_selected_grasp() const;

private:

    std::shared_ptr<GraspMarkerSelection> grasp_selection_;

    bool is_grasp_marker() const;
    bool is_pregrasp_marker() const;

    static void invert_color(std_msgs::ColorRGBA &color);
};

#endif
