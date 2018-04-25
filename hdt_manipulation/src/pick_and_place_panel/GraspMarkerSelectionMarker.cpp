#include "GraspMarkerSelectionMarker.h"

// module includes
#include "GraspMarkerSelection.h"
#include "SimpleInteractiveMarkerServer.h"

GraspMarkerSelectionMarker::GraspMarkerSelectionMarker(
    SimpleInteractiveMarkerServer &server,
    const visualization_msgs::InteractiveMarker &int_marker,
    const std::shared_ptr<GraspMarkerSelection> &grasp_selection)
:
    SimpleInteractiveMarker(server, int_marker),
    grasp_selection_(grasp_selection)
{

}

GraspMarkerSelectionMarker::~GraspMarkerSelectionMarker()
{

}

void GraspMarkerSelectionMarker::on_button_click(const Feedback &feedback)
{
    auto invert_marker_colors = [this](visualization_msgs::InteractiveMarker &marker)
    {
        for (auto &control : marker.controls) {
            for (auto &marker : control.markers) {
                invert_color(marker.color);
            }
        }
    };

    if (is_grasp_marker() && !is_selected_grasp())
    {
        ROS_INFO("Clicked a grasp marker");

        // update the color of the old marker to reflect not being selected
        if (grasp_selection_->grasp_selected()) {
            std::string prev_grasp = grasp_selection_->get_grasp();
            SimpleInteractiveMarkerPtr prev_selected_marker = server_.get(prev_grasp);
            assert(prev_selected_marker);

            visualization_msgs::InteractiveMarker old_marker = prev_selected_marker->interactive_marker();
            invert_marker_colors(old_marker);
            server_.update(prev_grasp, old_marker);
        }

        // update the color of this marker to reflect being selected and set the new selection to this marker
        visualization_msgs::InteractiveMarker this_marker = this->interactive_marker();
        invert_marker_colors(this_marker);
        server_.update(name(), this_marker);

        grasp_selection_->set_grasp(name());
    }

    if (is_pregrasp_marker() && !is_selected_pregrasp())
    {
        ROS_INFO("Clicked a pregrasp marker");

        // update the color of the old marker to reflect not being selected
        if (grasp_selection_->pregrasp_selected()) {
            std::string prev_pregrasp = grasp_selection_->get_pregrasp();
            SimpleInteractiveMarkerPtr prev_selected_marker = server_.get(prev_pregrasp);
            assert(prev_selected_marker);

            visualization_msgs::InteractiveMarker old_marker = prev_selected_marker->interactive_marker();
            invert_marker_colors(old_marker);
            server_.update(prev_pregrasp, old_marker);
        }

        // update the color of this marker to reflect being selected and set the new selection to this marker
        visualization_msgs::InteractiveMarker this_marker = this->interactive_marker();
        invert_marker_colors(this_marker);
        server_.update(name(), this_marker);

        grasp_selection_->set_pregrasp(name());
    }
}

bool GraspMarkerSelectionMarker::is_grasp_marker() const
{
    return name().size() >= 5 && name().substr(0, 5) == std::string("grasp");
}

bool GraspMarkerSelectionMarker::is_pregrasp_marker() const
{
    return name().size() >= 8 && name().substr(0, 8) == std::string("pregrasp");
}

bool GraspMarkerSelectionMarker::is_selected_pregrasp() const
{
    return name() == grasp_selection_->get_pregrasp();
}

bool GraspMarkerSelectionMarker::is_selected_grasp() const
{
    return name() == grasp_selection_->get_grasp();
}

void GraspMarkerSelectionMarker::invert_color(std_msgs::ColorRGBA &color)
{
    color.r = 1.0 - color.r;
    color.g = 1.0 - color.g;
    color.b = 1.0 - color.b;
}
