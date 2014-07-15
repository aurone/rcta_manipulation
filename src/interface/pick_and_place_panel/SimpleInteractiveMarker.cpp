#include "SimpleInteractiveMarker.h"

SimpleInteractiveMarker::SimpleInteractiveMarker(
    SimpleInteractiveMarkerServer &server,
    const visualization_msgs::InteractiveMarker &marker)
:
    server_(server),
    int_marker_(marker)
{

}

SimpleInteractiveMarker::~SimpleInteractiveMarker()
{
}

void SimpleInteractiveMarker::on_keep_alive(const Feedback &feedback)
{

}

void SimpleInteractiveMarker::on_pose_update(const Feedback &feedback, const geometry_msgs::Pose &pose)
{

}

void SimpleInteractiveMarker::on_menu_select(const Feedback &feedback, uint32_t entry_id)
{

}

void SimpleInteractiveMarker::on_button_click(const Feedback &feedback)
{

}

void SimpleInteractiveMarker::on_mouse_down()
{

}

void SimpleInteractiveMarker::on_mouse_up()
{

}
