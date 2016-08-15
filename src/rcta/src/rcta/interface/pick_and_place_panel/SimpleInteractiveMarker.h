#ifndef SimpleInteractiveMarker_h
#define SimpleInteractiveMarker_h

// standard includes
#include <cstdint>
#include <string>

// system includes
#include <std_msgs/Header.h>
#include <visualization_msgs/InteractiveMarker.h>

class SimpleInteractiveMarkerServer;

class SimpleInteractiveMarker
{
public:

    struct Feedback
    {
        std_msgs::Header header;
        std::string client_id;
        std::string control_name;
    };

    SimpleInteractiveMarker(SimpleInteractiveMarkerServer &server, const visualization_msgs::InteractiveMarker &marker);

    virtual ~SimpleInteractiveMarker();

    const std::string &name() const { return int_marker_.name; }

    const visualization_msgs::InteractiveMarker &interactive_marker() const { return int_marker_; }
    void interactive_marker(const visualization_msgs::InteractiveMarker &marker) { int_marker_ = marker; }

    virtual void on_keep_alive(const Feedback &feedback);
    virtual void on_pose_update(const Feedback &feedback, const geometry_msgs::Pose &pose);
    virtual void on_menu_select(const Feedback &feedback, uint32_t entry_id);
    virtual void on_button_click(const Feedback &feedback);
    virtual void on_mouse_down();
    virtual void on_mouse_up();

protected:

    SimpleInteractiveMarkerServer          &server_;

private:

    visualization_msgs::InteractiveMarker   int_marker_;
};

typedef std::shared_ptr<SimpleInteractiveMarker> SimpleInteractiveMarkerPtr;

#endif
