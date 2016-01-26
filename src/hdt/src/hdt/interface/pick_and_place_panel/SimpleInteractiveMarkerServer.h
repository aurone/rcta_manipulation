#ifndef SimpleInteractiveMarkerServer_h
#define SimpleInteractiveMarkerServer_h

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

class SimpleInteractiveMarker;

// An interface to a simplified interactive marker server.
class SimpleInteractiveMarkerServer
{
public:

    SimpleInteractiveMarkerServer(const std::string &name);

    void insert(const std::shared_ptr<SimpleInteractiveMarker> &sim);
    std::shared_ptr<SimpleInteractiveMarker> get(const std::string &marker);
    void update(const std::string &marker, const visualization_msgs::InteractiveMarker &int_marker);
    void remove(const std::string &marker);
    void clear();

    void set_feedback_callback(const std::function<void()>& cb) { fb_cb_ = cb; }

    void flush();

private:

    interactive_markers::InteractiveMarkerServer server_;
    std::map<std::string, std::shared_ptr<SimpleInteractiveMarker>> markers_;
    std::function<void()> fb_cb_;

    void print_interactive_marker_feedback(const visualization_msgs::InteractiveMarkerFeedback& feedback_msg) const;

    void process_feedback(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr &feedback);
};

#endif
