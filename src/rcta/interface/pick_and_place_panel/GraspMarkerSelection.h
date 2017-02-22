#ifndef GraspMarkerSelection_h
#define GraspMarkerSelection_h

// standard includes
#include <string>

class GraspMarkerSelection
{
public:

    GraspMarkerSelection() : grasp_marker_(), pregrasp_marker_() { }

    void set_pregrasp(const std::string &pregrasp_marker) { pregrasp_marker_ = pregrasp_marker; }
    void set_grasp(const std::string &grasp_marker) { grasp_marker_ = grasp_marker; }

    const std::string &get_pregrasp() { return pregrasp_marker_; }
    const std::string &get_grasp() { return grasp_marker_; }

    bool pregrasp_selected() const { return !pregrasp_marker_.empty(); }
    bool grasp_selected() const { return !grasp_marker_.empty(); }

    void clear() { grasp_marker_ = std::string(); pregrasp_marker_ = std::string(); }

private:

    std::string grasp_marker_;
    std::string pregrasp_marker_;
};

#endif
