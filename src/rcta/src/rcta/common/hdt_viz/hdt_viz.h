/* \author Kalin Gochev */

#ifndef _HDT_VIZ_
#define _HDT_VIZ_

//#include <common/hdt_description/RobotModel.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometric_shapes/shape_operations.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <pviz/pviz.h>
#include <hdt_description/RobotModel.h>

class HDTViz : public PVizGeneric
{
  public:

    void HSVtoRGB( double *r, double *g, double *b, double h, double s, double v );

    /* \brief constructor takes in the desired topic name */
    HDTViz(const std::string &ns = std::string());

    ~HDTViz();

    void getMaptoRobotTransform(double x, double y, double theta, KDL::Frame &frame);

    /**************** Robot Meshes ****************/

    void visualizeRobot(std::vector<double> &arm_pos, std::vector<double> &base_pos, double hue, std::string ns, int &id, bool use_embedded_materials = false);

    void visualizeRobot(std::vector<double> &arm_pos, BodyPose &base_pos, double hue, std::string ns, int &id, bool use_embedded_materials = false);

    void visualizeRobot(std::vector<double> &arm_pos, double hue, std::string ns, int &id, bool use_embedded_materials = false);

    void visualizeRobots(std::vector<std::vector<double> > &arm_pos, double hue, std::string ns, int &id, int throttle, bool use_embedded_materials = false);

    void visualizeRobotBase(
        BodyPose &base_pos,
        double hue,
        std::string ns,
        int &id,
        bool use_embedded_materials = false);

    void visualizeRobotBase(
        std::vector<double> &base_pos,
        double hue,
        std::string ns,
        int &id,
        bool use_embedded_materials=false);

    void visualizeRobotWithTitle(std::vector<double> &arm_pos, BodyPose &base_pos, double hue, std::string ns, int &id, std::string title);

    void visualizeTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> &arm_path, std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath, std::string ns, int throttle);

    void visualizeTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> &arm_path,  std::string ns, int throttle);

    void visualizeGripper(const geometry_msgs::Pose &pose, double hue, std::string ns, int id, bool open);

    /****************** Get Markers *****************/
    void getGripperMeshesMarkerMsg(const geometry_msgs::Pose &pose, double hue, std::string ns, int id, bool open, std::vector<visualization_msgs::Marker> &markers);

    void getGripperMeshesMarkerMsg(const geometry_msgs::Pose &pose, double hue, std::string ns, int id, double position, std::vector<visualization_msgs::Marker> &markers);

    visualization_msgs::MarkerArray getRobotMeshesMarkerMsg(double hue, std::string ns, int id, std::vector<geometry_msgs::PoseStamped> &poses, bool use_embedded_materials = false);

    visualization_msgs::MarkerArray getRobotMarkerMsg(std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, BodyPose &body_pos, double hue, std::string ns, int id, bool use_embedded_materials = false);

  private:
    int last_traj_size_;

    visualization_msgs::MarkerArray getRobotMarkers(std::vector<double> &jnt0_pos, BodyPose &base_pos, double hue, std::string ns, int id, bool use_embedded_materials);

    inline void transformMarkerArray(visualization_msgs::MarkerArray& markers, tf::Transform tran_){
      for(int i = 0; i < markers.markers.size(); i++){
        transformMarker(markers.markers[i], tran_);
      }
    }

    inline void transformMarker(visualization_msgs::Marker& marker, tf::Transform tran_){
      tf::Transform marker_(
                tf::Quaternion(marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w),
                tf::Vector3(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z));
      marker_ = tran_ * marker_;
      marker.pose.position.x = marker_.getOrigin().getX();
      marker.pose.position.y = marker_.getOrigin().getY();
      marker.pose.position.z = marker_.getOrigin().getZ();
      marker.pose.orientation.x = marker_.getRotation().x();
      marker.pose.orientation.y = marker_.getRotation().y();
      marker.pose.orientation.z = marker_.getRotation().z();
      marker.pose.orientation.w = marker_.getRotation().w();
    }

    bool gatherRobotMarkers(
      const robot_state::RobotState& robot_state,
      const std::vector<std::string>& link_names,
      const std_msgs::ColorRGBA& color,
      const std::string& ns,
      const ros::Duration& d,
      visualization_msgs::MarkerArray& markers,
      bool ignore_arm=false,
      bool include_attached=false);

    bool reinit_robot();

    hdt::RobotModelPtr robot_model_;

    robot_model_loader::RobotModelLoaderPtr rm_loader_;
    robot_model::RobotModelPtr rm_;
    robot_state::RobotStatePtr rs_;

    int num_joints_;

    std::vector<std::string> arm_joint_names_;
    std::vector<std::string> arm_link_names_;

    std::vector<std::string> arm_meshes_;
    std::vector<std::string> gripper_meshes_;

    std::vector<std::string> base_meshes_;
    std::vector<std::string> robot_meshes_;

    KDL::JntArray jnt_pos_in_;
    KDL::JntArray jnt_pos_out_;
    KDL::Frame p_out_;
    KDL::Chain chain_;
    KDL::Tree kdl_tree_;
    std::vector<KDL::ChainFkSolverPos_recursive *> fk_rsolver_;
    std::vector<KDL::ChainFkSolverPos_recursive *> fk_lsolver_;

    /* \brief visualize robot meshes...not to be used publicly */
    void visualizeRobotMeshes(double hue, std::string ns, int start_id, std::vector<geometry_msgs::PoseStamped> &poses, bool use_embedded_materials = false);
};

#endif

