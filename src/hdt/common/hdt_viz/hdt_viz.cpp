/* \author Kalin Gochev */

#include <hdt_viz/hdt_viz.h>

using namespace boost;

static std::string RIGHT_CHAIN_RTIP_NAME = "finger1";
static std::string RIGHT_CHAIN_LTIP_NAME = "finger2";
static std::string LEFT_CHAIN_RTIP_NAME = "finger1";
static std::string LEFT_CHAIN_LTIP_NAME = "finger2";

void HDTViz::HSVtoRGB( double *r, double *g, double *b, double h, double s, double v )
{
	int i;
	double f, p, q, t;
	if( s == 0 ) {
		// achromatic (grey)
		*r = *g = *b = v;
		return;
	}
	h /= 60;        // sector 0 to 5
	i = floor(h);
	f = h - i;			// factorial part of h
	p = v * ( 1 - s );
	q = v * ( 1 - s * f );
	t = v * ( 1 - s * ( 1 - f ) );
	switch( i ) {
		case 0:
			*r = v;
			*g = t;
			*b = p;
			break;
		case 1:
			*r = q;
			*g = v;
			*b = p;
			break;
		case 2:
			*r = p;
			*g = v;
			*b = t;
			break;
		case 3:
			*r = p;
			*g = q;
			*b = v;
			break;
		case 4:
			*r = t;
			*g = p;
			*b = v;
			break;
    default:
			*r = v;
			*g = p;
			*b = q;
			break;
	}
}

bool HDTViz::reinit_robot(){
  if (!nh_.hasParam("robot_description") || !nh_.hasParam("robot_description_semantic")) {
        ROS_ERROR("Failed to initialize Manipulator Command Panel; requires \"robot_description\" and \"robot_description_semantic\" parameters");
        return false;
    }

    std::string urdf_string;
    if (!nh_.getParam("robot_description", urdf_string)) {
        ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
        return false;
    }

    hdt::RobotModelPtr robot_model = hdt::RobotModel::LoadFromURDF(urdf_string);
    if (!robot_model) {
        ROS_ERROR("Failed to load robot model from the URDF");
        return false;
    }

    robot_model_loader::RobotModelLoaderPtr rm_loader(new robot_model_loader::RobotModelLoader("robot_description", true));
    if (!rm_loader) {
        ROS_ERROR("Failed to instantiate Robot Model Loader");
        return false;
    }

    robot_model::RobotModelPtr rm = rm_loader->getModel();
    if (!rm) {
        ROS_ERROR("Robot Model Loader was unable to construct Robot Model");
        return false;
    }

    robot_state::RobotStatePtr rs(new robot_state::RobotState(rm));
    if (!rs) {
        ROS_ERROR("Failed to instantiate Robot State");
        return false;
    }

    // All lights are green from above
    robot_model_ = robot_model;
    rm_loader_ = rm_loader;
    rm_ = rm;
    rs_ =  rs;

    ROS_INFO("Root link name: %s", rm_->getRootLinkName().c_str());
    ROS_INFO("Robot Joints:");
    std::vector<std::string> jNames = rm_->getJointModelNames();
    for (int i = 0; i < (int) jNames.size(); i++) {
        ROS_INFO("    %s", jNames[i].c_str());
        arm_joint_names_.push_back(jNames[i]);
    }

    rs_.reset(new robot_state::RobotState(rm_));
    if (!rs_) {
        ROS_ERROR("Failed to instantiate Robot State");
        return false;
    }

    rs_->setToDefaultValues();
    rs_->setRootTransform(Eigen::Affine3d::Identity());
    rs_->updateLinkTransforms();

    // initialize an interactive marker for the first joint group that's a kinematic chain
    /*interactive_markers_.reserve(rm_->getJointModelGroupNames().size());
    for (const auto& joint_model_group_name : rm_->getJointModelGroupNames()) {
        const robot_model::JointModelGroup* jmg = rm_->getJointModelGroup(joint_model_group_name);
        if (jmg->isChain()) {
            // find the tip link of the arm and the link to which the arm is attached
            tip_link_ = get_tip_link(*jmg);
            base_link_ = get_base_link(*jmg);
            ROS_INFO("Found manipulator attached to %s with tip link %s", base_link_.c_str(), tip_link_.c_str());

            ROS_INFO("Joint Group %s:", jmg->getName().c_str());
            ROS_INFO("    Joints:");
            for (const std::string& joint_name : jmg->getJointModelNames()) {
                ROS_INFO("        %s", joint_name.c_str());
            }
            ROS_INFO("    Links:");
            for (const std::string& link_name : jmg->getLinkModelNames()) {
                ROS_INFO("        %s", link_name.c_str());
            }

            // insert an interactive marker for the tip link of the arm
            visualization_msgs::InteractiveMarker interactive_marker;
            interactive_marker.header.seq = 0;
            interactive_marker.header.stamp = ros::Time(0);
            interactive_marker.header.frame_id = rm_->getRootLinkName();
            tf::poseEigenToMsg(rs_->getLinkState(tip_link_)->getGlobalLinkTransform(), interactive_marker.pose);
            interactive_marker.name = jmg->getName() + "_control";
            interactive_marker.description = std::string("Control of ") + tip_link_ + std::string(" of manipulator ") + jmg->getName();
            interactive_marker.scale = 0.5f;
            interactive_marker.menu_entries.clear();
            interactive_marker.controls.clear();
            interactive_marker.controls = create_sixdof_controls();

            ROS_INFO("Inserting interactive marker \"%s\"", interactive_marker.name.c_str());
            server_.insert(interactive_marker);
            server_.setCallback(interactive_marker.name, boost::bind(&ManipulatorCommandPanel::do_process_feedback, this, _1));
            interactive_markers_.push_back(interactive_marker);
            break;
        }
    }*/

    const Eigen::Affine3d& root_to_manipulator_frame = rs_->getFrameTransform("arm_1_shoulder_twist_link");
    const Eigen::Affine3d& root_to_base_frame = rs_->getFrameTransform("base_link");
    const Eigen::Affine3d& mount_frame_to_manipulator_frame_ = root_to_base_frame.inverse() * root_to_manipulator_frame;

    

    //initialized_ = true;
    return true;// initialized_;
}

bool HDTViz::gatherRobotMarkers(
    const robot_state::RobotState& robot_state,
    const std::vector<std::string>& link_names,
    const std_msgs::ColorRGBA& color,
    const std::string& ns,
    const ros::Duration& d,
    visualization_msgs::MarkerArray& markers,
    bool ignore_arm,
    bool include_attached)
{
    ////////////////////////////////////////////////////////////////////////////////
    // Method derived from moveit_ros_planning/robot_state.cpp on groovy-devel
    // branch version a5f7c1c728
    ////////////////////////////////////////////////////////////////////////////////

    std::size_t num_orig_markers = markers.markers.size();

    auto groups = rm_->getJointModelGroupNames();

    ros::Time tm = ros::Time::now();
    for (std::size_t i = 0; i < link_names.size(); ++i) {

        if(ignore_arm){
          bool skip = false;
          for(size_t g = 0; g < groups.size(); g++){
            if(groups[g].compare("manipulator")==0 || 
               groups[g].compare("hdt_gripper")==0) {
              if(rm_->getJointModelGroup(groups[g])->hasLinkModel(link_names[i])){
                skip = true;
                break;
              } 
            }
          }
          if(skip) continue;
        }

        visualization_msgs::Marker mark;
        
        const robot_state::LinkState* ls = robot_state.getLinkState(link_names[i]);
        if (!ls) {
            continue;
        }

        // add markers for attached objects
        if (include_attached) {
            std::vector<const robot_state::AttachedBody*> attached_bodies;
            ls->getAttachedBodies(attached_bodies);
            for (std::size_t j = 0; j < attached_bodies.size(); ++j) {
                if (attached_bodies[j]->getShapes().size() > 0) {
                    visualization_msgs::Marker att_mark;
                    att_mark.header.frame_id = robot_state.getRobotModel()->getModelFrame();
                    att_mark.header.stamp = tm;
                    shapes::constructMarkerFromShape(attached_bodies[j]->getShapes()[0].get(), att_mark);
                    tf::poseEigenToMsg(attached_bodies[j]->getGlobalCollisionBodyTransforms()[0], att_mark.pose);
                    markers.markers.push_back(att_mark);
                }
            }
        }

        if (!ls->getLinkModel() || !ls->getLinkModel()->getShape()) {
            continue;
        }

        mark.header.frame_id = robot_state.getRobotModel()->getModelFrame();
        mark.header.stamp = tm;
        tf::poseEigenToMsg(ls->getGlobalCollisionBodyTransform(), mark.pose);

        // we prefer using the visual mesh, if a mesh is available
        const std::string& mesh_resource = ls->getLinkModel()->getVisualMeshFilename();
        if (mesh_resource.empty()) {
            if (!shapes::constructMarkerFromShape(ls->getLinkModel()->getShape().get(), mark)) {
                continue;
            }

            // if the object is invisible (0 volume) we skip it
            if (fabs(mark.scale.x * mark.scale.y * mark.scale.z) < std::numeric_limits<float>::epsilon()) {
                continue;
            }
        }
        else {
            tf::poseEigenToMsg(ls->getGlobalLinkTransform(), mark.pose);

            mark.type = mark.MESH_RESOURCE;
            mark.mesh_use_embedded_materials = false;
            mark.mesh_resource = mesh_resource;
            const Eigen::Vector3d &mesh_scale = ls->getLinkModel()->getVisualMeshScale();

            mark.scale.x = mesh_scale[0];
            mark.scale.y = mesh_scale[1];
            mark.scale.z = mesh_scale[2];
        }
        markers.markers.push_back(mark);
    }

    for (std::size_t i = num_orig_markers; i < markers.markers.size(); ++i) {
        visualization_msgs::Marker& m = markers.markers[i];
        m.color = color;
        m.ns = ns;
        m.lifetime = d;
        m.id = i;
    }

    return true;
}

HDTViz::HDTViz(const std::string &ns) : rm_loader_(),
    rm_(),
    rs_()
{

  if (!reinit_robot()) {
	ROS_ERROR("Failed to initialize robot! Some visualizations might not work!");
        return;
  }

  num_joints_ = 7; //arm + torso
  reference_frame_ = "/base_footprint";
  last_traj_size_ = 0;

  srand (time(NULL));
  arm_joint_names_.push_back("arm_1_shoulder_twist");
  arm_joint_names_.push_back("arm_2_shoulder_lift");
  arm_joint_names_.push_back("arm_3_elbow_twist");
  arm_joint_names_.push_back("arm_4_elbow_lift");
  arm_joint_names_.push_back("arm_5_wrist_twist");
  arm_joint_names_.push_back("arm_6_wrist_lift");
  arm_joint_names_.push_back("arm_7_gripper_lift");
}

HDTViz::~HDTViz(){
  arm_joint_names_.clear();
}

void HDTViz::visualizeRobot(std::vector<double> &jnt0_pos, double hue, std::string ns, int &id, bool use_embedded_materials){
  
  std::vector<double> base_pos(3,0);

  visualizeRobot(jnt0_pos, base_pos, hue, ns, id, use_embedded_materials);  
}

void HDTViz::visualizeRobot(std::vector<double> &jnt0_pos, std::vector<double> &base_pos, double hue, std::string ns, int &id, bool use_embedded_materials)
{

  double r=0,g=0,b=0;
  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  visualization_msgs::MarkerArray marker_array;
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.r = r;
  color.g = g;
  color.b = b;
  ros::Duration d(0);

  rs_->getJointState("arm_1_shoulder_twist")->setVariableValues(&jnt0_pos[0]);
  rs_->getJointState("arm_2_shoulder_lift")->setVariableValues(&jnt0_pos[1]);
  rs_->getJointState("arm_3_elbow_twist")->setVariableValues(&jnt0_pos[2]);
  rs_->getJointState("arm_4_elbow_lift")->setVariableValues(&jnt0_pos[3]);
  rs_->getJointState("arm_5_wrist_twist")->setVariableValues(&jnt0_pos[4]);
  rs_->getJointState("arm_6_wrist_lift")->setVariableValues(&jnt0_pos[5]);
  rs_->getJointState("arm_7_gripper_lift")->setVariableValues(&jnt0_pos[6]);
  rs_->updateLinkTransforms();
  //TODO: include base position as well

  std::vector<std::string> link_names = rm_->getLinkModelNames();
  gatherRobotMarkers(*rs_, link_names, color, ns, d, marker_array);


  tf::Transform robot_local_to_global_;
  robot_local_to_global_.setOrigin(tf::Vector3(base_pos[0], base_pos[1], 0));
  robot_local_to_global_.setRotation(tf::Quaternion(tf::Vector3(0,0,1), base_pos[2]));
  transformMarkerArray(marker_array, robot_local_to_global_);

  marker_array_publisher_.publish(marker_array);
}

void HDTViz::visualizeRobotBase(std::vector<double> &base_pos, double hue, std::string ns, int &id, bool use_embedded_materials){
  BodyPose b;
  b.x = base_pos[0];
  b.y = base_pos[1];
  b.z = 0;
  b.theta = base_pos[2]; 
  visualizeRobotBase(b, hue, ns, id, use_embedded_materials);
}

void HDTViz::visualizeRobotBase(BodyPose &base_pos, double hue, std::string ns, int &id, bool use_embedded_materials){
  double r=0,g=0,b=0;
  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);
  visualization_msgs::MarkerArray marker_array;
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.r = r;
  color.g = g;
  color.b = b;
  ros::Duration d(0);

  std::vector<std::string> link_names = rm_->getLinkModelNames();
  gatherRobotMarkers(*rs_, link_names, color, ns, d, marker_array, true); //ignore_arm set

  tf::Transform robot_local_to_global_;
  robot_local_to_global_.setOrigin(tf::Vector3(base_pos.x, base_pos.y, 0));
  robot_local_to_global_.setRotation(tf::Quaternion(tf::Vector3(0,0,1), base_pos.theta));
  transformMarkerArray(marker_array, robot_local_to_global_);

  for(size_t i = 0; i < marker_array.markers.size(); i++){
    marker_array.markers[i].id = id + i;
    marker_array.markers[i].header.frame_id = reference_frame_;
  }
  id += marker_array.markers.size();
  marker_array_publisher_.publish(marker_array);
}

visualization_msgs::MarkerArray HDTViz::getRobotMarkers(std::vector<double> &jnt0_pos, BodyPose &base_pos, double hue, std::string ns, int id, bool use_embedded_materials){
  double r=0,g=0,b=0;
  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  visualization_msgs::MarkerArray marker_array;
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.r = r;
  color.g = g;
  color.b = b;
  ros::Duration d(0);

  rs_->getJointState("arm_1_shoulder_twist")->setVariableValues(&jnt0_pos[0]);
  rs_->getJointState("arm_2_shoulder_lift")->setVariableValues(&jnt0_pos[1]);
  rs_->getJointState("arm_3_elbow_twist")->setVariableValues(&jnt0_pos[2]);
  rs_->getJointState("arm_4_elbow_lift")->setVariableValues(&jnt0_pos[3]);
  rs_->getJointState("arm_5_wrist_twist")->setVariableValues(&jnt0_pos[4]);
  rs_->getJointState("arm_6_wrist_lift")->setVariableValues(&jnt0_pos[5]);
  rs_->getJointState("arm_7_gripper_lift")->setVariableValues(&jnt0_pos[6]);
  rs_->updateLinkTransforms();
  //TODO: include base position as well

  std::vector<std::string> link_names = rm_->getLinkModelNames();
  gatherRobotMarkers(*rs_, link_names, color, ns, d, marker_array);

  tf::Transform robot_local_to_global_;
  robot_local_to_global_.setOrigin(tf::Vector3(base_pos.x, base_pos.y, base_pos.z));
  robot_local_to_global_.setRotation(tf::Quaternion(tf::Vector3(0,0,1), base_pos.theta));
  transformMarkerArray(marker_array, robot_local_to_global_);
  return marker_array;
}

void HDTViz::visualizeRobots(std::vector<std::vector<double> > &arm_pos, double hue, std::string ns, int &id, int throttle, bool use_embedded_materials){

  BodyPose body_pos;
  visualization_msgs::MarkerArray ma, ma1;
  body_pos.x = 0;
  body_pos.y = 0;
  body_pos.z = 0;
  body_pos.theta = 0;
  for(int i = 0; i < (int)arm_pos.size(); i+=throttle){
    ma1 = getRobotMarkers(arm_pos[i], body_pos, hue, ns, 0, false);
    ma.markers.insert(ma.markers.end(), ma1.markers.begin(), ma1.markers.end());
  }
  for(int i = 0; i < (int)ma.markers.size(); i++){
    ma.markers[i].id = i;
  }
  marker_array_publisher_.publish(ma);
}

void HDTViz::visualizeRobot(std::vector<double> &jnt0_pos, BodyPose &body_pos, double hue, std::string ns, int &id, bool use_embedded_materials)
{
  double torso_pos;
  std::vector<double> base_pos(3,0);

  base_pos[0] = body_pos.x;
  base_pos[1] = body_pos.y;
  base_pos[2] = body_pos.theta;

  visualizeRobot(jnt0_pos, base_pos, hue, ns, id, use_embedded_materials);
}

void HDTViz::visualizeRobotWithTitle(std::vector<double> &jnt0_pos, BodyPose &body_pos, double hue, std::string ns, int &id, std::string title)
{
  visualizeRobot(jnt0_pos, body_pos, hue, ns, id);

  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = ns + "_title";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.pose.position.x = body_pos.x;
  marker.pose.position.y = body_pos.y;
  marker.pose.position.z = body_pos.z+1.5;
  
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.5;
  marker.text = title;
  marker.lifetime = ros::Duration(180.0);
  marker_publisher_.publish(marker);
}

void HDTViz::visualizeTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> &armpath, std::vector<trajectory_msgs::JointTrajectoryPoint> &bpath, std::string ns, int throttle)
{

  ROS_ERROR("HDTViz::visualizeTrajectory not implemented yet!");
  int length = armpath.size();
  std::vector<double> rangles(7, 0);
  BodyPose body_pos;
  visualization_msgs::MarkerArray ma, ma1;

  if (armpath.size() != bpath.size()) {
    ROS_ERROR("[HDTViz] The right arm and body trajectories are of unequal lengths.");
    return;
  }

  int color_inc = 240.0 / (length / throttle); // hue: red -> blue
  ROS_DEBUG("[HDTViz] length: %d color_inc: %d throttle: %d)", length, color_inc, throttle);

  for (int i = 0; i < length; ++i) {
    for (std::size_t j = 0; j < rangles.size(); ++j) {
      rangles[j] = armpath[i].positions[j];
    }
    body_pos.x = bpath[i].positions[0];
    body_pos.y = bpath[i].positions[1];
    body_pos.z = bpath[i].positions[2];
    body_pos.theta = bpath[i].positions[3];

    if ((i != length - 1) && (i % throttle != 0))
      continue;

    ma1 = getRobotMarkers(rangles, body_pos, (i / throttle) * color_inc, ns, 0, false);
    ma.markers.insert(ma.markers.end(), ma1.markers.begin(), ma1.markers.end());

    ROS_DEBUG("[pviz] length: %d color_inc: %d throttle: %d", length, color_inc, throttle);
    ROS_DEBUG("[pviz] Visualizing waypoint #%d (i mod color_inc: %d) with color: %d (color_inc: %d, throttle: %d)", i, (i / throttle), (i / throttle) * color_inc, color_inc, throttle);
  }
  if(last_traj_size_ == 0) last_traj_size_ = ma.markers.size();
  if(ma.markers.size() < last_traj_size_){
    ma.markers.resize(last_traj_size_);
  }
  for(int i = 0; i < (int)ma.markers.size(); i++){
    if(i > last_traj_size_){
      ma.markers[i] = ma.markers[last_traj_size_-1];
      ma.markers[i].action = visualization_msgs::Marker::DELETE;
    }
    ma.markers[i].id = i;
  }
  last_traj_size_ = ma.markers.size();
  ROS_INFO("[pviz] Visualizing a robot path with %d waypoints. (throttle = %d)", int(armpath.size()), throttle);
  marker_array_publisher_.publish(ma);
}

void HDTViz::visualizeTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> &armpath, std::string ns, int throttle)
{
  int length = armpath.size();
  std::vector<double> rangles(7, 0);
  BodyPose body_pos;
  visualization_msgs::MarkerArray ma, ma1;

  int color_inc = 240.0 / (length / throttle); // hue: red -> blue
  ROS_INFO("[HDTViz] length: %d color_inc: %d throttle: %d)", length, color_inc, throttle);

  for (int i = 0; i < length; ++i) {
    for (std::size_t j = 0; j < rangles.size(); ++j) {
      rangles[j] = armpath[i].positions[j];
    }
    body_pos.x = 0;
    body_pos.y = 0;
    body_pos.z = 0;
    body_pos.theta = 0;

    if ((i != length - 1) && (i % throttle != 0))
      continue;

    ma1 = getRobotMarkers(rangles, body_pos, (i / throttle) * color_inc, ns, 0, false);
    ma.markers.insert(ma.markers.end(), ma1.markers.begin(), ma1.markers.end());

    ROS_DEBUG("[pviz] length: %d color_inc: %d throttle: %d", length, color_inc, throttle);
    ROS_DEBUG("[pviz] Visualizing waypoint #%d (i mod color_inc: %d) with color: %d (color_inc: %d, throttle: %d)", i, (i / throttle), (i / throttle) * color_inc, color_inc, throttle);
  }
  if(last_traj_size_ == 0) last_traj_size_ = ma.markers.size();
  if(ma.markers.size() < last_traj_size_){
    ma.markers.resize(last_traj_size_);
  }
  for(int i = 0; i < (int)ma.markers.size(); i++){
    if(i > last_traj_size_){
      ma.markers[i] = ma.markers[last_traj_size_-1];
      ma.markers[i].action = visualization_msgs::Marker::DELETE;
    }
    ma.markers[i].id = i;
  }
  last_traj_size_ = ma.markers.size();
  ROS_INFO("[pviz] Visualizing a robot path with %d waypoints. (throttle = %d)", int(armpath.size()), throttle);
  marker_array_publisher_.publish(ma);
}

