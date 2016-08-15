#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include <roman_client_ros_utils/RomanSpec.h>

#include "common/rsap_modules.h"
#include "rsap/rsap_error.h"
#include "rsap/rsap_ipc.h"
#include "robot/robot_pub_types.h"
#include "plan/plan_types.h"
#include "roman_client/roman_mech_client_pub.h"
#include "roman_client/roman_mech_client_msgs.h"

// ros2jpl cmd passthrough translator.
using namespace roman_client_ros_utils;

rsap_ipc_t* ipc; // define globally so that it is can be used in calbacks
rsap_module_t ipc_module = ROMAN_CLIENT_ROS2JPL; //TODO add ROS2JPL module

//function defs
void roman_spec_callback(const RomanSpec& rosmsg);
void roman_halt_callback(const std_msgs::Empty& rosmsg);
void roman_reset_callback(const std_msgs::Empty& rosmsg);
void translate_spec(const RomanSpec& rosmsg,roman_spec_t& jplspec);

int main(int argc, char **argv)
{

  ros::init(argc, argv, "roman_client_ros2jpl");
  ros::NodeHandle n;

  ipc = rsap_ipc_alloc_ex(1024, 100*1024*100);
  assert(ipc);

  std::stringstream ss;
  ss << getenv("RS_LIMB_ROOT") << "/sbin/ipc.cfg";
  ROS_INFO("rsap ipc.cfg file:: %s", ss.str().c_str());
  if(rsap_ipc_init(ipc, ipc_module, ss.str().c_str()) !=0)
  {
    ROS_ERROR("could not open jpl rsap ipc, make sure ipc.cfg file has ROMAN_CLIENT_ROS2JPL defined");
    rsap_ipc_free(ipc);
    return -1;
  }

  //The second parameter to the subscribe() function is the size of the message
  //queue.  If messages are arriving faster than they are being processed, this
  //is the number of messages that will be buffered up before beginning to throw
  //away the oldest ones.
  ros::Subscriber spec_sub = n.subscribe("roman_spec", 1, roman_spec_callback);
  ros::Subscriber halt_sub = n.subscribe("roman_halt", 10, roman_halt_callback);
  ros::Subscriber reset_sub = n.subscribe("roman_reset", 10, roman_reset_callback);
  ros::spin();

  rsap_ipc_fini(ipc);
  rsap_ipc_free(ipc);
  return 0;
}

// callbacks
// ----------------
void roman_spec_callback(const RomanSpec& rosmsg)
{
  ROS_INFO("Got msg on roman_spec in rs, passing to client via lump..");
  roman_spec_t spec;
  translate_spec(rosmsg, spec);
  roman_mech_client_pub_execute_trajectory(ipc,&spec,ipc_module);
}

void roman_halt_callback(const std_msgs::Empty& rosmsg)
{
  ROS_INFO("Got msg on roman_halt (software e-stop) in ros, passing to client via lump.");
  roman_mech_client_pub_halt_msg(ipc,ipc_module);

}

void roman_reset_callback(const std_msgs::Empty& rosmsg)
{
  ROS_INFO("Got msg on roman_reset (reset from software e-stop) in ros, passing to client via lump.");
  roman_mech_client_pub_reset_msg(ipc,ipc_module);
}

// translators
// ----------------
void translate_spec(const RomanSpec& rosmsg,roman_spec_t& spec)
{

  //rosmsg.utime  is for debug only
  for(int m=0;m < rosmsg.num_mechanisms;m++)
    spec.valid[m] = rosmsg.valid[m];
  spec.num_waypoints = rosmsg.num_waypoints;
  for(int w=0;w < spec.num_waypoints;w++)
  {
    spec.waypoints[w].timestamp = rosmsg.waypoints[w].utime; 
    for(int j=0;j<ROBOT_NUM_JOINTS;j++)
       spec.waypoints[w].positions[j] = rosmsg.waypoints[w].positions[j];
    spec.waypoints[w].world2robot.pos.x = rosmsg.waypoints[w].world2robot.position.x;
    spec.waypoints[w].world2robot.pos.y = rosmsg.waypoints[w].world2robot.position.y;
    spec.waypoints[w].world2robot.pos.z = rosmsg.waypoints[w].world2robot.position.z;
    spec.waypoints[w].world2robot.rot.u = rosmsg.waypoints[w].world2robot.orientation.w;
    spec.waypoints[w].world2robot.rot.x = rosmsg.waypoints[w].world2robot.orientation.x;
    spec.waypoints[w].world2robot.rot.y = rosmsg.waypoints[w].world2robot.orientation.y;
    spec.waypoints[w].world2robot.rot.z = rosmsg.waypoints[w].world2robot.orientation.z;
  }
}

