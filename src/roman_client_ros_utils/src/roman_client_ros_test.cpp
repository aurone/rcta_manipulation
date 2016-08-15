#include <sstream>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <roman_client_ros_utils/RomanSpec.h>


#include "robot/robot_pub_types.h"
#include "plan/plan_types.h"

//usage
//----
//rosrun roman_client_ros_utils roman_client_ros_test s to publish a spec
//rosrun roman_client_ros_utils roman_client_ros_test h to publish a halt message
//rosrun roman_client_ros_utils roman_client_ros_test r to publish a reset message.

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roman_client_ros_test");
  ros::NodeHandle n;
  bool latchOn = true;
  ros::Publisher roman_spec_pub = n.advertise<roman_client_ros_utils::RomanSpec>("roman_spec", 1,latchOn);
  ros::Publisher roman_halt_pub = n.advertise<std_msgs::Empty>("roman_halt", 1,latchOn);
  ros::Publisher roman_reset_pub = n.advertise<std_msgs::Empty>("roman_reset", 1,latchOn);

  while (ros::ok())
  {
    if(argc>0)
    {
      if(strcmp(argv[1],"s")==0)
      {
        ROS_INFO("Sending a spec");
        roman_client_ros_utils::RomanSpec spec_msg;
        spec_msg.utime = ros::Time::now().toNSec()*1e3;
        spec_msg.num_mechanisms = ROBOT_NUM_MECHS;
        for(int m=0;m<ROBOT_NUM_MECHS;m++)
            spec_msg.valid.push_back(false);
        spec_msg.valid[ROBOT_MECH_LIMB_1] = true;
        spec_msg.valid[ROBOT_MECH_TORSO] = true;
        spec_msg.num_waypoints  = 2;
        for(int w=0;w<spec_msg.num_waypoints;w++)
        {
          roman_client_ros_utils::RomanWaypoint wpt;
          wpt.utime = w*1e5;//100ms increment
          wpt.num_joints = ROBOT_NUM_JOINTS;
          for(int j=0;j<ROBOT_NUM_JOINTS;j++)
            wpt.positions.push_back(0.0);
          wpt.world2robot.position.x = 0.0;
          wpt.world2robot.position.y = 0.0;
          wpt.world2robot.position.z = 0.0;
          wpt.world2robot.orientation.w = 1.0;
          wpt.world2robot.orientation.x = 0.0;
          wpt.world2robot.orientation.y = 0.0;
          wpt.world2robot.orientation.z = 0.0;
          spec_msg.waypoints.push_back(wpt);
        }
        roman_spec_pub.publish(spec_msg);
      }
      else if(strcmp(argv[1],"h")==0)
      {
        ROS_INFO("Sending halt");
        std_msgs::Empty halt_msg;
        roman_halt_pub.publish(halt_msg);
      }
      else if(strcmp(argv[1],"r")==0)
      {
        ROS_INFO("Sending reset");
        std_msgs::Empty reset_msg;
        roman_reset_pub.publish(reset_msg);
      }// end if else

    
    }// end if
    sleep(1); 
    ros::spinOnce();
    ros::shutdown();
  }
  return 0;
}
