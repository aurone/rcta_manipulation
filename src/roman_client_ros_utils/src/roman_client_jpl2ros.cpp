#include <sstream>
#include <signal.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <roman_client_ros_utils/RomanState.h>
#include <roman_client_ros_utils/RomanSpecReply.h>

#include "common/rsap_modules.h"
#include "rsap/rsap_error.h"
#include "rsap/rsap_ipc.h"
#include "robot/robot_pub_types.h"
#include "plan/plan_types.h"
#include "roman_client/roman_mech_client_pub.h"
#include "roman_client/roman_mech_client_msgs.h"


using namespace roman_client_ros_utils;

void translate_state(roman_mech_client_pub_state_msg_t* jplmsg, RomanState* rosmsg);
void translate_spec_reply(roman_mech_client_pub_spec_reply_t* jplmsg,RomanSpecReply* rosmsg);
void process_rsap_ipc_queue(rsap_ipc_t* ipc, ros::Publisher& state_pub, ros::Publisher& spec_reply_pub);

// jpl2ros passthrough translator.
// This node listens for state & spec reply messages from roman_client and publishes them onto ros.
// State is published by roman_client at 10Hz (hardcoded in roman_client)
// Spec reply is the acknowledgement from roman_client after a trajectory has been executed.
// Its provides fault handling information.


int main(int argc, char **argv)
{

  // RSAP Comms 
  rsap_ipc_t* ipc;
  if(!getenv("RS_LIMB_ROOT"))
  {
    ROS_ERROR("RS_LIMB_ROOT is not set");
    return RSAP_ERROR("RS_LIMB_ROOT is not set");
  }


  // alloc & init
  ipc = rsap_ipc_alloc_ex(1024, 100*1024*100);
  assert(ipc);

  std::stringstream ss;
  ss << getenv("RS_LIMB_ROOT") << "/sbin/ipc.cfg";
  ROS_INFO("rsap ipc.cfg file:: %s", ss.str().c_str());
  rsap_module_t module = ROMAN_CLIENT_JPL2ROS;  //TODO add JPL2ROS modules
  if(rsap_ipc_init(ipc, module, ss.str().c_str()) !=0)
  {
    ROS_ERROR("could not open jpl rsap ipc, make sure ipc.cfg file has ROMAN_CLIENT_JPL2ROS defined");
    rsap_ipc_free(ipc);
    return -1;
  }

  rsap_ipc_subscribe(ipc, ROMAN_MECH_CLIENT_STATE_MSG);
  rsap_ipc_subscribe(ipc, ROMAN_MECH_CLIENT_SPEC_REPLY);


  ros::init(argc, argv, "roman_client_jpl2ros");
  ros::NodeHandle n;

  //  The second parameter to advertise() is the size of the message queue
  //  used for publishing messages.  If messages are published more quickly
  //  than we can send them, the number here specifies how many messages to
  //  buffer up before throwing some away.
  bool latchOn = true;// A latching publication will automatically send out the last published message to any new subscribers. 
  ros::Publisher state_pub = n.advertise<RomanState>("roman_state", 1);
  ros::Publisher spec_reply_pub = n.advertise<RomanSpecReply>("roman_spec_reply", 100,latchOn);

  while(ros::ok())
  {
    // call process rsap  ipc queue & translate message
    process_rsap_ipc_queue(ipc,state_pub,spec_reply_pub);
    ros::spinOnce();
  }

  // fini & free
  rsap_ipc_unsubscribe(ipc, ROMAN_MECH_CLIENT_STATE_MSG);
  rsap_ipc_unsubscribe(ipc, ROMAN_MECH_CLIENT_SPEC_REPLY);
  rsap_ipc_fini(ipc);
  rsap_ipc_free(ipc);

  return 0;
}


// jpl rsap ipc process
//-----------------------------------
void process_rsap_ipc_queue(rsap_ipc_t* ipc, ros::Publisher& state_pub, ros::Publisher& spec_reply_pub)
{
 
  uint64_t type;
  uint8_t data[RSAP_IPC_MAX_SIZE];
  size_t size;
  while(1)      
  {
    // read, translate and dispatch all pending messages
    size = RSAP_IPC_MAX_SIZE;
    if(rsap_ipc_read(ipc, &type, &size, data) == 0)
    {
      switch(type)
      {
        case ROMAN_MECH_CLIENT_STATE_MSG:
        {
          roman_mech_client_pub_state_msg_t* state_jplmsg = (roman_mech_client_pub_state_msg_t*) data;
          RomanState state_rosmsg; 
          translate_state(state_jplmsg,&state_rosmsg); 
          state_pub.publish(state_rosmsg);
          break;
        }
        case ROMAN_MECH_CLIENT_SPEC_REPLY:
        {
          roman_mech_client_pub_spec_reply_t* spec_reply_jplmsg = (roman_mech_client_pub_spec_reply_t*) data;
          RomanSpecReply spec_reply_rosmsg; 
          translate_spec_reply(spec_reply_jplmsg,&spec_reply_rosmsg); 
          spec_reply_pub.publish(spec_reply_rosmsg);
          break;
        }
        default:
        {
           ROS_ERROR("unhandled message: type=%ld", type);
           return;
        }
      }
    }
    else
      break;
  }// end while 1
  return;
}


// msg translators
//-----------------------------------

void translate_state(roman_mech_client_pub_state_msg_t* jplmsg, RomanState* rosmsg)
{
  rosmsg->utime = jplmsg->timestamp;
  rosmsg->in_fault = jplmsg->in_fault;

  // populate fault state
  // --------------------
  rosmsg->fault_status.utime = rosmsg->utime;
  // fault handling for the control algorithms
  elmo_mech_client_pub_fault_type_t type = jplmsg->fault_status.ctrl_status;
  rosmsg->fault_status.ctrl_fault_status = elmo_mech_client_pub_fault_type_names[type];
  rosmsg->fault_status.mech_id = jplmsg->fault_status.mech;
  rosmsg->fault_status.joint_id = jplmsg->fault_status.joint;
  // fault handling for each mech that is controlled
  rosmsg->fault_status.num_mechanisms = ROBOT_NUM_MECHS;
  for(int m=0;m<rosmsg->fault_status.num_mechanisms;m++)
  {
    elmo_mech_client_pub_fault_type_t type = jplmsg->fault_status.mech_status[m];
    rosmsg->fault_status.mech_fault_status.push_back(elmo_mech_client_pub_fault_type_names[type]);
  }

  // populate robot state
  // --------------------
  rosmsg->state.num_joints = ROBOT_NUM_JOINTS;
  for(int i=0;i<rosmsg->state.num_joints;i++)
    rosmsg->state.positions.push_back(jplmsg->state.positions[i]);

  // roman_client is not doing any odometry. this is just set to identity for now.
  rosmsg->state.world2robot.position.x    = jplmsg->state.world2robot.pos.x;
  rosmsg->state.world2robot.position.y    = jplmsg->state.world2robot.pos.y;
  rosmsg->state.world2robot.position.z    = jplmsg->state.world2robot.pos.z;
  rosmsg->state.world2robot.orientation.w = jplmsg->state.world2robot.rot.u;
  rosmsg->state.world2robot.orientation.x = jplmsg->state.world2robot.rot.x;
  rosmsg->state.world2robot.orientation.y = jplmsg->state.world2robot.rot.y;
  rosmsg->state.world2robot.orientation.z = jplmsg->state.world2robot.rot.z;

  return;
}

void translate_spec_reply(roman_mech_client_pub_spec_reply_t* jplmsg,RomanSpecReply* rosmsg)
{
  rosmsg->utime = rsap_get_time();
  rosmsg->attempted = jplmsg->attempted; // attemped spec
  rosmsg->in_fault = jplmsg->fault_status.in_fault;

  // populate fault state
  // --------------------
  rosmsg->fault_status.utime = rosmsg->utime;
  // fault handling for the control algorithms
  elmo_mech_client_pub_fault_type_t type = jplmsg->fault_status.ctrl_status;
  rosmsg->fault_status.ctrl_fault_status = elmo_mech_client_pub_fault_type_names[type];
  rosmsg->fault_status.mech_id = jplmsg->fault_status.mech;
  rosmsg->fault_status.joint_id = jplmsg->fault_status.joint;
  // fault handling for each mech that is controlled
  rosmsg->fault_status.num_mechanisms = ROBOT_NUM_MECHS;
  for(int m=0;m<rosmsg->fault_status.num_mechanisms;m++)
  {
    elmo_mech_client_pub_fault_type_t type = jplmsg->fault_status.mech_status[m];
    rosmsg->fault_status.mech_fault_status.push_back(elmo_mech_client_pub_fault_type_names[type]);
  }

  return;
}
//-----------------------------------
