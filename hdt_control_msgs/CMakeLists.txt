cmake_minimum_required(VERSION 2.8.12)
project(hdt_control_msgs)

find_package(
    catkin 
    REQUIRED 
    COMPONENTS 
        actionlib_msgs 
        geometry_msgs
        message_generation 
        sensor_msgs
        std_msgs)

add_message_files(
    DIRECTORY msg 
    FILES 
        AcknowledgeReset.msg 
        ClearEmergencyStop.msg 
        ControllerDiagnosticStatus.msg 
        EmergencyStop.msg 
        JointDiagnosticStatus.msg 
        JointState.msg 
        TeleopDiagnosticStatus.msg)

add_action_files(
    DIRECTORY action
    FILES
        TeleportAndaliteCommand.action 
        TeleportHDTCommand.action 
        ViservoCommand.action)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs sensor_msgs std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs sensor_msgs geometry_msgs)

