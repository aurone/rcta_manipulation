#ifndef rcta_actionlib_h
#define rcta_actionlib_h

namespace rcta {

template <typename ActionType>
bool ReconnectActionClient(
    std::unique_ptr<actionlib::SimpleActionClient<ActionType>>& client,
    const std::string& action_name,
    ros::Rate poll_frequency = ros::Rate(10.0),
    const ros::Duration& timeout = ros::Duration(1.0))
{
    if (!client) {
        client.reset(new actionlib::SimpleActionClient<ActionType>(action_name, false));
    }

    ROS_DEBUG("Waiting for action server '%s'", action_name.c_str());

    if (!client) {
        ROS_WARN("Action client is null");
        return false;
    }

    ros::Time start = ros::Time::now();
    while (ros::ok() && (timeout == ros::Duration(0) || ros::Time::now() < start + timeout)) {
        ros::spinOnce();
        if (!client->isServerConnected()) {
            client.reset(new actionlib::SimpleActionClient<ActionType>(action_name, false));
            if (!client) {
                ROS_WARN("Failed to reinstantiate action client '%s'", action_name.c_str());
                return false;
            }
        }

        if (client->isServerConnected()) {
            return true;
        }

        poll_frequency.sleep();

        ROS_DEBUG("Waited %0.3f seconds for action server '%s'...", (ros::Time::now() - start).toSec(), action_name.c_str());
    }

    return false;
}

} // namespace rcta

#endif
