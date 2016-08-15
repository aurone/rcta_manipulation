// system includes
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <spellbook/msg_utils/msg_utils.h>
#include <spellbook/stringifier/stringifier.h>

class JointStatesFilter
{
public:

    JointStatesFilter() :
        nh_(),
        ph_("~"),
        joint_states_raw_sub_(),
        joint_states_pub_(),
        filtered_joint_state_(),
        position_offsets_(),
        low_speed_threshold_(0.1),
        low_speed_position_alpha_(0.05),
        nominal_position_alpha_(0.25),
        last_msg_()
    {
        joint_states_raw_sub_ = nh_.subscribe("joint_states_raw", 10, &JointStatesFilter::joint_states_raw_callback, this);
        joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
    }

    enum RunResult
    {
        SUCCESS = 0,
        FAILED_TO_INITIALIZE
    };

    int run()
    {
        if (!ph_.getParam("position_offsets_degs", position_offsets_)) {
            ROS_ERROR("Failed to retrieve 'position_offsets_degs' from the param server");
            return FAILED_TO_INITIALIZE;
        }

        if (position_offsets_.size() != 7) {
            ROS_ERROR("Wrong number of joint offsets in 'position_offsets_degs' param. Expected 7");
            return FAILED_TO_INITIALIZE;
        }

        ROS_INFO("Retrieved joint position offsets: %s", to_string(position_offsets_).c_str());

        position_offsets_ = msg_utils::to_radians(position_offsets_);

        ros::spin();
        return 0;
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    ros::Subscriber joint_states_raw_sub_;
    ros::Publisher joint_states_pub_;

    sensor_msgs::JointState filtered_joint_state_;

    std::vector<double> position_offsets_;
    double low_speed_threshold_;
    double low_speed_position_alpha_;
    double nominal_position_alpha_;

    sensor_msgs::JointState::ConstPtr last_msg_;

    bool init_joint_state(const sensor_msgs::JointState& first_msg)
    {
        filtered_joint_state_.header.seq = 0;
        filtered_joint_state_.header.frame_id = first_msg.header.frame_id;
        filtered_joint_state_.header.stamp = ros::Time::now();
        filtered_joint_state_.name = first_msg.name;
        if (!msg_utils::vector_sum(first_msg.position, position_offsets_, filtered_joint_state_.position)) {
            return false;
        }

        if (filtered_joint_state_.position.size() != 7) {
            ROS_ERROR("Initial joint state vector contains insufficient joints");
            return false;
        }
        filtered_joint_state_.velocity = first_msg.velocity;
        filtered_joint_state_.effort = first_msg.effort;
        return true;
    }

    void joint_states_raw_callback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        auto interp = [](double from, double to, double a) { return (1.0 - a) * from + a * to; };

        if (filtered_joint_state_.name.empty()) {
            if (!init_joint_state(*msg)) {
                ROS_ERROR("Failed to initialize inital joint state estimate from raw joint state");
                ros::shutdown();
                return;
            }
        }
        else {
            if (msg->name.size() != filtered_joint_state_.name.size()) {
                ROS_WARN("Joint state signature does not match seed joint state");
                return;
            }

            static int seqno = 1; // start at 1 since the 0'th joint state is created in init_joint_state
            filtered_joint_state_.header.seq = seqno++;
            filtered_joint_state_.header.stamp = ros::Time::now();
            for (size_t i = 0; i < filtered_joint_state_.name.size(); ++i) {
                double alpha = msg->velocity[i] < low_speed_threshold_ ? low_speed_position_alpha_ : nominal_position_alpha_;
                // NOTE: vanilla interpolation is fine here since all joints on the hdt are revolute
                filtered_joint_state_.position[i] = interp(filtered_joint_state_.position[i], msg->position[i] + position_offsets_[i], alpha);
            }
            filtered_joint_state_.velocity = msg->velocity;
            filtered_joint_state_.effort = msg->effort;
        }

        joint_states_pub_.publish(filtered_joint_state_);
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "joint_states_filter");
    ros::NodeHandle nh;

    JointStatesFilter filter;
    filter.run();

    return filter.run();
}
