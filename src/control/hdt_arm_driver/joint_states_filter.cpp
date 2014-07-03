#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class JointStatesFilter
{
public:

    JointStatesFilter() :
        nh_(),
        joint_states_raw_sub_(),
        joint_states_pub_(),
        filtered_joint_state_(),
        position_offsets_(7, 0.0),
        low_speed_threshold_(0.1),
        low_speed_position_alpha_(0.05),
        nominal_position_alpha_(0.25),
        last_msg_()
    {
        joint_states_raw_sub_ = nh_.subscribe("joint_states_raw", 10, &JointStatesFilter::joint_states_raw_callback, this);
        joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
    }

    int run()
    {
        ros::spin();
        return 0;
    }

private:

    ros::NodeHandle nh_;
    ros::Subscriber joint_states_raw_sub_;
    ros::Publisher joint_states_pub_;

    sensor_msgs::JointState filtered_joint_state_;

    std::vector<double> position_offsets_;
    double low_speed_threshold_;
    double low_speed_position_alpha_;
    double nominal_position_alpha_;

    sensor_msgs::JointState::ConstPtr last_msg_;

    void init_joint_state(const sensor_msgs::JointState& first_msg)
    {
        filtered_joint_state_.header.seq = 0;
        filtered_joint_state_.header.frame_id = first_msg.header.frame_id;
        filtered_joint_state_.header.stamp = ros::Time::now();
        filtered_joint_state_.name = first_msg.name;
        filtered_joint_state_.position = first_msg.position;
        filtered_joint_state_.velocity = first_msg.velocity;
        filtered_joint_state_.effort = first_msg.effort;
    }

    void joint_states_raw_callback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        // last_msg_ = msg;

        auto interp = [](double from, double to, double a) { return (1.0 - a) * from + a * to; };

        if (filtered_joint_state_.name.empty()) {
            init_joint_state(*msg);
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
                filtered_joint_state_.position[i] = interp(filtered_joint_state_.position[i], msg->position[i], alpha);
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
