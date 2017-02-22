-- NUMBER_OF_JOINTS: 7
-- DH_ANGLES:     0.0 0.0 0.0 0.0 0.0 0.0 0.0
-- DH_LENGTHS:    0.0 0.0 0.0 0.0 0.0 0.03175 0.01905
-- DH_OFFSETS:    0.1524 0.0 0.5461 0.06985 0.6223 0.0 0.0
-- DH_TWISTS:     3.14159 -3.14159 3.14159 -3.14159 3.14159 -3.14159 0.0
-- JOINT_TYPES:      0 0 0 0 0 0 0
-- RESOLUTIONS:      0.001 0.001 0.001 0.001 0.001 0.001 0.001
-- MIN_POSITIONS:    -3.054 -2.181 -3.054 -2.181 -1.57 -1.047 -1.047
-- MAX_POSITIONS:     3.054  2.181  3.054  2.181  1.57  1.047  1.047
-- MAX_VELOCITIES:      3.14159 3.14159 3.14159 3.14159 3.14159 3.14159 3.14159
-- MAX_ACCELERATIONS:   10.0 10.0 10.0 10.0 10.0 10.0 10.0
-- STOW_POSITIONS:      0.0 0.0 0.0 0.0 0.0 0.0 0.0

JointType = {
    Revolute = 0,
    Prismatic = 1
}

-- Note: all angles are measured in radians
HDTManipulator = {
    joints = {
        arm_1_shoulder_twist = {
            dh_angle = 0.0,
            dh_length = 0.0,
            dh_offset = 0.1524,
            dh_twist = 3.14159,
            joint_type = JointType.Revolute,
            resolution = 0.001,
            min_position = -3.054,
            max_position = 3.054,
            max_velocity = 3.14159,
            max_acceleration = 10.0,
            stow_position = 0.0
        },

        arm_2_shoulder_lift = {
            dh_angle = 0.0,
            dh_length = 0.0,
            dh_offset = 0.0,
            dh_twist = -3.14159,
            joint_type = JointType.Revolute,
            resolution = 0.001,
            min_position = -2.181,
            max_position = 2.181,
            max_velocity = 3.14159,
            max_acceleration = 10.0,
            stow_position = 0.0
        },

        arm_3_elbow_twist = {
            dh_angle = 0.0,
            dh_length = 0.0,
            dh_offset = 0.5461,
            dh_twist = 3.14159,
            joint_type = JointType.Revolute,
            resolution = 0.001,
            min_position = -3.054,
            max_position = 3.054,
            max_velocity = 3.14159,
            max_acceleration = 10.0,
            stow_position = 0.0
        },

        arm_4_elbow_lift = {
            dh_angle =  0.0,
            dh_length = 0.0,
            dh_offset = 0.06985,
            dh_twist = -3.14159,
            joint_type = JointType.Revolute,
            resolution = 0.001,
            min_position = -2.181,
            max_position = 2.181,
            max_velocity = 3.14159,
            max_acceleration = 10.0,
            stow_position = 0.0
        },

        arm_5_wrist_twist = {
            dh_angle = 0.0,
            dh_length = 0.0,
            dh_offset = 0.6223,
            dh_twist = 3.14159,
            joint_type = JointType.Revolute,
            resolution = 0.001,
            min_position = -1.57,
            max_position = 1.57,
            max_velocity = 3.14159,
            max_acceleration = 10.0,
            stow_position = 0.0
        },

        arm_6_wrist_lift = {
            dh_angle = 0.0,
            dh_length = 0.03175,
            dh_offset = 0.0,
            dh_twist = -3.14159,
            joint_type = JointType.Revolute,
            resolution = 0.001,
            min_position = -1.047,
            max_position = 1.047,
            max_velocity = 3.14159,
            max_acceleration = 10.0,
            stow_position = 0.0
        },

        arm_7_gripper_lift = {
            dh_angle = 0.0,
            dh_length = 0.01905,
            dh_offset = 0.0,
            dh_twist = 0.0,
            joint_type = JointType.Revolute,
            resolution = 0.001,
            min_position = -1.047,
            max_position = 1.047,
            max_velocity = 3.14159,
            max_acceleration = 10.0,
            stow_position = 0.0
        }
    }
}
