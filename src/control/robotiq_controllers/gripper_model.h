#ifndef GripperModel_h
#define GripperModel_h

#include <cstdint>

class GripperModel
{
public:

    const double minimum_width() const;
    const double maximum_width() const;
    const double minimum_speed() const;
    const double maximum_speed() const;
    const double minimum_force() const;
    const double maximum_force() const;

    const double position_per_tick() const;

    /// @brief Return the approximate change in speed per command tick.
    const double speed_per_tick() const;

    /// @brief Return the approximate change in force per command tick.
    const double force_per_tick() const;

    /// @brief Return the value at which and above, when the gripper is commanded to one of these values, the fingertips
    ///        will be at their minimum width apart.
    const uint8_t effective_min_pos_value() const;

    /// @brief Return the value at which and below, when the gripper is commanded to one of these values, the fingertips
    ///        will be at their maximum width apart
    const uint8_t effective_max_pos_value() const;

    const uint8_t effective_min_speed_value() const;
    const uint8_t effective_max_speed_value() const;
    const uint8_t effective_min_force_value() const;
    const uint8_t effective_max_force_value() const;

    double pos_value_to_width(uint8_t value) const;
    double speed_value_to_speed(uint8_t value) const;
    double force_value_to_force(uint8_t value) const;

    uint8_t width_to_pos_value(double width) const;
    uint8_t speed_to_speed_value(double speed) const;
    uint8_t force_to_force_value(double force) const;

private:

    static const double GRIPPER_MAX_POS_M_;
    static const double GRIPPER_MIN_POS_M_;
    static const uint8_t EFFECTIVE_MIN_POS_VALUE_;
    static const uint8_t EFFECTIVE_MAX_POS_VALUE_;

    static const double GRIPPER_MIN_SPEED_MPS_;
    static const double GRIPPER_MAX_SPEED_MPS_;
    static const uint8_t EFFECTIVE_MIN_SPEED_VALUE_;
    static const uint8_t EFFECTIVE_MAX_SPEED_VALUE_;

    static const double GRIPPER_MIN_FORCE_N_;
    static const double GRIPPER_MAX_FORCE_N_;
    static const uint8_t EFFECTIVE_MIN_FORCE_VALUE_;
    static const uint8_t EFFECTIVE_MAX_FORCE_VALUE_;
};

#endif
