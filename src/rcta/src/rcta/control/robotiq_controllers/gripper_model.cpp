// standard includes
#include <cmath>
#include <limits>

// module includes
#include "gripper_model.h"

const double GripperModel::GRIPPER_MIN_POS_M_ = 0.0;
const double GripperModel::GRIPPER_MAX_POS_M_ = 0.0841;
const uint8_t GripperModel::EFFECTIVE_MIN_POS_VALUE_ = 230;
const uint8_t GripperModel::EFFECTIVE_MAX_POS_VALUE_ = 13;

const double GripperModel::GRIPPER_MIN_SPEED_MPS_ = 0.013;
const double GripperModel::GRIPPER_MAX_SPEED_MPS_ = 0.1;
const uint8_t GripperModel::EFFECTIVE_MIN_SPEED_VALUE_ = 0;
const uint8_t GripperModel::EFFECTIVE_MAX_SPEED_VALUE_ = 255;

const double GripperModel::GRIPPER_MIN_FORCE_N_ = 30.0;
const double GripperModel::GRIPPER_MAX_FORCE_N_ = 100.0;
const uint8_t GripperModel::EFFECTIVE_MIN_FORCE_VALUE_ = 0;
const uint8_t GripperModel::EFFECTIVE_MAX_FORCE_VALUE_ = 255;

const double GripperModel::GRIPPER_UPDATE_RATE_HZ_ = 200.0;

const double GripperModel::minimum_width() const
{
    return GRIPPER_MIN_POS_M_;
}

const double GripperModel::maximum_width() const
{
    return GRIPPER_MAX_POS_M_;
}

const double GripperModel::minimum_speed() const
{
    return GRIPPER_MIN_SPEED_MPS_;
}

const double GripperModel::maximum_speed() const
{
    return GRIPPER_MAX_SPEED_MPS_;
}

const double GripperModel::minimum_force() const
{
    return GRIPPER_MIN_FORCE_N_;
}

const double GripperModel::maximum_force() const
{
    return GRIPPER_MAX_FORCE_N_;
}

const double GripperModel::position_per_tick() const
{
    return (GRIPPER_MAX_POS_M_ - GRIPPER_MIN_POS_M_) / (double)std::numeric_limits<uint8_t>::max();
}

const double GripperModel::speed_per_tick() const
{
    return (GRIPPER_MAX_SPEED_MPS_ - GRIPPER_MIN_SPEED_MPS_) / (double)std::numeric_limits<uint8_t>::max();
}

const double GripperModel::force_per_tick() const
{
    return (GRIPPER_MAX_FORCE_N_ - GRIPPER_MIN_FORCE_N_) / (double)std::numeric_limits<uint8_t>::max();
}

const uint8_t GripperModel::effective_min_pos_value() const
{
    return EFFECTIVE_MIN_POS_VALUE_;
}

const uint8_t GripperModel::effective_max_pos_value() const
{
    return EFFECTIVE_MAX_POS_VALUE_;
}

const uint8_t GripperModel::effective_min_speed_value() const
{
    return EFFECTIVE_MIN_SPEED_VALUE_;
}

const uint8_t GripperModel::effective_max_speed_value() const
{
    return EFFECTIVE_MAX_SPEED_VALUE_;
}

const uint8_t GripperModel::effective_min_force_value() const
{
    return EFFECTIVE_MIN_FORCE_VALUE_;
}

const uint8_t GripperModel::effective_max_force_value() const
{
    return EFFECTIVE_MAX_FORCE_VALUE_;
}

double GripperModel::pos_value_to_width(uint8_t value) const
{
    return maximum_width() - position_per_tick() * (double)value;
}

double GripperModel::speed_value_to_speed(uint8_t value) const
{
    return minimum_speed() + speed_per_tick() * (double)value;
}

double GripperModel::force_value_to_force(uint8_t value) const
{
    return minimum_force() + force_per_tick() * (double)value;
}

static inline double clamp(double d, double min, double max)
{
    if (d < min) {
        return min;
    }
    else if (d > max) {
        return max;
    }
    else {
        return d;
    }
}

uint8_t GripperModel::width_to_pos_value(double width) const
{
    return (uint8_t)round((maximum_width() - clamp(width, GRIPPER_MIN_POS_M_, GRIPPER_MAX_POS_M_)) /
                          position_per_tick());
}

uint8_t GripperModel::speed_to_speed_value(double speed) const
{
    return (uint8_t)round((clamp(speed, GRIPPER_MIN_SPEED_MPS_, GRIPPER_MAX_SPEED_MPS_) - minimum_speed()) / speed_per_tick());
}

uint8_t GripperModel::force_to_force_value(double force) const
{
    return (uint8_t)round((clamp(force, GRIPPER_MIN_FORCE_N_, GRIPPER_MAX_FORCE_N_) - minimum_force()) / force_per_tick());
}

const double GripperModel::update_rate() const
{
    return GRIPPER_UPDATE_RATE_HZ_;
}
