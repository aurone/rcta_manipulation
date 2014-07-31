#include <sstream>
#include <string>
#include <ros/ros.h>
#include <config_block/config_block.h>
#include "controller.h"
// #include "generic_xbox_pad.h"

namespace hdt
{

Controller::Controller() :
    joystick_(),
    button_command_to_action_(),
    axis_command_to_action_()
{
}

bool Controller::init(
    const std::string& path,
    au::ConfigBlock& mappings,
    const ButtonCommandToActionMap& button_cmd_to_action,
    const AxisCommandToActionMap& axis_cmd_to_action)
{
    if (!joystick_.Initialize(path)) {
        return false;
    }

    au::ConfigBlock mapping_block;
    if (mappings.get(joystick_.Name(), mapping_block)) {
        if (!init_mapping(mapping_block)) {
            ROS_ERROR("Failed to initialize mapping for joystick '%s'", joystick_.Name().c_str());
            return false;
        }
    }
    else {
        ROS_WARN("Failed to retrieve mapping for '%s'", joystick_.Name().c_str());
        init_null_mapping();
    }

    button_command_to_action_ = button_cmd_to_action;
    axis_command_to_action_ = axis_cmd_to_action;
    return true;
}

void Controller::process_events()
{
    joystick_.ClearButtons();
    joystick_.Update();

    std::stringstream ss;
    ss << "Axes: [ ";
    for (unsigned i = 0; i < joystick_.NumAxes(); ++i) {
        ss << joystick_.Axis(i) << ' ';
    }
    ss << ']';
    ss << ' ';
    ss << "Buttons: [ ";
    for (unsigned i = 0; i < joystick_.NumButtons(); ++i) {
        ss << joystick_.Button(i) << ' ';
    }
    ss << ']';
    ROS_INFO("%s", ss.str().c_str());

    for (unsigned i = 0; i < joystick_.NumButtons(); ++i) {
        if (joystick_.PressedButton(i)) {
            int button_command = button_index_to_pressed_command(i);
            if (button_command < 0) {
                continue; // do nothing
            }

            auto it = button_command_to_action_.find(button_command);
            if (it == button_command_to_action_.end()) {
                // ERROR
            }
            else {
                (it->second)();
            }
        }

        if (joystick_.ReleasedButton(i)) {
            int button_command = button_index_to_released_command(i);
            if (button_command < 0) {
                continue; // do nothing
            }

            auto it = button_command_to_action_.find(button_command);
            if (it == button_command_to_action_.end()) {
                // ERROR
            }
            else {
                (it->second)();
            }
        }
    }

    // TODO: only call axis commands if value has changed?
    for (unsigned i = 0; i < joystick_.NumAxes(); ++i) {
        float val = joystick_.Axis(i);
        int axis_command = axis_index_to_command(i);
        if (axis_command < 0) {
            continue;
        }

        auto it = axis_command_to_action_.find(axis_command);
        if (it == axis_command_to_action_.end()) {
            // ERROR
        }
        else {
            (it->second)((double)val);
        }
    }
}

bool Controller::init_mapping(au::ConfigBlock& mapping)
{
    au::ConfigBlock buttons_conf;
    au::ConfigBlock axes_conf;
    if (!mapping.get("Buttons", buttons_conf) || !mapping.get("Axes", axes_conf)) {
        ROS_ERROR("Mapping is malformed. Expected 'Buttons' and 'Axes' sections");
        return false;
    }

    buttons_.reserve(joystick_.NumButtons());
    for (int i = 0; i < joystick_.NumButtons(); ++i) {
        std::string intstr = std::to_string(i);
        au::ConfigBlock button_conf;

        Button b;
        if (!buttons_conf.get(intstr, button_conf)) {
            ROS_WARN("Expected button configuration at Buttons[\"%d\"]", i);
            b = { i, "", -1, -1 };
            buttons_.push_back(b);
        }
        else {
            b = { i, "", -1, -1 };
            if (!button_conf.get("Name", b.name)) {
                ROS_WARN("Expected 'Name' key");
            }
            if (!button_conf.get("OnPress", b.on_press_cmd)) {
                ROS_WARN("Expected 'OnPress' key");
            }
            if (!button_conf.get("OnRelease", b.on_release_cmd)) {
                ROS_WARN("Expected 'OnRelease' key");
            }

            ROS_DEBUG("Button %d: Name: %s, OnPress ID: %d, OnRelease ID: %d", b.button_index, b.name.c_str(), b.on_press_cmd, b.on_release_cmd);
        }

        buttons_.push_back(b);
    }

    axes_.reserve(joystick_.NumAxes());
    for (int i = 0; i < joystick_.NumAxes(); ++i) {
        std::string intstr = std::to_string(i);
        au::ConfigBlock axis_conf;

        Axis a;
        if (!axes_conf.get(intstr, axis_conf)) {
            ROS_WARN("Expected axis configuration at Axes[\"%d\"]", i);
            a = { i, "", -1 };
        }
        else {
            a = { i, "", -1 };
            if (!axis_conf.get("Name", a.name)) {
                ROS_WARN("Expected 'Name' key");
            }
            if (!axis_conf.get("OnValue", a.on_value_cmd)) {
                ROS_WARN("Expected 'OnValue' key");
            }
        }
        axes_.push_back(a);
    }

    return true;
}

bool Controller::init_null_mapping()
{
    buttons_.reserve(joystick_.NumButtons());
    for (int i = 0; i < joystick_.NumButtons(); ++i) {
        Button b = { i, "", -1, -1 };
        buttons_.push_back(b);
    }
    axes_.reserve(joystick_.NumAxes());
    for (int i = 0; i < joystick_.NumAxes(); ++i) {
        Axis a = { i, "", -1 };
        axes_.push_back(a);
    }
    return true;
}

int Controller::button_index_to_pressed_command(unsigned int button_index) const
{
    assert(button_index >= 0 && button_index < buttons_.size());
    return buttons_[button_index].on_press_cmd;
}

int Controller::button_index_to_released_command(unsigned int button_index) const
{
    assert(button_index >= 0 && button_index < buttons_.size());
    return buttons_[button_index].on_release_cmd;
}

int Controller::axis_index_to_command(unsigned int axis_index) const
{
    assert(axis_index >= 0 && axis_index < axes_.size());
    return axes_[axis_index].on_value_cmd;
}


} // namespace hdt
