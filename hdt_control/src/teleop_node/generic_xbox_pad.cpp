#include "generic_xbox_pad.h"

// project includes
#include <config_block/config_block.h>

namespace hdt
{

std::shared_ptr<GenericXboxPad> GenericXboxPad::buildFromConfig(
    const std::shared_ptr<LinuxJoystick>& joystick,
    const au::ConfigBlock& luaconf,
    const ButtonCommandToActionMap& button_cmd_to_action,
    const AxisCommandToActionMap& axis_cmd_to_action)
{
   return std::shared_ptr<GenericXboxPad>(new GenericXboxPad(joystick, button_cmd_to_action, axis_cmd_to_action));
}

int GenericXboxPad::button_index_to_pressed_command(int button_index)
{
    return -1;
}

int GenericXboxPad::button_index_to_released_command(int button_index)
{
    return -1;
}

int GenericXboxPad::axis_index_to_command(int axis_index)
{
    return -1;
}

GenericXboxPad::GenericXboxPad(
    const std::shared_ptr<LinuxJoystick>& joystick,
    const ButtonCommandToActionMap& button_cmd_to_action,
    const AxisCommandToActionMap& axis_cmd_to_action)
:
    Controller(joystick, button_cmd_to_action, axis_cmd_to_action),
    button_label_to_index_
    {
        { "A",                0 },
        { "B",                1 },
        { "X",                2 },
        { "Y",                3 },
        { "RightBumper",      4 },
        { "LeftBumper",       5 },
        { "Select",           6 },
        { "Start",            7 },
        { "Xbox",             8 },
        { "LeftThumbstick",   9 },
        { "RightThumbstick", 10 },
    },
    axis_label_to_index_
    {
        { "LeftStickVertical",        0 },
        { "LeftStickHorizontal",      1 },
        { "RightStickVertical",       2 },
        { "RightStickHorizontal",     3 },
        { "DPadLeft",                 4 },
        { "DPadRight",                5 },
        { "DPadUp",                   6 },
        { "DPadDown",                 7 },
        { "LeftTrigger",              8 },
        { "RightTrigger",            10 },
    }
{
}

} // namespace hdt
