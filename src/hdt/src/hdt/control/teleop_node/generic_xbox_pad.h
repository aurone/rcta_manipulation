#ifndef hdt_GenericXboxPad_h
#define hdt_GenericXboxPad_h

#include <map>
#include <string>
#include "controller.h"

namespace au
{
class ConfigBlock;
}

namespace hdt
{

class GenericXboxPad : public Controller
{
public:

    static std::shared_ptr<GenericXboxPad> buildFromConfig(const std::shared_ptr<LinuxJoystick>& joystick,
                                                           const au::ConfigBlock& luaconf,
                                                           const ButtonCommandToActionMap& button_cmd_to_action,
                                                           const AxisCommandToActionMap& action_cmd_to_action);

    int button_index_to_pressed_command(int button_index);
    int button_index_to_released_command(int button_index);
    int axis_index_to_command(int axis_index);

private:

    GenericXboxPad(const std::shared_ptr<LinuxJoystick>& joystick,
                   const ButtonCommandToActionMap& button_cmd_to_action,
                   const AxisCommandToActionMap& axis_cmd_to_action);

    std::map<std::string, int> button_label_to_index_;
    std::map<std::string, int> axis_label_to_index_;
};

} // namespace hdt

#endif
