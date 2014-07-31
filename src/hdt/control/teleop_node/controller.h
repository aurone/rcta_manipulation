#ifndef hdt_Controller_h
#define hdt_Controller_h

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <LinuxJoystick.h>

namespace au
{
class ConfigBlock;
}

namespace hdt
{

/// @ brief A remote controller. Subclasses handle the translation step from
///         control index (as seen from the linux joystick driver) to the
///         control label (human-readable name of a control)
class Controller
{
public:

    typedef std::map<int, std::function<void()>> ButtonCommandToActionMap;
    typedef std::map<int, std::function<void(double)>> AxisCommandToActionMap;

    Controller();

    /// @brief Construct a Controller constructed from configuration.
    /// @param path Path to the device file for the controller (e.g., /dev/input/js0)
    /// @param mappings A config section that contains mappings for known joysticks
    bool init(const std::string& path,
              au::ConfigBlock& mappings,
              const ButtonCommandToActionMap& button_cmd_to_action,
              const AxisCommandToActionMap& axis_cmd_to_action);

    void process_events();

private:

    struct Button
    {
        int button_index;
        std::string name;
        int on_press_cmd;
        int on_release_cmd;
    };

    struct Axis
    {
        int axis_index;
        std::string name;
        int on_value_cmd;
    };

    std::vector<Button> buttons_;
    std::vector<Axis> axes_;

    LinuxJoystick joystick_;
    ButtonCommandToActionMap button_command_to_action_;
    AxisCommandToActionMap axis_command_to_action_;

    bool init_mapping(au::ConfigBlock& mapping);
    bool init_null_mapping();

    int button_index_to_pressed_command(unsigned int) const;
    int button_index_to_released_command(unsigned int) const;
    int axis_index_to_command(unsigned int) const;
};

} // namespace hdt

#endif
