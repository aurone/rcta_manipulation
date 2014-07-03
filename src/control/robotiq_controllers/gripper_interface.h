#ifndef GripperInterface_h
#define GripperInterface_h

#include <memory>
#include "gripper_connection.h"
#include "gripper_model.h"
#include "gripper_status.h"

class GripperConnection;
class GripperStatusResponse;

class GripperInterface
{
public:

    GripperInterface(const std::shared_ptr<GripperConnection>& conn);
    ~GripperInterface();

    const bool connected() const;

    /// @{ Methods corresponding to Robot Output Registers and Functionalities
    bool activate();
    bool reset();
    bool release();
    bool stop();

    bool set_position(double width);
    bool set_speed(double speed);
    bool set_force(double force);

    bool set_position(uint8_t value);
    bool set_speed(uint8_t value);
    bool set_force(uint8_t value);
    /// @}

    /// @{ Methods corresponding to Robot Input Registers and Status
    const bool update();

    const bool is_activating() const;
    const bool is_activated() const;
    const bool is_reset() const;
    const bool is_standby() const;

    Status get_status() const;

    const bool fingers_in_motion() const;
    const bool made_contact_closing() const;
    const bool made_contact_opening() const;
    const bool completed_positioning() const;

    ObjectStatus get_object_status() const;

    const bool has_priority_fault() const;
    const bool has_minor_fault() const;
    const bool has_major_fault() const;

    FaultStatus get_fault_status() const;

    double get_position() const;
    double get_speed() const; ///< The current speed setting on the gripper
    double get_force() const; ///< The current force setting on the gripper
    double get_current() const; // in mA
    double get_requested_position() const;
    /// @}

    const GripperModel& model() const { return model_; }

private:

    GripperModel model_;

    mutable uint16_t transaction_id_;

    uint8_t last_vel_, last_force_;

    std::shared_ptr<GripperConnection> conn_;
    std::unique_ptr<GripperStatusResponse> last_status_;
};

#endif
