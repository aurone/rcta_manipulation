#ifndef GripperSimulator_h
#define GripperSimulator_h

#include <cstdint>
#include <chrono>
#include <memory>
#include <boost/asio.hpp>
#include "gripper_server.h"
#include "gripper_model.h"

// TODO: catch LED status for minor faults

class GripperSimulator
{
public:

    GripperSimulator(boost::asio::io_service& io_service);

    void start();
    void update();

    /// @brief Read 
    std::vector<uint8_t> read_registers(int offset, int num_registers) const;
    void write_registers(int offset, const std::vector<uint8_t>& data);

private:

    static const uint8_t INITIAL_VELOCITY_ = 0xFF;
    static const uint8_t INITIAL_FORCE_ = 0xFF;

    GripperModel model_;

    // simulate the 200 Hz controller loop on the Robotiq gripper
    boost::asio::io_service& io_service_;
    boost::asio::deadline_timer timer_;

    double loop_rate_hz_;

    static const int NUM_ROBOT_OUTPUT_REGISTERS_ = 6;
    static const int NUM_ROBOT_INPUT_REGISTERS_ = 6;

    uint8_t routput_registers_[NUM_ROBOT_OUTPUT_REGISTERS_];
    uint8_t rinput_registers_[NUM_ROBOT_INPUT_REGISTERS_];

    double position_;
    uint8_t target_pos_;

    enum ActivationStatus
    {
        ActivationStatusNotActivated = 0,
        ActivationStatusClosing,
        ActivationStatusOpening,
        ActivationStatusActivated
    } activation_status_;
    static std::string to_string(ActivationStatus s);

    enum ReleaseStatus
    {
        ReleaseStatusNormal = 0,
        ReleaseStatusOpening,
        ReleaseStatusReleased
    } release_status_;

    enum Status
    {
        NotConnected = 0,
        Reset,
        Activating,
        Activated,
        Releasing,
        MajorFault
    } status_;
    static std::string to_string(Status s);

    bool activating_;
    bool releasing_;

    struct LED
    {
        enum Color { Blue, Green, Red } color;
        enum Status { Off, Blinking, Continuous } status;
    } controller_power_led_, gripper_power_led_, comms_led_, fault_led_;

    std::chrono::time_point<std::chrono::high_resolution_clock> last_update_;

    // bits from routput_register0_
    inline bool rACT() const;
    inline bool rRS1() const;
    inline bool rRS2() const;
    inline bool rGTO() const;
    inline bool rATR() const;
    inline bool rRS3() const;
    inline bool rRS4() const;
    inline bool rRS5() const;

    inline uint8_t rRS6() const;
    inline uint8_t rRS7() const;

    inline uint8_t rPR() const;
    inline uint8_t rSP() const;
    inline uint8_t rFR() const;

    inline void set_bit(uint8_t& o, uint8_t pos, bool on);

    inline bool connected() const;
    inline bool overcurrent() const;
    inline bool finished_activate_opening() const;
    inline bool finished_activate_closing() const;
    inline bool finished_activating() const;
    inline bool finished_release_opening() const;
    inline bool finished_releasing() const;

    void handle_state_transitions();
    void handle_state_updates(double time_delta);
    void handle_output_registers();
};

#endif
