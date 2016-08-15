// standard includes
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>

// system includes
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// module includes
#include "gripper_simulator.h"

static double signd(double d)
{
    if (d > 0.0) return 1.0; else if (d < 0.0) return -1.0; else return 0.0;
}

static double clampedf(double d, double min, double max)
{
    if (d < min) return min; else if (d > max) return max; else return d;
};

GripperSimulator::GripperSimulator(boost::asio::io_service& io_service) :
    model_(),
    io_service_(io_service),
    timer_(io_service),
    loop_rate_hz_(200.0),
    routput_registers_({0, 0, 0, 0, INITIAL_VELOCITY_, INITIAL_FORCE_}),
    rinput_registers_({0, 0, 0, 0, 0, 0}),
    position_(model_.maximum_width()),
    target_pos_(0x00), // fully open
    activation_status_(ActivationStatusNotActivated),
    release_status_(ReleaseStatusNormal),
    status_(NotConnected),
    activating_(false),
    releasing_(false),
    // LEDs initialized to their powered-on and done-booting state
    controller_power_led_({ LED::Blue, LED::Continuous }),
    gripper_power_led_({ LED::Blue, LED::Continuous }),
    comms_led_({ LED::Green, LED::Blinking }),
    fault_led_({ LED::Red, LED::Off }),
    last_update_()
{
}

void GripperSimulator::start()
{
    printf("Starting simulator\n");
    timer_.expires_at(boost::posix_time::microsec_clock::universal_time());
    last_update_ = std::chrono::high_resolution_clock::now();
    update();
}

void GripperSimulator::update()
{
    // update, read_registers, and write_registers should be guaranteed mutually exclusive
    std::chrono::time_point<std::chrono::high_resolution_clock> update_time = std::chrono::high_resolution_clock::now();

    double time_delta = std::chrono::duration<double, std::ratio<1>>(update_time - last_update_).count();

    handle_state_transitions();
    handle_state_updates(time_delta);
    handle_output_registers();

    static double last_position = position_;
    if (last_position != position_) {
//        printf("new gripper position: %0.3f\n", position_);
        last_position = position_;
    }

    timer_.expires_at(timer_.expires_at() + boost::posix_time::microseconds((long)(1000000.0 * 1.0 / loop_rate_hz_)));
    timer_.async_wait(boost::bind(&GripperSimulator::update, this));
    last_update_ = update_time;
}

std::vector<uint8_t> GripperSimulator::read_registers(int offset, int num_registers) const
{
    printf("Robot Input Registers (%d read): [ ", num_registers);

    for (int i = 0; i < 2 * offset; ++i) {
        printf("%02x ", (unsigned)rinput_registers_[i]);
    }

    for (int i = 2 * offset; i < 2 * offset + 2 * num_registers; ++i) {
        printf("\x1b[36;1m%02x\x1b[m ", (unsigned)rinput_registers_[i]);
    }

    for (int i = 2 * offset + 2 * num_registers; i < NUM_ROBOT_INPUT_REGISTERS_; ++i) {
        printf("%02x ", (unsigned)rinput_registers_[i]);
    }

    printf("]\n");
    fflush(stdout);

    return std::vector<uint8_t>(&rinput_registers_[2 * offset], &rinput_registers_[2 * offset + 2 * num_registers]);
}

static std::string to_string(std::vector<uint8_t>::const_iterator b, std::vector<uint8_t>::const_iterator e)
{
    std::stringstream ss;
    ss.fill('0');
    ss << std::setw(2) << std::hex;
    ss << "[ ";
    for (std::vector<uint8_t>::const_iterator i = b; i != e; ++i) {
        ss << std::hex << std::setw(2) << (unsigned int)*i << ' ';
    }
    ss << ']';
    return ss.str();
}

void GripperSimulator::write_registers(int offset, const std::vector<uint8_t>& data)
{
    memcpy((void *)&routput_registers_[2 * offset], (const void*)data.data(), data.size());

    printf("Robot Output Registers (%zd written): [ ", data.size() >> 1);
    for (int i = 0; i < 2 * offset; ++i) {
        printf("%02x ", (unsigned)routput_registers_[i]);
    }

    for (int i = 2 * offset; i < 2 * offset + (int)data.size(); ++i) {
        printf("\x1b[32;1m%02x\x1b[m ", (unsigned)routput_registers_[i]);
    }

    for (int i = 2 * offset + (int)data.size(); i < NUM_ROBOT_OUTPUT_REGISTERS_; ++i) {
        printf("%02x ", (unsigned)routput_registers_[i]);
    }

    printf("]\n");
    fflush(stdout);
}

std::string GripperSimulator::to_string(ActivationStatus s)
{
    switch (s) {
    case ActivationStatusNotActivated:
        return "NotActivated";
    case ActivationStatusClosing:
        return "Closing";
    case ActivationStatusOpening:
        return "Opening";
    case ActivationStatusActivated:
        return "Activated";
    }
}

std::string GripperSimulator::to_string(Status s)
{
    switch (s) {
    case NotConnected:
        return "NotConnected";
    case Reset:
        return "Reset";
    case Activating:
        return "Activating";
    case Activated:
        return "Activated";
    case Releasing:
        return "Releasing";
    case MajorFault:
        return "MajorFault";
    }
}

bool GripperSimulator::rACT() const
{
    return routput_registers_[0] & 0x01;
}

bool GripperSimulator::rRS1() const
{
    return routput_registers_[0] & 0x02;
}

bool GripperSimulator::rRS2() const
{
    return routput_registers_[0] & 0x04;
}

bool GripperSimulator::rGTO() const
{
    return routput_registers_[0] & 0x08;
}

bool GripperSimulator::rATR() const
{
    return routput_registers_[0] & 0x10;
}

bool GripperSimulator::rRS3() const
{
    return routput_registers_[0] & 0x20;
}

bool GripperSimulator::rRS4() const
{
    return routput_registers_[0] & 0x40;
}

bool GripperSimulator::rRS5() const
{
    return routput_registers_[0] & 0x80;
}

uint8_t GripperSimulator::rRS6() const
{
    return routput_registers_[1];
}

uint8_t GripperSimulator::rRS7() const
{
    return routput_registers_[2];
}

uint8_t GripperSimulator::rPR() const
{
    return routput_registers_[3];
}

uint8_t GripperSimulator::rSP() const
{
    return routput_registers_[4];
}

uint8_t GripperSimulator::rFR() const
{
    return routput_registers_[5];
}

void GripperSimulator::set_bit(uint8_t& o, uint8_t pos, bool on)
{
    if (on) {
        o |= (0x01 << pos);
    }
    else {
        o &= ~(0x01 << pos);
    }
}

bool GripperSimulator::connected() const
{
    return true;
}

bool GripperSimulator::overcurrent() const
{
    return false;
}

bool GripperSimulator::finished_activate_opening() const
{
    return activation_status_ == ActivationStatusActivated;
}

bool GripperSimulator::finished_activate_closing() const
{
    return activation_status_ > ActivationStatusClosing;
}

bool GripperSimulator::finished_activating() const
{
    return activation_status_ == ActivationStatusActivated;
}

bool GripperSimulator::finished_release_opening() const
{
    return false;
}

bool GripperSimulator::finished_releasing() const
{
    return false;
}

void GripperSimulator::handle_state_transitions()
{
    // todo: check for rATR vs rACT priority??!?!?!"
    Status prev_status = status_;

    switch (status_) {
    case NotConnected:
        // note: blindly reset without overcurrent check; will be
        // checked on next Reset iteration and moved into MajorFault
        if (connected()) {
            status_ = Reset;
        }
        break;
    case Reset:
        //> Activating, Releasing, MajorFault
        // todo: check for overcurrent
        if (overcurrent()) {
            status_ = MajorFault;
        }
        else if (rATR()) {
            status_ = Releasing;
        }
        else if (rACT()) {
            status_ = Activating;
        }
        else if (rGTO()) {
            // todo: trigger priority fault 0x07
        }
        else {
            // idle
        }
        break;
    case Activating: // todo: is a priority fault generated for trying to reset or release the gripper while activating?
        //> Activated, MajorFault
        if (overcurrent()) {
            status_ = MajorFault;
        }
        else if (finished_activating()) {
            status_ = Activated;
        }
        else if (rGTO()) {
            // todo: trigger priority fault 0x05
        }
        break;
    case Activated:
        //> Releasing, Reset, MajorFault

        // check for activate bit
        if (overcurrent()) {
            status_ = MajorFault;
        }
        else if (rATR()) {
            status_ = Releasing;
        }
        else if (!rACT()) {
            status_ = Reset;
        }
        else {
            // proceed with current commands
        }

        break;
    case Releasing:
        //> MajorFault
        if (overcurrent() || finished_releasing()) {
            status_ = MajorFault;
        }
        break;
    case MajorFault:
        //> Reset
        if (!rACT()) {
            // note: blindly reset without overcurrent check; will be
            // checked on next Reset iteration and moved back into MajorFault
            status_ = Reset;
        }
        break;
    }

    if (status_ != prev_status) {
        printf("Transitioning from %s --> %s\n", to_string(prev_status).c_str(), to_string(status_).c_str());
    }
}

void GripperSimulator::handle_state_updates(double time_delta)
{
    switch (status_) {
    case NotConnected:
        comms_led_.status = LED::Blinking;
        break;
    case Reset:
        comms_led_.status = LED::Continuous;
        fault_led_.status = LED::Off;
        break;
    case Activating:
        if (activation_status_ == ActivationStatusNotActivated) {
            activation_status_ = ActivationStatusClosing;
        }

        if (!finished_activate_closing()) {
            position_ -= model_.maximum_speed() * time_delta;

            if (position_ < 0.0) {
                position_ = 0.0;
                activation_status_ = ActivationStatusOpening;
            }
        }
        else if (!finished_activate_opening()) {
            position_ += model_.maximum_speed() * time_delta;

            if (position_ > model_.maximum_width()) {
                position_ = model_.maximum_width();
                activation_status_ = ActivationStatusActivated;
            }
        }
        break;
    case Activated:
    {
        if (rGTO()) {
            // set target position
            target_pos_ = rPR();
        }

        double target_position = model_.pos_value_to_width(target_pos_);
        if (target_position != position_) { // equality ok because of overshoot correction
            double speed = model_.speed_value_to_speed(rSP());

            double dir = signd(target_position - position_);
            position_ += dir * speed * time_delta;
            if (dir != signd(target_position - position_)) {
                position_ = target_position; // don't overshoot target_position
            }
            position_ = clampedf(position_, model_.minimum_width(), model_.maximum_width());
            printf("x(t+1) = %0.3f\n", position_);
        }

    }   break;
    case Releasing:
        fault_led_.status = LED::Blinking; // minor fault indicator
        if (!finished_release_opening()) {
            // todo: move towards fully opened
        }
        else {
            // todo: finish releasing
        }
        break;
    case MajorFault:
        fault_led_.status = LED::Continuous;
        break;
    }
}

void GripperSimulator::handle_output_registers()
{
    set_bit(rinput_registers_[0], 0, rACT());
    set_bit(rinput_registers_[0], 1, false);
    set_bit(rinput_registers_[0], 2, false);
    set_bit(rinput_registers_[0], 3, rGTO());

    switch (status_) {
    case NotConnected:
    case Reset:
    case Releasing:
    case MajorFault: // note: this case is not entirely true; bits 4 and 5 should be set if the fault case was entered from the activated state
        set_bit(rinput_registers_[0], 4, false);
        set_bit(rinput_registers_[0], 5, false);
        break;
    case Activating:
        set_bit(rinput_registers_[0], 4, false);
        set_bit(rinput_registers_[0], 5, true);
        break;
    case Activated:
        set_bit(rinput_registers_[0], 4, true);
        set_bit(rinput_registers_[0], 5, true);
        break;
    }

    if (rGTO() && position_ == model_.pos_value_to_width(target_pos_)) {
        set_bit(rinput_registers_[0], 6, true);
        set_bit(rinput_registers_[0], 7, true);
    }
    else {
        set_bit(rinput_registers_[0], 6, false);
        set_bit(rinput_registers_[0], 7, false);
    }

    rinput_registers_[1] = 0x00;

    // todo: set based on fault status
    set_bit(rinput_registers_[2], 0, false);
    set_bit(rinput_registers_[2], 1, false);
    set_bit(rinput_registers_[2], 2, false);
    set_bit(rinput_registers_[2], 3, false);
    set_bit(rinput_registers_[2], 4, false);
    set_bit(rinput_registers_[2], 5, false);
    set_bit(rinput_registers_[2], 6, false);
    set_bit(rinput_registers_[2], 7, false);

    rinput_registers_[3] = rPR();

//    double target_position = model_.pos_value_to_width(target_pos_);
    uint8_t cpos = model_.width_to_pos_value(position_);
    rinput_registers_[4] = cpos;

    rinput_registers_[5] = 0x01; // todo: current
}
