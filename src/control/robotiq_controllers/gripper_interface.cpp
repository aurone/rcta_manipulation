#include <limits>
#include <thread>
#include "gripper_interface.h"

GripperInterface::GripperInterface(const std::shared_ptr<GripperConnection>& conn, double throttle_rate) :
    conn_(conn),
    model_(),
    transaction_id_(0),
    last_status_(),
    last_vel_(0xFF),
    last_force_(0xFF),
    last_request_stamp_(),
    timestamp_valid_(false),
    update_interval_us_((int)std::ceil(1e6 / (throttle_rate <= 0.0 ? model_.update_rate() : throttle_rate)))
{
    printf("Update interval is %ld us\n", update_interval_us_.count());
}

GripperInterface::~GripperInterface()
{
}

bool GripperInterface::activate()
{
    ActivateGripperRequest req(transaction_id_++);
    WriteRegistersGripperResponse res;
    return send_request(req, res);
}

bool GripperInterface::reset()
{
    ResetGripperRequest req(transaction_id_++);
    WriteRegistersGripperResponse res;
    return send_request(req, res);
}

bool GripperInterface::release()
{
    AutomaticReleaseGripperRequest req(transaction_id_++);
    WriteRegistersGripperResponse res;
    return send_request(req, res);
}

bool GripperInterface::stop()
{
    StopGripperRequest req(transaction_id_++);
    WriteRegistersGripperResponse res;
    return send_request(req, res);
}

bool GripperInterface::set_position(double width)
{
    GripperMotionRequest req(transaction_id_++, model_.width_to_pos_value(width), last_vel_, last_force_);
    WriteRegistersGripperResponse res;
    return send_request(req, res);
}

bool GripperInterface::set_speed(double speed)
{
    return set_speed(model_.speed_to_speed_value(speed));
}

bool GripperInterface::set_force(double force)
{
    return set_force(model_.force_to_force_value(force));
}

bool GripperInterface::set_position(uint8_t value)
{
    GripperMotionRequest req(transaction_id_++, value, last_vel_, last_force_);
    WriteRegistersGripperResponse res;
    return send_request(req, res);
}

bool GripperInterface::set_speed(uint8_t value)
{
    GripperDynamicsRequest req(transaction_id_++, value, last_force_);
    WriteRegistersGripperResponse res;
    if (send_request(req, res)) {
        last_vel_ = value;
        return true;
    }
    else {
        return true;
    }
}

bool GripperInterface::set_force(uint8_t value)
{
    GripperDynamicsRequest req(transaction_id_++, last_vel_, value);
    WriteRegistersGripperResponse res;
    if (send_request(req, res)) {
        last_force_ = value;
        return true;
    }
    else {
        return false;
    }
}

const bool GripperInterface::update()
{
    GripperStatusRequest req(transaction_id_++);
    std::unique_ptr<GripperStatusResponse> res(new GripperStatusResponse);
    if (!res) {
        return false;
    }

    if (send_request(req, *res)) {
        last_status_ = std::move(res);
        return true;
    }
    else {
        last_status_.reset();
        return false;
    }
}

const bool GripperInterface::is_activating() const
{
    if (!last_status_) {
        return false;
    }
    return last_status_->status() == Status::ActivationInProgress;
}

const bool GripperInterface::is_activated() const
{
    if (!last_status_) {
        return false;
    }
    return last_status_->status() == Status::ActivationComplete;
}

const bool GripperInterface::is_reset() const
{
    if (!last_status_) {
        return false;
    }
    return last_status_->status() == Status::Reset;
}

const bool GripperInterface::is_standby() const
{
    if (!last_status_) {
        return false;
    }
    return last_status_->gto_on() && last_status_->status() == Status::ActivationComplete;
}

Status GripperInterface::get_status() const
{
    if (!last_status_) {
        return Status::Invalid;
    }
    return last_status_->status();
}

const bool GripperInterface::fingers_in_motion() const
{
    if (!last_status_) {
        return false;
    }
    return last_status_->object_status() == ObjectStatus::FingersInMotion;
}

const bool GripperInterface::made_contact_closing() const
{
    if (!last_status_) {
        return false;
    }
    return last_status_->object_status() == ObjectStatus::FingersStoppedDueToContactWhileClosing;
}

const bool GripperInterface::made_contact_opening() const
{
    if (!last_status_) {
        return false;
    }
    return last_status_->object_status() == ObjectStatus::FingersStoppedDueToContactWhileOpening;
}

const bool GripperInterface::completed_positioning() const
{
    if (!last_status_) {
        return false;
    }
    return last_status_->object_status() == ObjectStatus::FingersAtRequestedPosition;
}

ObjectStatus GripperInterface::get_object_status() const
{
    if (!last_status_) {
        return ObjectStatus::Invalid;
    }
    return last_status_->object_status();
}

const bool GripperInterface::has_priority_fault() const
{
    if (!last_status_) {
        return false;
    }
    return is_priority_fault(last_status_->fault_status());
}

const bool GripperInterface::has_minor_fault() const
{
    if (!last_status_) {
        return false;
    }
    return is_minor_fault(last_status_->fault_status());
}

const bool GripperInterface::has_major_fault() const
{
    if (!last_status_) {
        return false;
    }
    return is_major_fault(last_status_->fault_status());
}

FaultStatus GripperInterface::get_fault_status() const
{
    if (!last_status_) {
        return FaultStatus::Invalid;
    }
    return last_status_->fault_status();
}

double GripperInterface::get_position() const
{
    if (!last_status_) {
        return -1.0;
    }
    else {
        return model_.pos_value_to_width(last_status_->pos());
    }
}

double GripperInterface::get_speed() const
{
    return model_.speed_value_to_speed(last_vel_);
}

double GripperInterface::get_force() const
{
    return model_.force_value_to_force(last_force_);
}

double GripperInterface::get_current() const
{
    if (!last_status_) {
        return -1.0;
    }
    return (double)last_status_->current() / 0.1;
}

double GripperInterface::get_requested_position() const
{
    if (!last_status_) {
        return -1.0;
    }
    return model_.pos_value_to_width(last_status_->pos_echo());
}

const bool GripperInterface::connected() const
{
    GripperStatusRequest req(transaction_id_++);
    GripperStatusResponse res;
    return send_request(req, res);
}

bool GripperInterface::send_request(const GripperRequest& req, GripperResponse& res) const
{
    using namespace std::chrono;

    time_point<high_resolution_clock> now = high_resolution_clock::now();

    if (!timestamp_valid_) {
        last_request_stamp_ = now;
        timestamp_valid_ = true;
    }
    else {
        time_point<high_resolution_clock> next_valid_time = last_request_stamp_ + update_interval_us_;
        if (now < next_valid_time) {
            std::this_thread::sleep_for(next_valid_time - now); // sleep until this request can be sent
        }
        last_request_stamp_ = high_resolution_clock::now();
    }

    return conn_->send_request(req, res);
}
