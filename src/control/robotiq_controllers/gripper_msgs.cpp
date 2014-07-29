#include <cstring>
#include "gripper_msgs.h"

const uint16_t GripperRequest::PROTOCOL_ID = 0x0000;
const uint8_t GripperRequest::SLAVE_ID = 0x02;
const uint8_t GripperRequest::FUNCTION_CODE_READ_INPUT_REGISTERS = 0x04;
const uint8_t GripperRequest::FUNCTION_CODE_PRESET_MULTIPLE_REGISTERS = 0x10;

const uint16_t GripperRequest::ADDRESS_REGISTER0 = 0x0000;
const uint16_t GripperRequest::ADDRESS_REGISTER1 = 0x0001;
const uint16_t GripperRequest::ADDRESS_REGISTER2 = 0x0002;

////////////////////////////////////////////////////////////////////////////////
// GripperRequest
////////////////////////////////////////////////////////////////////////////////

GripperRequest::GripperRequest(uint16_t trans_id) :
    trans_id_(trans_id)
{
}

GripperRequest::~GripperRequest()
{
}

std::vector<uint8_t>
GripperRequest::create_msg_data(uint8_t* payload, uint16_t payload_length, OpType op) const
{
    const size_t HEADER_LENGTH = 8;
    std::vector<uint8_t> msg_data;
    msg_data.resize(payload_length + HEADER_LENGTH);

    msg_data[0] = hi(trans_id());           msg_data[1] = lo(trans_id());
    msg_data[2] = hi(PROTOCOL_ID);          msg_data[3] = lo(PROTOCOL_ID);
    msg_data[4] = hi(payload_length + 2);   msg_data[5] = lo(payload_length + 2);
    msg_data[6] = SLAVE_ID;
    msg_data[7] = op == OpType::Read ? FUNCTION_CODE_READ_INPUT_REGISTERS : FUNCTION_CODE_PRESET_MULTIPLE_REGISTERS;
    memcpy((void*)(msg_data.data() + HEADER_LENGTH), (const void*)payload, (size_t)payload_length);

    return msg_data;
}

////////////////////////////////////////////////////////////////////////////////
// GripperResponse
////////////////////////////////////////////////////////////////////////////////

GripperResponse::~GripperResponse()
{
}

uint16_t GripperResponse::trans_id() const
{
    return combine(response_msg_[0], response_msg_[1]); 
}

uint16_t GripperResponse::protocol_id() const
{
    return combine(response_msg_[2], response_msg_[3]); 
}

uint16_t GripperResponse::length() const
{
    return combine(response_msg_[4], response_msg_[5]); 
}

uint8_t GripperResponse::slave_id() const
{
    return response_msg_[6]; 
}

uint8_t GripperResponse::op_code() const
{
    return response_msg_[7]; 
}

////////////////////////////////////////////////////////////////////////////////
// ActivateGripperRequest
////////////////////////////////////////////////////////////////////////////////

ActivateGripperRequest::ActivateGripperRequest(uint16_t trans_id) :
    GripperRequest(trans_id)
{
}

std::vector<uint8_t>
ActivateGripperRequest::construct_payload() const
{
    uint8_t buff[] = { hi(ADDRESS_REGISTER0), lo(ADDRESS_REGISTER0),
                       0x00, 0x03,
                       0x06,
                       0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };

    return create_msg_data(buff, sizeof(buff), OpType::Write);
}

std::string ActivateGripperRequest::name() const
{
    return "Activate Gripper Request";
}

////////////////////////////////////////////////////////////////////////////////
// ResetGripperRequest
////////////////////////////////////////////////////////////////////////////////

ResetGripperRequest::ResetGripperRequest(uint16_t trans_id) :
    GripperRequest(trans_id)
{
}

std::vector<uint8_t>
ResetGripperRequest::construct_payload() const
{
    uint8_t buff[] = { hi(ADDRESS_REGISTER0), lo(ADDRESS_REGISTER0),
                       0x00, 0x03,
                       0x06,
                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    return create_msg_data(buff, sizeof(buff), OpType::Write);
}

std::string ResetGripperRequest::name() const
{
    return "Reset Gripper Request";
}

////////////////////////////////////////////////////////////////////////////////
// StopGripperRequest
////////////////////////////////////////////////////////////////////////////////

StopGripperRequest::StopGripperRequest(uint16_t trans_id) :
    GripperRequest(trans_id)
{
}

std::vector<uint8_t>
StopGripperRequest::construct_payload() const
{
    uint8_t buff[] = { hi(ADDRESS_REGISTER0), lo(ADDRESS_REGISTER0),
                       0x00, 0x03,
                       0x06,
                       0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
    return create_msg_data(buff, sizeof(buff), OpType::Write);
}

std::string StopGripperRequest::name() const
{
    return "Stop Gripper Request";
}

////////////////////////////////////////////////////////////////////////////////
// AutomaticReleaseGripperRequest
////////////////////////////////////////////////////////////////////////////////

AutomaticReleaseGripperRequest::AutomaticReleaseGripperRequest(uint16_t trans_id) :
    GripperRequest(trans_id)
{
}

std::vector<uint8_t>
AutomaticReleaseGripperRequest::construct_payload() const
{
    uint8_t buff[] = { hi(ADDRESS_REGISTER0), lo(ADDRESS_REGISTER0),
                       0x00, 0x03,
                       0x06,
                       0x11, 0x00, 0x00, 0x00, 0x00, 0x00 };
    return create_msg_data(buff, sizeof(buff), OpType::Write);
}

std::string AutomaticReleaseGripperRequest::name() const
{
    return "Automatic Release Gripper Request";
}

////////////////////////////////////////////////////////////////////////////////
// GripperPositionRequest
////////////////////////////////////////////////////////////////////////////////

GripperPositionRequest::GripperPositionRequest(uint16_t trans_id, uint8_t pos) :
    GripperRequest(trans_id),
    pos_(pos)
{
}

std::vector<uint8_t>
GripperPositionRequest::construct_payload() const
{
    uint8_t buff[] = { hi(ADDRESS_REGISTER0), lo(ADDRESS_REGISTER0),
                       0x00, 0x02,
                       0x04,
                       0x09, 0x00,
                       0x00, pos_ };
    return create_msg_data(buff, sizeof(buff), OpType::Write);
}

std::string GripperPositionRequest::name() const
{
    return "Gripper Position Request";
}

GripperStatusRequest::GripperStatusRequest(uint16_t trans_id) :
    GripperRequest(trans_id)
{
}

GripperDynamicsRequest::GripperDynamicsRequest(uint16_t trans_id, uint8_t vel, uint8_t force) :
    GripperRequest(trans_id),
    vel_(vel),
    force_(force)
{
}

std::vector<uint8_t>
GripperDynamicsRequest::construct_payload() const
{
    uint8_t buff[] = { hi(ADDRESS_REGISTER2), lo(ADDRESS_REGISTER2),
                       0x00, 0x01,
                       0x02,
                       vel_, force_ };
    return create_msg_data(buff, sizeof(buff), OpType::Write);
}

std::string GripperDynamicsRequest::name() const
{
    return "Gripper Dynamics Request";
}

GripperMotionRequest::GripperMotionRequest(uint16_t trans_id, uint8_t pos, uint8_t vel, uint8_t force) :
    GripperRequest(trans_id),
    pos_(pos),
    vel_(vel),
    force_(force)
{
}

std::vector<uint8_t>
GripperMotionRequest::construct_payload() const
{
    uint8_t buff[] = { hi(ADDRESS_REGISTER0), lo(ADDRESS_REGISTER0),
                       0x00, 0x03,
                       0x06,
                       0x09, 0x00,
                       0x00, pos_,
                       vel_, force_ };
    return create_msg_data(buff, sizeof(buff), OpType::Write);
}

std::string GripperMotionRequest::name() const
{
    return "Gripper Motion Request";
}

////////////////////////////////////////////////////////////////////////////////
// WriteRegistersGripperResponse
////////////////////////////////////////////////////////////////////////////////

uint16_t WriteRegistersGripperResponse::start_register() const
{
    return combine(response_msg()[8], response_msg()[9]);
}

uint16_t WriteRegistersGripperResponse::num_registers_written() const
{
    return combine(response_msg()[10], response_msg()[11]);
}

std::vector<uint8_t>
GripperStatusRequest::construct_payload() const
{
    uint8_t buff[] = { hi(ADDRESS_REGISTER0), lo(ADDRESS_REGISTER0),
                       0x00, 0x03 };
    return create_msg_data(buff, sizeof(buff), OpType::Read);
}

std::string GripperStatusRequest::name() const
{
    return "Gripper Status Request";
}

////////////////////////////////////////////////////////////////////////////////
// ReadRegistersGripperResponse
////////////////////////////////////////////////////////////////////////////////

ReadRegistersGripperResponse::~ReadRegistersGripperResponse()
{
}

uint8_t ReadRegistersGripperResponse::num_data_bytes() const
{
   return response_msg()[8];
}

const uint8_t* ReadRegistersGripperResponse::data() const
{
   return &response_msg().data()[9];
}

////////////////////////////////////////////////////////////////////////////////
// GripperStatusResponse
////////////////////////////////////////////////////////////////////////////////

GripperStatusResponse::~GripperStatusResponse()
{

}

bool GripperStatusResponse::act_on() const
{
    return data()[0] & (1 << 0);
}

bool GripperStatusResponse::gto_on() const
{
    return data()[0] & (1 << 3); 
}

Status GripperStatusResponse::status() const
{
    const uint8_t sbits = (((data()[0] >> 4) & 0x01) << 1) | ((data()[0] >> 5) & (0x01));
    if (sbits == 0x00) {
        return Status::Reset;
    }
    else if (sbits == 0x02) {
        return Status::ActivationInProgress;
    }
    else if (sbits == 0x03) {
        return Status::ActivationComplete;
    }
    else {
        return Status::Invalid;
    }
}

ObjectStatus GripperStatusResponse::object_status() const
{
    const uint8_t obits = (((data()[0] >> 7) & 0x01) << 1) | ((data()[0] >> 6) & (0x01));
    if (obits == 0x00) {
        return ObjectStatus::FingersInMotion;
    }
    else if (obits == 0x01) {
        return ObjectStatus::FingersStoppedDueToContactWhileClosing;
    }
    else if (obits == 0x02) {
        return ObjectStatus::FingersStoppedDueToContactWhileOpening;
    }
    else {
        return ObjectStatus::FingersAtRequestedPosition;
    }
}

FaultStatus GripperStatusResponse::fault_status() const
{
    const uint8_t fbits = data()[2] & 0x0F; // assert reserved spots are 0's
    if (fbits == 0x00) {
        return FaultStatus::NoFault;
    }
    else if (fbits == 0x05) {
        return FaultStatus::ActionDelayed;
    }
    else if (fbits == 0x07) {
        return FaultStatus::ActivationBitNotSet;
    }
    else if (fbits == 0x09) {
        return FaultStatus::CommunicationNotReady;
    }
    else if (fbits == 0x0B) {
        return FaultStatus::AutomaticReleaseInProgress;
    }
    else if (fbits == 0x0E) {
        return FaultStatus::OvercurrentProtectionTriggered;
    }
    else if (fbits == 0x0F) {
        return FaultStatus::AutomaticReleaseCompleted;
    }
    else {
        return FaultStatus::Invalid;
    }
}

uint8_t GripperStatusResponse::pos_echo() const
{
    return data()[3];
}

uint8_t GripperStatusResponse::pos() const
{
    return data()[4];
}

uint8_t GripperStatusResponse::current() const
{
    return data()[5];
}

