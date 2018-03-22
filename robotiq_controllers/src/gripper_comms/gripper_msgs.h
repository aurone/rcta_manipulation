#ifndef gripper_msgs_h
#define gripper_msgs_h

// standard includes
#include <cstdint>
#include <string>
#include <vector>

// module includes
#include "gripper_status.h"

inline uint8_t hi(uint16_t val)
{
    return (uint8_t)((0xFF00 & val) >> 8);
}

inline uint8_t lo(uint16_t val)
{
    return (uint8_t)(0x00FF & val);
}

inline uint16_t combine(uint8_t hi, uint8_t lo)
{
    return (((uint16_t)hi) << 8) | lo;
}

////////////////////////////////////////////////////////////////////////////////
// Generic request and response messages
////////////////////////////////////////////////////////////////////////////////

class GripperRequest
{
public:

    static const uint16_t PROTOCOL_ID;
    static const uint8_t SLAVE_ID;
    static const uint8_t FUNCTION_CODE_READ_INPUT_REGISTERS;
    static const uint8_t FUNCTION_CODE_PRESET_MULTIPLE_REGISTERS;
    static const uint16_t ADDRESS_REGISTER0;
    static const uint16_t ADDRESS_REGISTER1;
    static const uint16_t ADDRESS_REGISTER2;

    GripperRequest(uint16_t trans_id);
    virtual ~GripperRequest();

    uint16_t trans_id() const { return trans_id_; }

    virtual std::vector<uint8_t> construct_payload() const = 0;
    virtual std::string name() const = 0;

    enum class OpType
    {
        Read,
        Write
    };

    /// @brief Create message data given message payload and operation.
    std::vector<uint8_t> create_msg_data(uint8_t* payload, uint16_t payload_length, OpType op) const;

private:

    uint16_t trans_id_;
};

class GripperResponse
{
public:

    virtual ~GripperResponse();

    void set_response_msg(std::vector<uint8_t>&& msg) { response_msg_ = msg; }
    const std::vector<uint8_t> response_msg() const { return response_msg_; }

    uint16_t trans_id() const;
    uint16_t protocol_id() const;
    uint16_t length() const;
    uint8_t slave_id() const;
    uint8_t op_code() const;

private:

    std::vector<uint8_t> response_msg_;
};

////////////////////////////////////////////////////////////////////////////////
// Requests and responses for gripper functionalities
////////////////////////////////////////////////////////////////////////////////

class ActivateGripperRequest : public GripperRequest
{
public:

    ActivateGripperRequest(uint16_t trans_id);
    std::vector<uint8_t> construct_payload() const;
    std::string name() const;
};

class ResetGripperRequest : public GripperRequest
{
public:

    ResetGripperRequest(uint16_t trans_id);
    std::vector<uint8_t> construct_payload() const;
    std::string name() const;
};

class AutomaticReleaseGripperRequest : public GripperRequest
{
public:

    AutomaticReleaseGripperRequest(uint16_t trans_id);
    std::vector<uint8_t> construct_payload() const;
    std::string name() const;
};

class GripperPositionRequest : public GripperRequest
{
public:

    GripperPositionRequest(uint16_t trans_id, uint8_t pos);
    std::vector<uint8_t> construct_payload() const;
    std::string name() const;

private:

    uint8_t pos_;
};

class GripperDynamicsRequest : public GripperRequest
{
public:

    GripperDynamicsRequest(uint16_t trans_id, uint8_t vel, uint8_t force);
    std::vector<uint8_t> construct_payload() const;
    std::string name() const;

private:

    uint8_t vel_;
    uint8_t force_;
};

class GripperMotionRequest : public GripperRequest
{
public:

    GripperMotionRequest(uint16_t trans_id, uint8_t pos, uint8_t vel, uint8_t force);
    std::vector<uint8_t> construct_payload() const;
    std::string name() const;

private:

    uint8_t pos_;
    uint8_t vel_;
    uint8_t force_;
};

class StopGripperRequest : public GripperRequest
{
public:

    StopGripperRequest(uint16_t trans_id);
    std::vector<uint8_t> construct_payload() const;
    std::string name() const;
};

class WriteRegistersGripperResponse : public GripperResponse
{
public:

    uint16_t start_register() const;
    uint16_t num_registers_written() const;
};

////////////////////////////////////////////////////////////////////////////////
// Requests and responses for gripper status
////////////////////////////////////////////////////////////////////////////////

class GripperStatusRequest : public GripperRequest
{
public:

    GripperStatusRequest(uint16_t trans_id);
    std::vector<uint8_t> construct_payload() const;
    std::string name() const;
};

class ReadRegistersGripperResponse : public GripperResponse
{
public:

    virtual ~ReadRegistersGripperResponse();

    uint8_t num_data_bytes() const;
    const uint8_t* data() const;
};

class GripperStatusResponse : public ReadRegistersGripperResponse
{
public:

    virtual ~GripperStatusResponse();

    bool act_on() const;
    bool gto_on() const;

    Status status() const;
    ObjectStatus object_status() const;
    FaultStatus fault_status() const;

    uint8_t pos_echo() const;
    uint8_t pos() const;
    uint8_t current() const;
};

#endif
