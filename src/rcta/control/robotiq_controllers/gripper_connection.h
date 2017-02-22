#ifndef GripperConnection_h
#define GripperConnection_h

// standard includes
#include <cstdint>
#include <string>
#include <vector>

// system includes
#include <boost/asio.hpp>

// module includes
#include "gripper_msgs.h"

class GripperConnection
{
public:

    struct ConnectionOptions
    {
        static const uint32_t DefaultIPAddress = 0xC0A8010B;
        static const uint16_t DefaultPort = 502;
        static const uint16_t DefaultUnitID = 0x0002;
        static const uint16_t DefaultGripperInputRegisterOffset = 0x0000;
        static const uint16_t DefaultGripperOutputRegisterOffset = 0x0000;

        uint32_t ip_address;
        uint16_t portno;
        uint16_t unit_id;
        uint16_t gripper_input_register_offset;
        uint16_t gripper_output_register_offset;

        // Default construct ConnectionOptions with the default values listed above.
        ConnectionOptions() :
            ip_address(DefaultIPAddress),
            portno(DefaultPort),
            unit_id(DefaultUnitID),
            gripper_input_register_offset(DefaultGripperInputRegisterOffset),
            gripper_output_register_offset(DefaultGripperOutputRegisterOffset)
        {}
    };

    GripperConnection(const ConnectionOptions& = ConnectionOptions());
    ~GripperConnection();

    bool connect();
    bool connect(std::string& why);

    bool connected() const;

    bool send_request(const GripperRequest& request, GripperResponse& response);

private:

    ConnectionOptions ops_;
    boost::asio::io_service io_service_;
    boost::asio::ip::tcp::resolver resolver_;
    boost::asio::ip::tcp::socket socket_;

    static const std::size_t MODBUS_HEADER_LENGTH = 6;
    uint8_t modbus_header_[MODBUS_HEADER_LENGTH];

    std::vector<uint8_t> response_payload_;

    std::string to_string(const std::vector<uint8_t>& data);
};

#endif
