// standard includes
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <sstream>

// systemm includes
#include <netinet/in.h>
#include <sys/socket.h>

// module includes
#include "gripper_connection.h"

GripperConnection::GripperConnection(const ConnectionOptions& options) :
    ops_(options),
    io_service_(),
    resolver_(io_service_),
    socket_(io_service_)
{

}

GripperConnection::~GripperConnection()
{

}

bool GripperConnection::connect()
{
    std::string empty;
    return connect(empty);
}

bool GripperConnection::connect(std::string& why)
{
    try {
        boost::system::error_code ec;

        boost::asio::ip::address address(boost::asio::ip::address_v4((unsigned long)ops_.ip_address));
        boost::asio::ip::tcp::endpoint endpoint(address, ops_.portno);

        socket_.connect(endpoint, ec);

        if (ec) {
            why = "Failed to connect";
            return false;
        }

        return true;
    }
    catch (std::exception& e) {
        why = std::string("std::exception encountered trying to connect to gripper (") + std::string(e.what()) + std::string(")");
        return false;
    }
}

bool GripperConnection::connected() const
{
    return true;
}

std::string GripperConnection::to_string(const std::vector<uint8_t>& data)
{
    std::stringstream ss;

    ss.fill('0');
    ss << std::setw(2) << std::hex;
    ss << "[ ";
    for (size_t i = 0; i < data.size(); ++i) {
        ss << std::hex << std::setw(2) << (unsigned int)data[i] << ' ';
    }
    ss << ']';

    return ss.str();
}

bool GripperConnection::send_request(const GripperRequest& req, GripperResponse& res)
{
    std::vector<uint8_t> payload = req.construct_payload();

//    printf("%s is %s\n", req.name().c_str(), to_string(payload).c_str());

    // send request and block waiting for the response
    try {
        std::size_t num_bytes_written = boost::asio::write(socket_, boost::asio::buffer(payload));
        if (num_bytes_written != payload.size()) {
            fprintf(stderr, "Only wrote %zd of %zd bytes\n", num_bytes_written, payload.size());
            return false;
        }
    }
    catch (boost::system::system_error& e) {
        fprintf(stderr, "Failed to write request message to socket (%s)\n", e.what());
        return false;
    }

    // read in header
    try {
        std::size_t num_bytes_read = boost::asio::read(socket_, boost::asio::buffer(modbus_header_));
        if (num_bytes_read != MODBUS_HEADER_LENGTH) {
            fprintf(stderr, "Failed to read %zd of %zd header bytes\n", MODBUS_HEADER_LENGTH - num_bytes_read, MODBUS_HEADER_LENGTH);
            return false;
        }
    }
    catch (boost::system::system_error& e) {
        fprintf(stderr, "Failed to read response header from socket (%s)\n", e.what());
        return false;
    }

    // parse header
    uint16_t trans_id = combine(modbus_header_[0], modbus_header_[1]);
    uint16_t protocol_id = combine(modbus_header_[2], modbus_header_[3]);
    uint16_t length = combine(modbus_header_[4], modbus_header_[5]);

    // read payload
    response_payload_.resize(length);
    try {
        std::size_t num_bytes_read = boost::asio::read(socket_, boost::asio::buffer(response_payload_));
        if (num_bytes_read != response_payload_.size()) {
            fprintf(stderr, "Failed to read %zd of %zd payload bytes\n", response_payload_.size() - num_bytes_read, response_payload_.size());
            return false;
        }
    }
    catch (boost::system::system_error& e) {
        fprintf(stderr, "Failed to read response payload from socket (%s)\n", e.what());
        return false;
    }

    // copy buffer data into response
    std::vector<uint8_t> response;
    response.resize(sizeof(modbus_header_) + length);
    memcpy((void*)response.data(), (const void*)modbus_header_, sizeof(modbus_header_));
    memcpy((void*)(response.data() + sizeof(modbus_header_)), (const void*)response_payload_.data(), length);

//    printf("Response message is %s\n", to_string(response).c_str());

    res.set_response_msg(std::move(response));
    return true;
}
