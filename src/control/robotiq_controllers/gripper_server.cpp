#include <cstdio>
#include <iomanip>
#include <boost/bind.hpp>
#include "gripper_server.h"
#include "gripper_simulator.h"
#include "gripper_msgs.h"

#ifdef DEBUG
#define LOG_DEBUG(fmt, ...) printf(fmt "\n", ##__VA_ARGS__)
#else
#define LOG_DEBUG(fmt, ...)
#endif

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

GripperServer::GripperServer(
    boost::asio::io_service& io_service,
    const boost::asio::ip::tcp::endpoint& endpoint,
    const std::shared_ptr<GripperSimulator>& simulator)
:
    acceptor_(io_service, endpoint),
    simulator_(simulator),
    started_(false)
{
}

void GripperServer::start_accept()
{
    Connection::pointer new_conn = Connection::create(acceptor_.io_service(), simulator_);
    acceptor_.async_accept(new_conn->socket(), boost::bind(&GripperServer::handle_accept, this, new_conn, boost::asio::placeholders::error));
}

void GripperServer::handle_accept(Connection::pointer new_conn, const boost::system::error_code& error)
{
    if (!error) {
        printf("Accepted connection!\n");
        new_conn->start();
        start_accept(); // start listening for another connection
    }
    else {
        printf("Connection refused\n");
    }
}

void GripperServer::start()
{
    if (!started_) {
        printf("Starting server\n");
        start_accept();
        started_ = true;
    }
}

GripperServer::Connection::~Connection()
{
    printf("Connection closed\n");
}

GripperServer::Connection::pointer GripperServer::Connection::create(
    boost::asio::io_service& io_service,
    const std::shared_ptr<GripperSimulator>& simulator)
{
    return pointer(new Connection(io_service, simulator));
}

void GripperServer::Connection::start()
{
    buff_.resize(HEADER_LENGTH_);
    start_read_header();
}

GripperServer::Connection::Connection(
    boost::asio::io_service& io_service,
    const std::shared_ptr<GripperSimulator>& simulator)
:
    socket_(io_service),
    buff_(),
    simulator_(simulator),
    header_bytes_read_(0),
    payload_bytes_read_(0)
{
}

void GripperServer::Connection::handle_read_header(const boost::system::error_code& error, size_t bytes_transferred)
{
    if (header_bytes_read_ != HEADER_LENGTH_) {
        fprintf(stderr, "Failed to read message header\n");
        socket_.close();
        return;
    }

    LOG_DEBUG("Read a %zd byte header: %s", bytes_transferred, to_string(buff_.cbegin(), buff_.cend()).c_str());

    last_trans_id_ = combine(buff_[0], buff_[1]);
    uint16_t protocol_id = combine(buff_[2], buff_[3]);
    if (protocol_id != GripperRequest::PROTOCOL_ID) {
        // close the connection
        socket_.close();
    }
    payload_size_ = combine(buff_[4], buff_[5]);
    buff_.resize(payload_size_);

    start_read_payload();
}

void GripperServer::Connection::handle_read_payload(const boost::system::error_code& error, size_t bytes_transferred)
{
    if (payload_bytes_read_ != payload_size_) {
        fprintf(stderr, "Failed to read message payload\n");
        return;
    }

    LOG_DEBUG("Read a %zd byte payload: %s", bytes_transferred, to_string(buff_.cbegin(), buff_.cend()).c_str());

    const uint8_t UNIT_ID_OFFSET = 0x00;
    const uint8_t FUNCTION_CODE_OFFSET = UNIT_ID_OFFSET + 1;
    const uint8_t FIRST_REGISTER_ADDRESS_OFFSET = FUNCTION_CODE_OFFSET + 1;
    const uint8_t WORD_COUNT_OFFSET = FIRST_REGISTER_ADDRESS_OFFSET + 2;
    const uint8_t WRITE_LENGTH_OFFSET = WORD_COUNT_OFFSET + 2;
    const uint8_t WRITE_DATA_OFFSET = WRITE_LENGTH_OFFSET + 1;

    if (buff_[FUNCTION_CODE_OFFSET] == GripperRequest::FUNCTION_CODE_PRESET_MULTIPLE_REGISTERS) {
        uint16_t first_register = combine(buff_[FIRST_REGISTER_ADDRESS_OFFSET], buff_[FIRST_REGISTER_ADDRESS_OFFSET + 1]);
        uint16_t word_count = combine(buff_[WORD_COUNT_OFFSET], buff_[WORD_COUNT_OFFSET + 1]);
        std::vector<uint8_t>::size_type num_bytes = (std::vector<uint8_t>::size_type)buff_[WRITE_LENGTH_OFFSET];
        std::vector<uint8_t> data(num_bytes);
        for (int i = 0 ; i < num_bytes; ++i) {
            data[i] = buff_[WRITE_DATA_OFFSET + i];
        }

        simulator_->write_registers((int)first_register, data);

        const uint16_t write_response_length = 0x0006;

        response_buff_.clear();
        response_buff_.reserve(2 + 2 + 2 + 2 + 2 + 2);
        response_buff_.push_back(hi(last_trans_id_));
        response_buff_.push_back(lo(last_trans_id_));
        response_buff_.push_back(hi(GripperRequest::PROTOCOL_ID));
        response_buff_.push_back(lo(GripperRequest::PROTOCOL_ID));
        response_buff_.push_back(hi(write_response_length));
        response_buff_.push_back(lo(write_response_length));
        response_buff_.push_back(GripperRequest::SLAVE_ID);
        response_buff_.push_back(GripperRequest::FUNCTION_CODE_PRESET_MULTIPLE_REGISTERS);
        response_buff_.push_back(hi(first_register));
        response_buff_.push_back(lo(first_register));
        response_buff_.push_back(hi(word_count));
        response_buff_.push_back(lo(word_count));

        start_respond();
    }
    else if (buff_[FUNCTION_CODE_OFFSET] == GripperRequest::FUNCTION_CODE_READ_INPUT_REGISTERS) {
        uint16_t first_register = combine(buff_[FIRST_REGISTER_ADDRESS_OFFSET], buff_[FIRST_REGISTER_ADDRESS_OFFSET + 1]);
        uint16_t word_count = combine(buff_[WORD_COUNT_OFFSET], buff_[WORD_COUNT_OFFSET + 1]);
        std::vector<uint8_t> registers = simulator_->read_registers((int)first_register, (int)word_count);

        response_buff_.clear();
        response_buff_.reserve(2 + 2 + 2 + 2 + 1 + registers.size());
        response_buff_.push_back(hi(last_trans_id_));
        response_buff_.push_back(lo(last_trans_id_));
        response_buff_.push_back(hi(GripperRequest::PROTOCOL_ID));
        response_buff_.push_back(lo(GripperRequest::PROTOCOL_ID));
        response_buff_.push_back(hi(registers.size() + 3));
        response_buff_.push_back(lo(registers.size() + 3)); // slave id + function code + num registers + register_data
        response_buff_.push_back(GripperRequest::SLAVE_ID);
        response_buff_.push_back(GripperRequest::FUNCTION_CODE_READ_INPUT_REGISTERS);
        response_buff_.push_back(registers.size());
        response_buff_.insert(response_buff_.end(), registers.begin(), registers.end());

        start_respond();
    }
    else {
        fprintf(stderr, "Unrecognized function code (%x)\n", (unsigned)buff_[7]);
    }
}

void GripperServer::Connection::handle_write_response(const boost::system::error_code& error, size_t bytes_transferred)
{
    if (error) {
        fprintf(stderr, "Failed to write message response\n");
        return;
    }

    LOG_DEBUG("Wrote a %zd byte response: %s", bytes_transferred, to_string(response_buff_.cbegin(), response_buff_.cend()).c_str());

    // reset for the next message header
    header_bytes_read_ = 0;
    payload_bytes_read_ = 0;
    payload_size_ = 0;
    buff_.resize(HEADER_LENGTH_);

    start_read_header();
}

std::size_t GripperServer::Connection::read_header_condition(
    const boost::system::error_code& error,
    std::size_t bytes_transferred)
{
    if (error) {
        return 0;
    }

    int bytes_remaining = HEADER_LENGTH_ - header_bytes_read_;

    if (bytes_transferred > bytes_remaining) {
        fprintf(stderr, "Read more than necessary for the header (%d remaining, %zd read)\n", bytes_remaining, bytes_transferred);
    }

    if (bytes_transferred + header_bytes_read_ >= HEADER_LENGTH_) {
        header_bytes_read_ = HEADER_LENGTH_;
//        printf("Finished reading header\n");
        return 0;
    }
    else {
        header_bytes_read_ -= bytes_transferred;
        return HEADER_LENGTH_ - header_bytes_read_;
    }
}

std::size_t GripperServer::Connection::read_payload_condition(
    const boost::system::error_code& error,
    std::size_t bytes_transferred)
{
    int bytes_remaining = payload_size_ - payload_bytes_read_;

    if (bytes_transferred > bytes_remaining) {
        printf("Read more than necessary for the payload (%d remaining, %zd read)\n", bytes_remaining, bytes_transferred);
    }

    if (bytes_transferred + payload_bytes_read_ >= payload_size_) {
        payload_bytes_read_ = payload_size_;
//        printf("Finished reading payload\n");
        return 0;
    }
    else {
        payload_bytes_read_ -= bytes_transferred;
        return payload_size_ - payload_bytes_read_;
    }
}

void GripperServer::Connection::start_read_header()
{
    boost::asio::async_read(socket_, boost::asio::buffer(buff_),
                            boost::bind(&Connection::read_header_condition,
                                        shared_from_this(),
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred),
                            boost::bind(&Connection::handle_read_header,
                                        shared_from_this(),
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred));
}

void GripperServer::Connection::start_read_payload()
{
    boost::asio::async_read(socket_, boost::asio::buffer(buff_),
                            boost::bind(&Connection::read_payload_condition,
                                        shared_from_this(),
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred),
                            boost::bind(&Connection::handle_read_payload,
                                        shared_from_this(),
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred));
}

void GripperServer::Connection::start_respond()
{
    boost::asio::async_write(socket_, boost::asio::buffer(response_buff_),
                             boost::bind(&Connection::handle_write_response,
                                         shared_from_this(),
                                         boost::asio::placeholders::error,
                                         boost::asio::placeholders::bytes_transferred));
}
