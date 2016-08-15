#ifndef GripperServer_h
#define GripperServer_h

// standard includes
#include <memory>
#include <vector>

// system includes
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

class GripperSimulator;

class GripperServer
{
public:

    GripperServer(boost::asio::io_service& io_service,
                  const boost::asio::ip::tcp::endpoint& endpoint,
                  const std::shared_ptr<GripperSimulator>& simulator);

    void start();

private:

    class Connection : public boost::enable_shared_from_this<Connection>
    {
    public:

        ~Connection();

        typedef boost::shared_ptr<Connection> pointer;
        static pointer create(boost::asio::io_service& io_service, const std::shared_ptr<GripperSimulator>& simulator);

        boost::asio::ip::tcp::socket& socket() { return socket_; }

        void start();

    private:

        static const uint16_t MODBUS_TCP_IP_PROTOCOL_ID_ = 0x0000;
        static const std::size_t HEADER_LENGTH_ = 6;

        Connection(boost::asio::io_service& io_service, const std::shared_ptr<GripperSimulator>& simulator);

        void handle_read_header(const boost::system::error_code&, size_t bytes_transferred);
        void handle_read_payload(const boost::system::error_code&, size_t bytes_transferred);
        void handle_write_response(const boost::system::error_code& error, size_t bytes_transferred);

        boost::asio::ip::tcp::socket socket_;
        std::vector<uint8_t> buff_;
        std::vector<uint8_t> response_buff_;
        std::shared_ptr<GripperSimulator> simulator_;

        std::size_t header_bytes_read_;
        std::size_t payload_bytes_read_;
        uint16_t payload_size_;

        uint16_t last_trans_id_;

        std::size_t read_header_condition(const boost::system::error_code& error, std::size_t bytes_transferred);
        std::size_t read_payload_condition(const boost::system::error_code& error, std::size_t bytes_transferred);

        void start_read_header();
        void start_read_payload();
        void start_respond();
    };

    boost::asio::ip::tcp::acceptor acceptor_;
    std::shared_ptr<GripperSimulator> simulator_;

    bool started_;

    void start_accept();
    void handle_accept(Connection::pointer, const boost::system::error_code& error);
};

#endif
