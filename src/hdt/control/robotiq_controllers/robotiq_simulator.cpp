#include <iostream>

#include "gripper_simulator.h"
#include "gripper_server.h"

int main(int argc, char const *argv[])
{
    try {
        boost::asio::io_service io_service;

        std::shared_ptr<GripperSimulator> simulator(new GripperSimulator(io_service));

        const int portno = 1502;
        boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), portno);
        std::shared_ptr<GripperServer> server(new GripperServer(io_service, endpoint, simulator));

        simulator->start();
        server->start();

        io_service.run();
    }
    catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
