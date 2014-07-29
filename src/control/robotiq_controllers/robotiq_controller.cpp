#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <errno.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>

#include "gripper_interface.h"
#include "gripper_connection.h"
#include "gripper_msgs.h"

void print_prompt()
{
    printf("(A)ctivate, (R)eset, Au(T)omatic Release, (S)top, (G)oto [0-255], Spee(D) [0-255], (F)orce [0-255]\n");
    printf(">>> ");
}

void print_status(const GripperInterface& gripper)
{
    printf("Activating:             %s\n", gripper.is_activating() ? "TRUE" : "FALSE");
    printf("Activated:              %s\n", gripper.is_activated() ? "TRUE" : "FALSE");
    printf("Reset:                  %s\n", gripper.is_reset() ? "TRUE" : "FALSE");
    printf("Standby:                %s\n", gripper.is_standby() ? "TRUE" : "FALSE");
    printf("Contact Closing:        %s\n", gripper.made_contact_closing() ? "TRUE" : "FALSE");
    printf("Contact Opening:        %s\n", gripper.made_contact_opening() ? "TRUE" : "FALSE");
    printf("Completed Positioning:  %s\n", gripper.completed_positioning() ? "TRUE" : "FALSE");

    printf("Position:               %0.3f\n", gripper.get_position());
    printf("Current:                %0.3f\n", gripper.get_current());
    printf("Position Echo:          %0.3f\n", gripper.get_requested_position());

    printf("Status:                 %s\n", to_string(gripper.get_status()).c_str());
    printf("Object Status:          %s\n", to_string(gripper.get_object_status()).c_str());
    printf("Fault Status:           %s\n", to_string(gripper.get_fault_status()).c_str());
}

void sleep_for_gripper_update()
{
    const unsigned int gripper_update_us = 5000;
    usleep(gripper_update_us);
}

int main(int argc, char *argv[])
{
    std::shared_ptr<GripperConnection> default_conn;

    if (argc > 1) {
        std::string mode(argv[1]);
        if (mode == "live") {
            default_conn.reset(new GripperConnection);
        }
        else if (mode == "sim") {
            GripperConnection::ConnectionOptions ops;
            ops.ip_address = 0x7F000001; // localhost
            ops.portno = 1502;
            default_conn.reset(new GripperConnection(ops));
        }
        else {
            fprintf(stderr, "Second argument must be either 'live' or 'sim'\n");
            exit(1);
        }
    }
    else {
        printf("Usage: %s (live|sim)\n", argv[0]);
        exit(2);
    }

    std::string why;
    if (!default_conn->connect(why)) {
        fprintf(stderr, "Failed to connect to Robotiq Gripper (%s)\n", why.c_str());
        return 1;
    }

    if (!default_conn->connected()) {
        fprintf(stderr, "Connection succeeded but gripper is not connected. Wtf?!?!\n");
        return 2;   
    }

    GripperInterface gripper(default_conn);

    while (true) {
        print_prompt();
        std::string s;
        std::getline(std::cin, s);
        if (!s.empty()) {
            switch (s[0])
            {
            case 'A':
                gripper.activate();
                break;
            case 'R':
                gripper.reset();
                break;
            case 'T':
                gripper.release();
                break;
            case 'S':
                gripper.stop();
                break;
            case 'G':
                if (s.size() != 6) {
                    printf("G 0x[0-F][0-F] required\n");
                }
                else {
                    std::string digits = s.substr(2);
                    uint8_t val = (uint8_t)std::stoi(digits, nullptr, 0);
                    gripper.set_position(val);
                }
                break;
            case 'D':
                if (s.size() != 6) {
                    printf("F 0x[0-F][0-F] required\n");
                }
                else {
                    std::string digits = s.substr(2);
                    uint8_t val = (uint8_t)std::stoi(digits, nullptr, 0);
                    gripper.set_speed(val);
                }
                break;
            case 'F':
                if (s.size() != 6) {
                    printf("F 0x[0-F][0-F] required\n");
                }
                else {
                    std::string digits = s.substr(2);
                    uint8_t val = (uint8_t)std::stoi(digits, nullptr, 0);
                    gripper.set_force(val);
                }
                break;
            }
        }

        sleep_for_gripper_update();

        if (!gripper.update()) {
            fprintf(stderr, "Failed to update gripper\n");
            exit(3);
        }

        print_status(gripper);
    }

    return 0;
}
