#include <cstdio>
#include <lcm/lcm-cpp.hpp>
#include "../lcm_types/cpp/usb_command_t.hpp"
#include "../lcm_types/cpp/usb_data_t.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>


int main(int argc, char **argv) {
    // the offset on real robot
    // constexpr float abad_side_sign[4] = {1.f, 1.f, -1.f, -1.f};
    // constexpr float hip_side_sign[4] = {-1.f, 1.f, -1.f, 1.f};
    // constexpr float knee_side_sign[4] = {-10.0 / 14.0f, 10.0 / 14.0f, -10.0 / 14.0f, 10.0 / 14.0f};

    // log file cmd
    std::fstream read_file_data_;
    std::fstream read_file_cmd_;

    read_file_data_.open("motor_data.txt", std::ios::out | std::ios::trunc);
    read_file_cmd_.open("motor_cmd.txt", std::ios::out | std::ios::trunc);

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " read_log";
        return 1;
    }

    lcm::LogFile log(argv[1], "r");
    if (!log.good()) {
        std::cerr << "Unable to open log file: " << argv[1];
        return 1;
    }

    while (true) {
        const lcm::LogEvent *event = log.readNextEvent();
        if (!event) {
            break;
        }
        if (event->channel == "MOTOR DATA") {
            usb_data_t msg{};
            if (msg.decode(event->data, 0, event->datalen) != event->datalen)
                continue;
            // decode successful

            read_file_data_ << std::setw(12) << std::setfill(' ') << msg.chip1_data[5] << " " <<
                    std::setw(12) << std::setfill(' ') << msg.chip1_data[6] << " " <<
                    std::setw(12) << std::setfill(' ') << msg.chip1_data[7] << " " <<
                    std::setw(12) << std::setfill(' ') << msg.chip1_data[8] << " " <<
                    std::setw(12) << std::setfill(' ') << msg.chip1_data[9] << " " << " \n";
        } else if (event->channel == "MOTOR COMMAND") {
            usb_command_t msg_cmd{};
            if (msg_cmd.decode(event->data, 0, event->datalen) != event->datalen)
                continue;
            // decode successful

            read_file_cmd_ << std::setw(12) << std::setfill(' ') << msg_cmd.chip1_cmd[6] << " \n";
        }
        // decode successful
    }
    read_file_data_.close();
    read_file_cmd_.close();
    return 0;
}
