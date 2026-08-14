#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

#include <serial/serial.h>
#include <cstring>
#include <sstream>
#include <iostream>

class ArduinoComms
{
public:
    ArduinoComms() = default;

    void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    {
        serial_conn_.setPort(serial_device);
        serial_conn_.setBaudrate(baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(timeout_ms);
        serial_conn_.setTimeout(to);
        serial_conn_.open();
    }

    void disconnect()
    {
        if (serial_conn_.isOpen())
        {
            serial_conn_.close();
        }
    }

    bool connected() const
    {
        return serial_conn_.isOpen();
    }

    std::string send_msg(const std::string &msg_to_send, bool print_output = false)
    {
        serial_conn_.write(msg_to_send);
        std::string response = serial_conn_.readline();

        if (print_output)
        {
            std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
        }

        return response;
    }

    void read_encoder_values(int &val_1, int &val_2)
    {
        std::string response = send_msg("e");

        std::string delimiter = " ";
        size_t del_pos = response.find(delimiter);
        if (del_pos != std::string::npos)
        {
            std::string token_1 = response.substr(0, del_pos);
            std::string token_2 = response.substr(del_pos + delimiter.length());

            try {
                val_1 = std::atoi(token_1.c_str());
                val_2 = std::atoi(token_2.c_str());
            } catch (...) {
                // keep previous values on parse error
            }
        }
    }

    void set_motor_values(int val_1, int val_2)
    {
        std::stringstream ss;
        ss << "m " << val_1 << " " << val_2 << "";
        send_msg(ss.str());
    }

    void set_pid_values(int k_p, int k_d, int k_i, int k_o)
    {
        std::stringstream ss;
        ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "";
        send_msg(ss.str());
    }

private:
    serial::Serial serial_conn_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
