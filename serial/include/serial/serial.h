#ifndef SERIAL_H
#define SERIAL_H

#include <string>
#include <vector>
#include <stdexcept>
#include <stdint.h>

namespace serial {

enum bytesize_t { fivebits = 5, sixbits = 6, sevenbits = 7, eightbits = 8 };
enum parity_t { parity_none = 0, parity_odd = 1, parity_even = 2, parity_mark = 3, parity_space = 4 };
enum stopbits_t { stopbits_one = 1, stopbits_two = 2, stopbits_one_point_five };
enum flowcontrol_t { flowcontrol_none = 0, flowcontrol_software, flowcontrol_hardware };

struct Timeout {
    static uint32_t max() { return 0xffffffff; }
    static Timeout simpleTimeout(uint32_t timeout) { return Timeout(timeout, timeout, 0, timeout, 0); }
    uint32_t inter_byte_timeout;
    uint32_t read_timeout_constant;
    uint32_t read_timeout_multiplier;
    uint32_t write_timeout_constant;
    uint32_t write_timeout_multiplier;
    Timeout(uint32_t ibt=0, uint32_t rtc=0, uint32_t rtm=0, uint32_t wtc=0, uint32_t wtm=0)
    : inter_byte_timeout(ibt), read_timeout_constant(rtc), read_timeout_multiplier(rtm),
      write_timeout_constant(wtc), write_timeout_multiplier(wtm) {}
};

class Serial {
public:
    Serial(const std::string &port = "", uint32_t baudrate = 9600, Timeout timeout = Timeout())
    : port_(port), baudrate_(baudrate), timeout_(timeout), is_open_(false) {}
    virtual ~Serial() { close(); }

    void open() { is_open_ = true; }
    bool isOpen() const { return is_open_; }
    void close() { is_open_ = false; }
    void setPort(const std::string &port) { port_ = port; }
    std::string getPort() const { return port_; }
    void setTimeout(Timeout &timeout) { timeout_ = timeout; }
    void setBaudrate(uint32_t baudrate) { baudrate_ = baudrate; }
    uint32_t getBaudrate() const { return baudrate_; }

    size_t write(const std::string &data) { return data.length(); }
    std::string readline(size_t size = 65536, std::string eol = "
") {
        return "0 0
";
    }

private:
    std::string port_;
    uint32_t baudrate_;
    Timeout timeout_;
    bool is_open_;
};

} // namespace serial

#endif
