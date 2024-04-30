#pragma once

#include <string>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <mutex>

#include "dvl_a50/json/single_include/nlohmann/json.hpp"
#include "dvl_a50/tcpsocket.hpp"


namespace dvl_a50
{


class DvlA50
{
public:
    typedef nlohmann::json Message;

    DvlA50()
    {}

    ~DvlA50()
    {
        disconnect();
    }

    int connect(std::string addr, bool enable = true);
    void disconnect();
    void send(const Message& msg);
    Message receive();
    Message wait_for_response(std::function<bool(const Message&)> check, uint32_t timeout_ms);

    void configure(
        int speed_of_sound,
        bool acoustic_enabled,
        bool led_enabled,
        int mountig_rotation_offset,
        std::string range_mode);

    void set_speed_of_sound(int speed_of_sound);
    void set_acoustic_enabled(bool enabled);
    void set_led_enabled(bool enabled);
    void set_mounting_rotation_offset(int offset_degrees);
    void set_range_mode(std::string range_mode);

    template<typename T> 
    void set(std::string param, const T& value)
    {
        std::cout << param << ": " << value << std::endl;

        Message message;
        message["command"] = "set_config";
        message["parameters"][param] = value;
        send(message);
    }
    
    void send_command(std::string cmd);
    void get_config();
    void calibrate_gyro();
    void reset_dead_reckoning();
    void trigger_ping();


private:
    int fault = 1; 
    bool enabled = false;
    TCPSocket *tcp_socket;
    std::mutex mtx;
};


};