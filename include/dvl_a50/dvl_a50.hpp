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


enum ResponseType
{
    Invalid,
    VelocityReport,
    DeadReckoningReport,
    CommandResponse,
    ConfigStatus,
};


struct Response
{
    ResponseType type;
    json data;
};


class DvlA50
{
public:
    DvlA50()
    {}

    ~DvlA50()
    {
        disconnect();
    }

    int connect(std::string addr, bool enable = true);
    void disconnect();

    Response set_enabled(bool enabled);
    Response receive();

    Response ping();
    Response send_command(std::string cmd);
    Response send_config(
        int speed_of_sound,
        bool acoustic_enabled,
        bool dark_mode_enabled,
        int mountig_rotation_offset,
        std::string range_mode);

    template<type T> 
    Response send_config_param(std::string param, const T& value)
    {
        std::cout << param << ": " << value << std::endl;

        json message;
        message["command"] = "set_config";
        message["parameters"][param] = value;
        send_message(json);
        return wait_for_response();
    }
    
    void send_message(const json& msg);
    Response wait_for_response(ResponeType type = ResponseType::CommandResponse);

private:
    int fault = 1; 
    bool enabled = false;
    TCPSocket *tcp_socket;
    std::mutex mtx;
    json json_data;
};


};