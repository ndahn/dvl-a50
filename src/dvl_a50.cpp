#include "dvl_a50/dvl_a50.hpp"


using namespace dvl_a50;


int DvlA50::connect(std::string addr, bool enable)
{
    // Open TCP/IP connection to DVL
    tcp_socket = new tcp_socket((char*)addr.c_str(), 16171);
    if(tcp_socket->Create() < 0) 
    {
        // Error creating the socket
        return -1;
    }

    tcp_socket->SetRcvTimeout(400);
    std::string error;
    
    int error_code = 0;
    //int fault = 1; 
    
    first_time = std::chrono::steady_clock::now();
    first_time_error = first_time;
    while(fault != 0)
    {
        fault = tcp_socket->Connect(5000, error, error_code);
        if(error_code == 114)
        {
            // Is the sensor on?
            usleep(2000000);
            std::chrono::steady_clock::time_point current_time_error = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(current_time_error - first_time_error).count();
            if(dt >= 78.5) //Max time to set up
            {
                fault = -10;
                break;
            }
        }
        else if(error_code == 103)
        {
            // No route to host, DVL might be booting?
            usleep(2000000);
        }
    }  
    
    if(fault == -10)
    {
        tcp_socket->Close();
        // Turn the sensor on and try again!
        return -10;
    }
    
    this->enabled = enable;
    if (!enable)
    {
        // Disable transducer operation to limit sensor heating out of water.
        this->send_config_param("acoustic_enabled", false);
    }
    usleep(2000);

    return 0;
}


void DvlA50::disconnect()
{
    if (tcp_socket != nullptr)
    {
        tcp_socket->Close();
        delete tcp_socket;
    }
}


Response DvlA50::set_enabled(bool enabled)
{
    Response res = send_config_param("acoustic_enabled", enabled);
    if (res.success)
    {
        this->enbaled = enabled;
    }
    return res;
}


Response DvlA50::receive()
{
    // Single threaded access only
    std::lock_guard<std::mutex> lock(mtx);

    char *tempBuffer = new char[1];
    std::string str; 
    
    if(fault != 0)
    {
        return Response(ResponseType::Invalid, {"fault", itos(fault)});
    }
    
    while(tempBuffer[0] != '\n')
    {
        if(tcp_socket->Receive(tempBuffer) != 0)
        {
            str = str + tempBuffer[0];
        }
    }
    
    try
    {
        json_data = json::parse(str);

        if (json_data.contains("altitude"))
        {
            return Response(ResponseType::VelocityReport, json_data);
        }
        else if (json_data.contains("pitch"))
        {
            return Response(ResponseType::DeadReckoningReport, json_data);
        }
        else if (json_data.contains("response_to"))
        {
            if(json_data["response_to"] == "set_config"
            || json_data["response_to"] == "calibrate_gyro"
            || json_data["response_to"] == "reset_dead_reckoning")
            {
                return Response(ResponseType::CommandResponse, json_data);
            }
            else if(json_data["response_to"] == "get_config")
            {
                return Response(ResponseType::ConfigStatus, json_data);
            }
        }
    }
    catch(std::exception& e)
    {
        return Response(ResponseType::Invalid, {"error", std::string(e.what())});
    }
}


Response DvlA50::ping()
{
    if (enabled)
    {
        set_enabled(false);
    }
    return send_command("trigger_ping");
}


Response DvlA50::send_command(std::string cmd)
{
    std::cout << "send command '" << cmd << "'" << std::endl;
    
    json json_data = {"command", cmd};
    send_message(json_data);
    return wait_for_response();
}


Response DvlA50::send_config(
    int speed_of_sound,
    bool acoustic_enabled,
    bool dark_mode_enabled,
    int mountig_rotation_offset,
    std::string range_mode)
{
    json message;
    message["command"] = "set_config";
    message["parameters"]["speed_of_sound"] = speed_of_sound;
    message["parameters"]["acoustic_enabled"] = acoustic_enabled;
    message["parameters"]["dark_mode_enabled"] = dark_mode_enabled;
    message["parameters"]["mountig_rotation_offset"] = mountig_rotation_offset;
    message["parameters"]["range_mode"] = range_mode;

    send_message(message);
    return wait_for_response();
}


void DvlA50::send_message(const json& msg)
{
    std::string str = msg.dump();
    char* c = &*str.begin();
    tcp_socket->Send(c);
}


Response DvlA50::wait_for_response(ResponseType type)
{
    Response res;
    do
    {
        res = receive();
    }
    while (res.type != type);
    return res;
}
