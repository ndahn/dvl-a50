#include <string>
#include <chrono>
#include <cstdlib>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "std_msgs/msg/string.hpp"

#include "dvl_msgs/msg/velocity_report.hpp"
#include "dvl_msgs/msg/dvl_beam.hpp"
#include "dvl_msgs/msg/dead_reckoning_report.hpp"
#include "dvl_msgs/srv/send_command.hpp"

#include "dvl_a50/dvl_a50.hpp"


using namespace dvl_a50;


class DvlA50Node : public rclcpp_lifecycle::LifecycleNode
{
public:
    DvlA50Node(std::string name)
    : rclcpp_lifecycle::LifecycleNode(name)
    {
        this->declare_parameter<std::string>("ip_address", "192.168.194.95");
        this->declare_parameter<std::string>("frame", "dvl_a50_link");
        this->declare_parameter<double>("rate", 15.0);
        this->declare_parameter<int>("speed_of_sound", 1500);
        this->declare_parameter<bool>("enable_on_start", true);
        this->declare_parameter<bool>("enable_led", true);
        this->declare_parameter<int>("mountig_rotation_offset", 0);
        this->declare_parameter<std::string>("range_mode", "auto");
    }

    ~DvlA50Node()
    {}

    CallbackReturn on_configure(const rclcpp_lifecycle::State& state)
    {
        // Parameters
        ip_address = this->get_parameter("ip_address").as_string();
        frame = this->get_parameter("frame").as_string();
        rate = this->get_parameter("rate").as_double();
        RCLCPP_INFO(get_logger(), "Connecting to DVL A50 at %s", ip_address.c_str());

        int success = dvl.connect(ip_address, false);
        if (success != 0)
        {
            RCLCPP_ERROR("Connection failed with error code %i", success);
            return CallbackReturn::FAILURE;
        }

        // Configure
        int speed_of_sound = this->get_parameter("speed_of_sound").as_int();
        bool enabled = this->get_parameter("enable_on_start").as_bool();
        bool led_enabled = this->get_parameter("led_enabled").as_bool();
        int mountig_rotation_offset = this->get_parameter("mountig_rotation_offset").as_int();
        std::string range_mode = this->get_parameter("range_mode").as_string();
        
        Response res = dvl.send_config(speed_of_sound, enabled, led_enabled, mountig_rotation_offset, range_mode);
        if (!res.success)
        {
            RCLCPP_WARN("Configure failed: %s", res.error_message);
        }

         //Publishers
        dvl_pub_report = this->create_publisher<dvl_msgs::msg::VelocityReport>("dvl/velocity", qos);
        dvl_pub_pos = this->create_publisher<dvl_msgs::msg::DeadReckoningReport>("dvl/position", qos);

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State& state)
    {
        LifecycleNode::on_activate(state);

        dvl_pub_report->on_activate();
        dvl_pub_pos->on_activate();
        
        // Services
        enable_srv = this->create_service<dvl_msgs::srv::SendCommand>(
            "enable", 
            bind(&DvlA50::send_param, this, "acoustic_enabled", true, std::_1, std::_2));

        disable_srv = this->create_service<dvl_msgs::srv::SendCommand>(
            "disable", 
            bind(&DvlA50::send_param, this, "acoustic_enabled", false, std::_1, std::_2));

        get_config_srv = this->create_service<dvl_msgs::srv::SendCommand>(
            "get_config", 
            bind(&DvlA50::send_command, this, "get_config", std::_1, std::_2));

        calibrate_gyro_srv = this->create_service<dvl_msgs::srv::SendCommand>(
            "calibrate_gyro", 
            bind(&DvlA50::send_command, this, "calibrate_gyro", std::_1, std::_2));

        reset_dead_reckoning_srv = this->create_service<dvl_msgs::srv::SendCommand>(
            "reset_dead_reckoning", 
            bind(&DvlA50::send_command, this, "reset_dead_reckoning", std::_1, std::_2));

        trigger_ping_srv = this->create_service<dvl_msgs::srv::SendCommand>(
            "trigger_ping", 
            bind(&DvlA50::send_command, this, "trigger_ping", std::_1, std::_2));

        // Start reading data
        RCLCPP_INFO("Starting to receive reports at <= %f Hz", rate);
        timer = this->create_wall_timer(
            std::chrono::duration<double>(1. / rate), 
            std::bind(&DvlA50Node::publish, this)
        );

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state)
    {
        LifecycleNode::on_deactivate(state);
        RCLCPP_INFO("Stopping report reception");

        // Stop reading data
        timer->cancel();

        dvl_pub_report->on_deactivate();
        dvl_pub_pos->on_deactivate();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state)
    {
        dvl.disconnect();
        timer.reset();

        dvl_pub_report.reset();
        dvl_pub_pos.reset();

        get_config_srv.reset();
        calibrate_gyro_srv.reset();
        reset_dead_reckoning_srv.reset();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state)
    {
        return CallbackReturn::SUCCESS;
    }


    void publish()
    {
        Response res = dvl.receive();

        if (res.type == Response::VelocityReport)
        {
            dvl_msgs::msg::VelocityReport msg;
            dvl_msgs::msg::DVLBeam beam0;
            dvl_msgs::msg::DVLBeam beam1;
            dvl_msgs::msg::DVLBeam beam2;
            dvl_msgs::msg::DVLBeam beam3;

            msg.header.frame_id = frame;
            msg.header.stamp = LifecycleNode::now();
                
            msg.time = double(json_data["time"]);
            msg.velocity.x = double(json_data["vx"]);
            msg.velocity.y = double(json_data["vy"]);
            msg.velocity.z = double(json_data["vz"]);
            msg.fom = double(json_data["fom"]);

            for (size_t i = 0; i < 3; i++)
            {
                for (size_t j = 0; j < 3; j++)
                {
                    double val = double(json_data["covariance"][i][j])
                    msg.covariance.push_back(val);
                }
            }

            double current_altitude = double(json_data["altitude"]);
            if(current_altitude >= 0.0 && msg.velocity_valid)
                old_altitude = msg.altitude = current_altitude;
            else
                msg.altitude = old_altitude;

            msg.time_of_validity = uint64_t(json_data["time_of_validity"]);
            msg.time_of_transmission = uint64_t(json_data["time_of_transmission"]);

            msg.velocity_valid = json_data["velocity_valid"];
            msg.status = uint8_t(json_data["status"]);
            msg.form = json_data["format"];
                    
            beam0.id = json_data["transducers"][0]["id"];
            beam0.velocity = double(json_data["transducers"][0]["velocity"]);
            beam0.distance = double(json_data["transducers"][0]["distance"]);
            beam0.rssi = double(json_data["transducers"][0]["rssi"]);
            beam0.nsd = double(json_data["transducers"][0]["nsd"]);
            beam0.valid = json_data["transducers"][0]["beam_valid"];
                    
            beam1.id = json_data["transducers"][1]["id"];
            beam1.velocity = double(json_data["transducers"][1]["velocity"]);
            beam1.distance = double(json_data["transducers"][1]["distance"]);
            beam1.rssi = double(json_data["transducers"][1]["rssi"]);
            beam1.nsd = double(json_data["transducers"][1]["nsd"]);
            beam1.valid = json_data["transducers"][1]["beam_valid"];
                    
            beam2.id = json_data["transducers"][2]["id"];
            beam2.velocity = double(json_data["transducers"][2]["velocity"]);
            beam2.distance = double(json_data["transducers"][2]["distance"]);
            beam2.rssi = double(json_data["transducers"][2]["rssi"]);
            beam2.nsd = double(json_data["transducers"][2]["nsd"]);
            beam2.valid = json_data["transducers"][2]["beam_valid"];
                    
            beam3.id = json_data["transducers"][3]["id"];
            beam3.velocity = double(json_data["transducers"][3]["velocity"]);
            beam3.distance = double(json_data["transducers"][3]["distance"]);
            beam3.rssi = double(json_data["transducers"][3]["rssi"]);
            beam3.nsd = double(json_data["transducers"][3]["nsd"]);
            beam3.valid = json_data["transducers"][3]["beam_valid"];
                    
            msg.beams = {beam0, beam1, beam2, beam3};
            velocity_pub->publish(msg);
        }
        else if (res.type == Response::DeadReckoningReport)
        {
            dvl_msgs::msg::DeadReckoningReport msg;

            //std::cout << std::setw(4) << json_data << std::endl;
            msg.header.frame_id = frame;
            msg.header.stamp = LifecycleNode::now();

            msg.time = double(json_data["ts"]);

            msg.position.x = double(json_data["x"]);
            msg.position.y = double(json_data["y"]);
            msg.position.z = double(json_data["z"]);

            msg.pos_std = double(json_data["std"]);

            msg.roll = double(json_data["roll"]);
            msg.pitch = double(json_data["pitch"]);
            msg.yaw = double(json_data["yaw"]);

            msg.type = json_data["type"];
            msg.status = json_data["status"];
            msg.format = json_data["format"];

            dead_reckoning_pub->publish(msg);
        }
        else
        {
            RCLCPP_WARN("Received unexpected DVL response of type %i: %s", res.type, res.data.dump().c_str());
        }
    }


    void send_command(
        std::string command,
        dvl_msgs::srv::DvlCommand::Request::SharedPtr req,
        dvl_msgs::srv::DvlCommand::Response::SharedPtr res)
    {
        Response res_json = json.send_command(command);
        
        res->response_to = json_data["response_to"];
        res->success = json_data["success"];
        res->error_message = json_data["error_message"];
        res->result = json_data["result"];
        res->format = json_data["format"];
        res->type = json_data["type"];
    }

    template<type T>
    void send_param(
        std::string param,
        const T& value,
        dvl_msgs::srv::DvlCommand::Request::SharedPtr req,
        dvl_msgs::srv::DvlCommand::Response::SharedPtr res)
    {
        Response res_json = json.send_config_param(param, value);
        
        res->response_to = json_data["response_to"];
        res->success = json_data["success"];
        res->error_message = json_data["error_message"];
        res->result = json_data["result"];
        res->format = json_data["format"];
        res->type = json_data["type"];
    }


private:
    DvlA50 dvl;

    std::string ip_address;
    std::string frame;
    double rate;
    double old_altitude;
    
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp_lifecycle::LifecyclePublisher<dvl_msgs::msg::VelocityReport>::SharedPtr velocity_pub;
    rclcpp_lifecycle::LifecyclePublisher<dvl_msgs::msg::DeadReckoningReport>::SharedPtr dead_reckoning_pub;
    rclcpp::Service<dvl_msgs::srv::DvlCommand>::SharedPtr enable_srv;
    rclcpp::Service<dvl_msgs::srv::DvlCommand>::SharedPtr disable_srv;
    rclcpp::Service<dvl_msgs::srv::DvlCommand>::SharedPtr get_config_srv;
    rclcpp::Service<dvl_msgs::srv::DvlCommand>::SharedPtr calibrate_gyro_srv;
    rclcpp::Service<dvl_msgs::srv::DvlCommand>::SharedPtr reset_dead_reckoning_srv;
    rclcpp::Service<dvl_msgs::srv::DvlCommand>::SharedPtr trigger_ping_srv;
}


int main(int argc, char * argv[])
{
    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    std::shared_ptr<DvlA50> node = std::make_shared<DvlA50>("dvl_a50");
    exe.add_node(node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
    return 0;
}
