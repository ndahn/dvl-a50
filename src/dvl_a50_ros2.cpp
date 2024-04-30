#include <string>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <marine_acoustic_msgs/msgs/dvl.hpp>

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
        speed_of_sound = this->get_parameter("speed_of_sound").as_int();
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
        dvl_pub_report = this->create_publisher<marine_acoustic_msgs::msg::Dvl>("dvl/velocity", qos);
        dvl_pub_pos = this->create_publisher<sensor_msgs::msg::Odometry>("dvl/position", qos);

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
            marine_acoustic_msgs::msg::Dvl msg;
            
            msg.header.frame_id = frame;
            msg.header.stamp = rclcpp::Time(uint64_t(json_data["time_of_validity"]) * 1000);

            msg.velocity_mode = marine_acoustic_msgs::msg::Dvl::DVL_MODE_BOTTOM;
            msg.dvl_type = marine_acoustic_msgs::msg::Dvl::DVL_TYPE_PISTON;
            
            msg.velocity.x = double(json_data["vx"]);
            msg.velocity.y = double(json_data["vy"]);
            msg.velocity.z = double(json_data["vz"]);
            
            for (size_t i = 0; i < 3; i++)
            {
                for (size_t j = 0; j < 3; j++)
                {
                    double val = double(json_data["covariance"][i][j])
                    msg.velocity_covar.push_back(val);
                }
            }

            double current_altitude = double(json_data["altitude"]);
            if(current_altitude >= 0.0 && msg.velocity_valid)
                old_altitude = msg.altitude = current_altitude;
            else
                msg.altitude = old_altitude;

            msg.course_gnd = std::atan2(msg.velocity.y, msg.velocity.x);
            msg.speed_gnd = std::sqrt(msg.velocity.x * msg.velocity.x + msg.velocity.y * msg.velocity.y);

            msg.sound_speed = speed_of_sound;
            msg.beam_ranges_valid = true;
            msg.beam_velocities_valid = true;

            // Beam specific data
            for (size_t beam = 0; beam < 4; beam++)
            {
                msg.num_good_beams += json_data["transducers"][beam]["beam_valid"];
                msg.range = json_data["transducers"][beam]["distance"];
                //msg.range_covar
                msg.beam_quality = json_data["transducers"][beam]["rssi"];
                msg.beam_velocity = json_data["transducers"][beam]["velocity"];
                //msg.beam_velocity_covar
            }

            /*
             * Beams point 22.5° away from center, LED pointing forward
             * Transducers rotated 45° around Z
             */
            // Beam 1 (+135° from X)
            msg.beam_unit_vec[0].x = -0.6532814824381883
            msg.beam_unit_vec[0].y =  0.6532814824381883
            msg.beam_unit_vec[0].z =  0.38268343236508984

            // Beam 2 (-135° from X)
            msg.beam_unit_vec[1].x = -0.6532814824381883
            msg.beam_unit_vec[1].y = -0.6532814824381883
            msg.beam_unit_vec[1].z =  0.38268343236508984

            // Beam 3 (-45° from X)
            msg.beam_unit_vec[2].x =  0.6532814824381883
            msg.beam_unit_vec[2].y = -0.6532814824381883
            msg.beam_unit_vec[2].z =  0.38268343236508984

            // Beam 4 (+45° from X)
            msg.beam_unit_vec[3].x =  0.6532814824381883
            msg.beam_unit_vec[3].y =  0.6532814824381883
            msg.beam_unit_vec[3].z =  0.38268343236508984

            velocity_pub->publish(msg);
        }
        else if (res.type == Response::DeadReckoningReport)
        {
            sensor_msgs::msg::Odometry msg;
            
            msg.header.frame_id = frame;
            msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(double(json_data["ts"])) * 1e9);

            msg.pose.pose.position.x = double(json_data["x"]);
            msg.pose.pose.position.y = double(json_data["y"]);
            msg.pose.pose.position.z = double(json_data["z"]);

            double std_dev = double(json_data["std"]);
            msg.pose.covariance[0] = std_dev;
            msg.pose.covariance[7] = std_dev;
            msg.pose.covariance[14] = std_dev;

            tf2::Quaternion quat;
            quat.setRPY(double(json_data["roll"]), double(json_data["pitch"]), double(json_data["yaw"]))
            msg.pose.pose.orientation = tf2::toMsg(quat);

            odom_pub->publish(msg);
        }
        else
        {
            RCLCPP_WARN("Received unexpected DVL response of type %i: %s", res.type, res.data.dump().c_str());
        }
    }


    void send_command(
        std::string command,
        std_srvs::srv::Trigger::Request::SharedPtr req,
        std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        Response res_json = json.send_command(command);
        res->success = json_data["success"];
        res->message = json_data["error_message"];
    }

    template<type T>
    void send_param(
        std::string param,
        const T& value,
        std_srvs::srv::Trigger::Request::SharedPtr req,
        std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        Response res_json = json.send_config_param(param, value);
        res->success = json_data["success"];
        res->message = json_data["error_message"];
    }


private:
    DvlA50 dvl;

    std::string ip_address;
    std::string frame;
    double rate;
    int speed_of_sound;
    double old_altitude;
    
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp_lifecycle::LifecyclePublisher<marine_acoustic_msgs::msg::Dvl>::SharedPtr velocity_pub;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Odometry>::SharedPtr odom_pub;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_config_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_gyro_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_dead_reckoning_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_ping_srv;
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
