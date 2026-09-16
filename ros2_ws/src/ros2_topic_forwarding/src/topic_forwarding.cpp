#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>

#include "ros2_topic_forwarding/msg/robot_data_message.hpp"

#include <nlohmann/json.hpp>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <mutex>
#include <string>
#include <csignal>


using json = nlohmann::json;


class Forwarder : public rclcpp::Node
{

public:

    Forwarder()
    : Node("topic_forwarding"),
      sock_(-1)
    {

       this->declare_parameter(
               "server_ip",
               "127.0.0.1"
               );


       this->declare_parameter(
               "server_port",
                9000
               );
     ip_ =
       this->get_parameter(
               "server_ip"
               ).as_string();
     port_ =
       this->get_parameter(
               "server_port"
               ).as_int();
     RCLCPP_INFO(
       this->get_logger(),
               "Server: %s:%d",
               ip_.c_str(),
              port_
               );

     // 启动时就建立连接（和官方一致）
     connect_server();

     imu_sub_ =
        this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu",
            10,
            std::bind(
                &Forwarder::imu_callback,
                this,
                std::placeholders::_1)
        );


        vel_sub_ =
        this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(
                &Forwarder::vel_callback,
                this,
                std::placeholders::_1)
        );


        acc_sub_ =
        this->create_subscription<geometry_msgs::msg::Accel>(
            "/cmd_accel",
            10,
            std::bind(
                &Forwarder::acc_callback,
                this,
                std::placeholders::_1)
        );


        joy_sub_ =
        this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(
                &Forwarder::joy_callback,
                this,
                std::placeholders::_1)
        );


        mag_sub_ =
        this->create_subscription<sensor_msgs::msg::MagneticField>(
            "/magnetometer",
            10,
            std::bind(
                &Forwarder::mag_callback,
                this,
                std::placeholders::_1)
        );


        pressure_sub_ =
        this->create_subscription<sensor_msgs::msg::FluidPressure>(
            "/pressure",
            10,
            std::bind(
                &Forwarder::pressure_callback,
                this,
                std::placeholders::_1)
        );


        robot_sub_ =
        this->create_subscription<
        ros2_topic_forwarding::msg::RobotDataMessage>(
            "/robot_data",
            10,
            std::bind(
                &Forwarder::robot_callback,
                this,
                std::placeholders::_1)
        );
        RCLCPP_INFO(
            this->get_logger(),
            "ROS2 topic forwarding started"
        );
    }
private:
    void send_json(const json &j)
    {
     // 官方用 \r\n 作为行尾符
     std::string data = j.dump() + "\r\n";
     std::lock_guard<std::mutex> lock(mutex_);
        RCLCPP_INFO(
               this->get_logger(),
               "Sending JSON: %s",
                data.c_str()
        );

         if(sock_ < 0)
    {
        connect_server();
    }
 if(sock_ >= 0)
        {
            int ret = send(
                sock_,
                data.c_str(),
                data.size(),
                0
             );
            RCLCPP_INFO(
                this->get_logger(),
                "send() returned: %d",
                ret
             );

            if(ret <= 0)
            {
                 RCLCPP_ERROR(
                   this->get_logger(),
                   "Failed to send data"
                );

                close(sock_);
                sock_ = -1;
             }
         }
         else
         {
              RCLCPP_ERROR(
                  this->get_logger(),
                "Socket is not connected"
               );
            }
         }
   void connect_server()
  {
    RCLCPP_INFO(
        this->get_logger(),
        "Connecting to server %s:%d",
        ip_.c_str(),
        port_
    );

    // 创建 TCP socket
    sock_ = socket(AF_INET, SOCK_STREAM, 0);

    if(sock_ < 0)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to create socket"
        );
        return;
    }

    // 使用 getaddrinfo 解析服务器域名/IP
    struct addrinfo hints{};
    struct addrinfo *result = nullptr;

    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    std::string port_string = std::to_string(port_);

    int ret = getaddrinfo(
        ip_.c_str(),
        port_string.c_str(),
        &hints,
        &result
    );

    if(ret != 0)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "DNS resolution failed: %s",
            gai_strerror(ret)
        );

        close(sock_);
        sock_ = -1;
        return;
    }

    // 尝试连接服务器
    if(connect(
        sock_,
        result->ai_addr,
        result->ai_addrlen
    ) < 0)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to connect to %s:%d",
            ip_.c_str(),
            port_
        );

        freeaddrinfo(result);

        close(sock_);
        sock_ = -1;
        return;
    }

    // 释放 DNS 解析结果
    freeaddrinfo(result);

    RCLCPP_INFO(
        this->get_logger(),
        "TCP connection established: %s:%d",
        ip_.c_str(),
        port_
    );
}

    // ========== 以下所有回调均按照官方 JSON 格式：数据嵌套在 data 对象里 ==========

    void imu_callback(
        const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        json json_data;
        json_data["topic"] = "imu";

        json data;
        data["Linear Acceleration"]["x"] = msg->linear_acceleration.x;
        data["Linear Acceleration"]["y"] = msg->linear_acceleration.y;
        data["Linear Acceleration"]["z"] = msg->linear_acceleration.z;
        data["Angular Velocity"]["x"] = msg->angular_velocity.x;
        data["Angular Velocity"]["y"] = msg->angular_velocity.y;
        data["Angular Velocity"]["z"] = msg->angular_velocity.z;
        data["Orientation"]["x"] = msg->orientation.x;
        data["Orientation"]["y"] = msg->orientation.y;
        data["Orientation"]["z"] = msg->orientation.z;
        data["Orientation"]["w"] = msg->orientation.w;

        json_data["data"] = data;
        send_json(json_data);
    }

    void vel_callback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Received /cmd_vel: linear x=%.2f y=%.2f z=%.2f | angular x=%.2f y=%.2f z=%.2f",
            msg->linear.x, msg->linear.y, msg->linear.z,
            msg->angular.x, msg->angular.y, msg->angular.z
        );

        json json_data;
        json_data["topic"] = "cmd_vel";

        json data;

        json linear_velocity;
        linear_velocity["x"] = msg->linear.x;
        linear_velocity["y"] = msg->linear.y;
        linear_velocity["z"] = msg->linear.z;
        data["Linear Velocity"] = linear_velocity;

        json angular_velocity;
        angular_velocity["x"] = msg->angular.x;
        angular_velocity["y"] = msg->angular.y;
        angular_velocity["z"] = msg->angular.z;
        data["Angular Velocity"] = angular_velocity;

        json_data["data"] = data;
        send_json(json_data);
    }

    void acc_callback(
        const geometry_msgs::msg::Accel::SharedPtr msg)
    {
        json json_data;
        json_data["topic"] = "cmd_accel";

        json data;
        data["Linear Velocity"]["x"] = msg->linear.x;
        data["Linear Velocity"]["y"] = msg->linear.y;
        data["Linear Velocity"]["z"] = msg->linear.z;
        data["Angular Velocity"]["x"] = msg->angular.x;
        data["Angular Velocity"]["y"] = msg->angular.y;
        data["Angular Velocity"]["z"] = msg->angular.z;

        json_data["data"] = data;
        send_json(json_data);
    }

    void joy_callback(
        const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        json json_data;
        json_data["topic"] = "joy";

        json data;

        // ROS2 的 Header 没有 seq，设为 0 保持字段兼容
        json header;
        header["Seq"] = 0;
        header["Stamp"] = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
        header["Frame ID"] = msg->header.frame_id;
        data["Header"] = header;

        for (size_t i = 0; i < msg->axes.size(); ++i) {
            std::string axis_name = "Axis " + std::to_string(i);
            data[axis_name] = msg->axes[i];
        }

        for (size_t i = 0; i < msg->buttons.size(); ++i) {
            std::string button_name = "Button " + std::to_string(i);
            data[button_name] = msg->buttons[i];
        }

        json_data["data"] = data;
        send_json(json_data);
    }

    void mag_callback(
        const sensor_msgs::msg::MagneticField::SharedPtr msg)
    {
        json json_data;
        json_data["topic"] = "magnetometer";

        json data;
        data["Magnetic Field"]["x"] = msg->magnetic_field.x;
        data["Magnetic Field"]["y"] = msg->magnetic_field.y;
        data["Magnetic Field"]["z"] = msg->magnetic_field.z;

        json covariance;
        for (int i = 0; i < 9; ++i) {
            covariance.push_back(msg->magnetic_field_covariance[i]);
        }
        data["Magnetic Field Covariance"] = covariance;

        json_data["data"] = data;
        send_json(json_data);
    }

    void pressure_callback(
        const sensor_msgs::msg::FluidPressure::SharedPtr msg)
    {
        json json_data;
        json_data["topic"] = "pressure";

        json data;
        data["Fluid Pressure"] = msg->fluid_pressure;
        data["Variance"] = msg->variance;

        json_data["data"] = data;
        send_json(json_data);
    }

    void robot_callback(
        const ros2_topic_forwarding::msg::RobotDataMessage::SharedPtr msg)
    {
        json json_data;
        json_data["topic"] = "robot_data";

        json data;
        data["roll"] = msg->roll;
        data["yaw"] = msg->yaw;
        data["cabinHumi"] = msg->cabin_hmi;
        data["pitch"] = msg->pitch;
        data["longitude"] = msg->longitude;
        data["latitude"] = msg->latitude;
        data["depth"] = msg->depth;
        data["speed"] = msg->speed;
        data["cabinTemp"] = msg->cabin_temp;
        data["times"] = msg->times;
        data["magneticField"] = msg->magnetic_field;
        data["acceleratedSpeed"] = msg->accelerated_speed;
        data["cabinPres"] = msg->cabin_pres;
        data["batteryVol"] = msg->battery_vol;
        data["clawCur"] = msg->claw_cur;

        json_data["data"] = data;
        send_json(json_data);
    }

private:
    std::string ip_;
    int port_;
    int sock_;
    std::mutex mutex_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Accel>::SharedPtr acc_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;

    rclcpp::Subscription<
    ros2_topic_forwarding::msg::RobotDataMessage>::SharedPtr robot_sub_;

};

int main(
int argc,
char **argv)
{

    // 忽略 SIGPIPE，防止向已关闭的 socket 发送数据时进程被杀死
    signal(SIGPIPE, SIG_IGN);

    rclcpp::init(argc,argv);
    rclcpp::spin(
        std::make_shared<Forwarder>()
    );

    rclcpp::shutdown();

    return 0;
}
