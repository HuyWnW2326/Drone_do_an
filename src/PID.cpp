#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/input_rc.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <math.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace geometry_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        vehicle_attitude_setpoint_publisher_= this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
        UAV_publisher_= this->create_publisher<geometry_msgs::msg::Point>("/UAV_position", 10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
        
        vehicle_local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
            pos_cur.x = msg->x;
            pos_cur.y = msg->y;
            pos_cur.z = msg->z;

            vel_cur.x = msg->vx;
            vel_cur.y = msg->vy;
            vel_cur.z = msg->vz;
        });     

        vehicle_global_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>("/fmu/out/vehicle_global_position", qos,
        [this](const px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg) {
            lon_current = msg->lon;
            lat_current = msg->lat;
            alt_current = msg->alt;
        });

        vehicle_attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos,
        [this](const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
            float w = msg->q[0];
            float x = msg->q[1];
            float y = msg->q[2];
            float z = msg->q[3];

            Roll_current = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) ;
            Pitch_current = asinf(std::clamp(2.0f * (w * y - z * x), -1.0f, 1.0f));
            Yaw_current = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) ;
        });

        input_rc_subscription_ = this->create_subscription<px4_msgs::msg::InputRc>("/fmu/out/input_rc", qos,
        [this](const px4_msgs::msg::InputRc::UniquePtr msg) {
            Rc_CH6 = msg->values[5];
        });

        // point1_subscription_ = this->create_subscription<geometry_msgs::msg::Point>("/point1", qos,
        // [this](const geometry_msgs::msg::Point::UniquePtr msg) {
        //     lat_target[0] = msg->x;
        //     lon_target[0] = msg->y;
        // });

        // point2_subscription_ = this->create_subscription<geometry_msgs::msg::Point>("/point2", qos,
        // [this](const geometry_msgs::msg::Point::UniquePtr msg) {
        //     lat_target[1] = msg->x;
        //     lon_target[1] = msg->y;
        // });

        // point3_subscription_ = this->create_subscription<geometry_msgs::msg::Point>("/point3", qos,
        // [this](const geometry_msgs::msg::Point::UniquePtr msg) {
        //     lat_target[2] = msg->x;
        //     lon_target[2] = msg->y;
        // });

        

        auto timer_callback = [this]() -> void {
            lat_target[0] = 10.850496;
            lon_target[0] = 106.770971;

            lat_target[1] = 10.850402;
            lon_target[1] = 106.770954;

            lat_target[2] = 10.850549;
            lon_target[2] = 106.770933;
            if(timer_count >= 50)
            {
                timer_count = 0;
                RCLCPP_INFO(this->get_logger(), "lat_target1 = %.6f", lat_target[0]);
                RCLCPP_INFO(this->get_logger(), "lon_target1 = %.6f", lon_target[0]);
                RCLCPP_INFO(this->get_logger(), "lat_target2 = %.6f", lat_target[1]);
                RCLCPP_INFO(this->get_logger(), "lon_target2 = %.6f", lon_target[1]);
                RCLCPP_INFO(this->get_logger(), "lat_target3 = %.6f", lat_target[2]);
                RCLCPP_INFO(this->get_logger(), "lon_target3 = %.6f", lon_target[2]);
                // RCLCPP_INFO(this->get_logger(), "x_d = %.3f", pos_d.x);
                // RCLCPP_INFO(this->get_logger(), "y_d = %.3f", pos_d.y);
                // RCLCPP_INFO(this->get_logger(), "z_d = %.3f", pos_d.z);
                // RCLCPP_INFO(this->get_logger(), "x_current = %.3f", pos_cur.x);
                // RCLCPP_INFO(this->get_logger(), "y_current = %.3f", pos_cur.y);
                // RCLCPP_INFO(this->get_logger(), "z_current = %.3f", pos_cur.z);
                std::cout << std::endl;
            }
            t_end = this->get_clock()->now().seconds();
            if(Rc_CH6 > 1500)
            {
                if(state_offboard == 0)
                {
                    pos_global_2_local(lat_target[0], lon_target[0]);
                    pos_d.z = pos_cur.z;
                    Yaw_d = Yaw_current;
                    t_start = this->get_clock()->now().seconds();
                    state_offboard = 1;
                }
                else if((t_end - t_start >= 5) && (state_offboard == 1))
                {
                    pos_global_2_local(lat_target[1], lon_target[1]);
                    t_start = this->get_clock()->now().seconds();
                    state_offboard = 2;
                }
                else if((t_end - t_start >= 7) && (state_offboard == 2))
                {
                    pos_global_2_local(lat_target[2], lon_target[2]);
                    state_offboard = 3;
                }
            }
            else 
                state_offboard = 0;
            publish_offboard_control_mode();
            if((state_offboard) && (Rc_CH6 > 1500))
                Controller_xyz(pos_d.x, pos_d.y, pos_d.z);
            timer_count ++;
        };
        timer_ = this->create_wall_timer(10ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr UAV_publisher_;

    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;
    rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr vehicle_global_position_subscription_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;
    rclcpp::Subscription<InputRc>::SharedPtr input_rc_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point1_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point2_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point3_subscription_;

    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
    std::atomic<uint64_t> t_start;
    std::atomic<uint64_t> t_end;

    uint64_t timer_count = 0;
    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    // Parameter of Quadcopter
    float fz = 0.0f;     
    float g = 9.8f;
    float m = 1.820f;
    float b = 4.6 * pow(10,-6);
    float omg_max = 1285.0f;
    // float m = 1.545;
    // float b = 4.6 * pow(10,-6);
    // float omg_max = 1100;
    const double f_max = 4 * b * pow(omg_max,2); 

    float lon_current = 0.0f;
    float lat_current = 0.0f;
    float alt_current = 0.0f;

    float lon_target[3];
    float lat_target[3];

    float delta_lon = 0.0f;
    float delta_lat = 0.0f;
    float phi = 0.0f;

    Vector3 pos_d;
    Vector3 pos_cur;
    Vector3 vel_d;
    Vector3 vel_cur;

    float Roll_current  = 0.0f;
    float Pitch_current = 0.0f;
    float Yaw_current   = 0.0f;
    float Yaw_d = 0.0f;

    float Rc_CH6 = 0.0f;

    uint8_t state_offboard = 0;

    void publish_offboard_control_mode();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
    void publish_vehicle_attitude_setpoint(float thrust_z, std::array<float, 4> quaternion);
    std::array<float, 4> RPY_to_Quaternion(float Roll, float Pitch, float Yaw);
    float Altitude_controller(float DesiredZ);
    void Controller_xyz(float DesiredX, float DesiredY, float DesiredZ);
    void xyz_setpoint(float x_desired, float y_desired, float z_desired);
    std::array<float ,2> constrainXY(float vel_x, float vel_y, float vel_max);
    void pos_global_2_local(float lat_target, float lon_target);
    float saturation(float value, float min, float max);
};

void OffboardControl::pos_global_2_local(float lat_target, float lon_target)
{
    phi = (lat_target + lat_current) / 2 * (M_PI /180);
    delta_lat = (lat_target - lat_current);   // Don vi do
    delta_lon = (lon_target - lon_current);   // Don vi do
    pos_d.x = pos_cur.x + delta_lat * 111320.0f;   // 1 do = 111320.0 m
    pos_d.y = pos_cur.y + delta_lon * 111320.0f * cosf(phi); // 1 do kinh do = 111320.0 * cosd(trung binh)
}

void OffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = true;
    msg.direct_actuator = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

void OffboardControl::publish_vehicle_attitude_setpoint(float thrust_z, std::array<float, 4> quaternion)
{
    VehicleAttitudeSetpoint msg{};
    msg.thrust_body[0] = 0.0;
    msg.thrust_body[1] = 0.0;
    msg.thrust_body[2] = thrust_z;
    msg.q_d[0] = quaternion[0];
    msg.q_d[1] = quaternion[1];
    msg.q_d[2] = quaternion[2];
    msg.q_d[3] = quaternion[3];
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_attitude_setpoint_publisher_->publish(msg);
}

std::array<float, 4> OffboardControl::RPY_to_Quaternion(float Roll, float Pitch, float Yaw)
{
    std::array<float, 4> quaternion;
    float cy = cos(Yaw/2);
    float sy = sin(Yaw/2);
    float cp = cos(Pitch/2);
    float sp = sin(Pitch/2);
    float cr = cos(Roll/2);
    float sr = sin(Roll/2);

    quaternion[0] = cr * cp * cy + sr * sp * sy;
    quaternion[1] = sr * cp * cy - cr * sp * sy;
    quaternion[2] = cr * sp * cy + sr * cp * sy;
    quaternion[3] = cr * cp * sy - sr * sp * cy;
    return quaternion;
}

float OffboardControl::Altitude_controller(float DesiredZ)
{
    float w_n = 4.0f;
    float zeta = 0.8f;
    float kp1 = 2.0f * w_n * zeta;
    float kp2 = w_n / (2.0f * zeta);
    
    float vel_z_d = saturation((DesiredZ - pos_cur.z) * kp2, -2.0f, 2.0f);
    float uz = kp1 * (vel_z_d - vel_cur.z);
    fz = ((g - uz) * m) / (cosf(Roll_current) * cosf(Pitch_current));
    float fz_out = -fz/f_max;
    return fz_out;
}

void OffboardControl::Controller_xyz(float DesiredX, float DesiredY, float DesiredZ)
{
    float w_n = 0.9f;
    float zeta = 0.8f;
    float kp1 = 2.0f * w_n * zeta;
    float kp2 = w_n / (2.0f * zeta);

    std::array<float ,2> vel_xy = constrainXY((DesiredX - pos_cur.x) * kp2, (DesiredY - pos_cur.y) * kp2, 4.0f);

    float ux1 = kp1 * (vel_xy[0] - vel_cur.x);
    float uy1 = kp1 * (vel_xy[1] - vel_cur.y);

    float ux = -(ux1 * m) / fz;
    float uy = -(uy1 * m) / fz;

    float Roll_d  = asin(std::clamp(ux * sinf(Yaw_d) - uy * cosf(Yaw_d), float(asin(-M_PI/8)), float(asin(M_PI/8))));
    float Pitch_d = asin(std::clamp((ux * cosf(Yaw_d) + uy * sinf(Yaw_d)) / cosf(Roll_d), float(asin(-M_PI/8)), float(asin(M_PI/8))));
    publish_vehicle_attitude_setpoint(Altitude_controller(DesiredZ), RPY_to_Quaternion(Roll_d, Pitch_d, Yaw_d));
}

void OffboardControl::xyz_setpoint(float x_desired, float y_desired, float z_desired)
{
    pos_d.x = pos_cur.x + x_desired;
    pos_d.y = pos_cur.y + y_desired;
    pos_d.z = pos_cur.z + z_desired;
}

std::array<float ,2> OffboardControl::constrainXY(float vel_x, float vel_y, float vel_max)
{
    float vel = sqrtf(vel_x * vel_x + vel_y * vel_y);
    if (vel > vel_max)
    {
        vel_x = vel_x / vel * vel_max;
        vel_y = vel_y / vel * vel_max;
    }
    return {vel_x, vel_y};
}

float OffboardControl::saturation(float value, float min, float max)
{
    if (value > max)
        return max;
    else if (value < min)
        return min;
    else
        return value;
}

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}