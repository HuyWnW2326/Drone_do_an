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

        point1_subscription_ = this->create_subscription<geometry_msgs::msg::Point>("/point1", qos,
        [this](const geometry_msgs::msg::Point::UniquePtr msg) {
            lat_target[0] = msg->x;
            lon_target[0] = msg->y;
        });

        point2_subscription_ = this->create_subscription<geometry_msgs::msg::Point>("/point2", qos,
        [this](const geometry_msgs::msg::Point::UniquePtr msg) {
            lat_target[1] = msg->x;
            lon_target[1] = msg->y;
        });

        point3_subscription_ = this->create_subscription<geometry_msgs::msg::Point>("/point3", qos,
        [this](const geometry_msgs::msg::Point::UniquePtr msg) {
            lat_target[2] = msg->x;
            lon_target[2] = msg->y;
        });

        auto timer_callback = [this]() -> void {
            if(timer_count >= 50)
            {
                timer_count = 0;
                // RCLCPP_INFO(this->get_logger(), "lat_target1 = %.6f", lat_target[0]);
                // RCLCPP_INFO(this->get_logger(), "lon_target1 = %.6f", lon_target[0]);
                // RCLCPP_INFO(this->get_logger(), "lat_target2 = %.6f", lat_target[1]);
                // RCLCPP_INFO(this->get_logger(), "lon_target2 = %.6f", lon_target[1]);
                // RCLCPP_INFO(this->get_logger(), "lat_target3 = %.6f", lat_target[2]);
                // RCLCPP_INFO(this->get_logger(), "lon_target3 = %.6f", lon_target[2]);
                RCLCPP_INFO(this->get_logger(), "x_d = %.3f", pos_d.x);
                RCLCPP_INFO(this->get_logger(), "y_d = %.3f", pos_d.y);
                RCLCPP_INFO(this->get_logger(), "z_d = %.3f", pos_d.z);
                RCLCPP_INFO(this->get_logger(), "x_current = %.3f", pos_cur.x);
                RCLCPP_INFO(this->get_logger(), "y_current = %.3f", pos_cur.y);
                RCLCPP_INFO(this->get_logger(), "z_current = %.3f", pos_cur.z);
                std::cout << std::endl;
            }
            t_end = this->get_clock()->now().seconds();
            if(Rc_CH6 > 1500)
            {
                // if(state_offboard == 0)
                // {
                //     pos_global_2_local(lat_target[0], lon_target[0]);
                //     pos_d.z = pos_cur.z;
                //     Yaw_d = Yaw_current;
                //     t_start = this->get_clock()->now().seconds();
                //     state_offboard = 1;
                // }
                // else if((t_end - t_start >= 5) && (state_offboard == 1))
                // {
                //     pos_global_2_local(lat_target[1], lon_target[1]);
                //     t_start = this->get_clock()->now().seconds();
                //     state_offboard = 2;
                // }
                // else if((t_end - t_start >= 7) && (state_offboard == 2))
                // {
                //     pos_global_2_local(lat_target[2], lon_target[2]);
                //     state_offboard = 3;
                // }
                if(state_offboard == 0)
                {
                    state_offboard = 1;
                    Yaw_d = Yaw_current;
                    xyz_setpoint(-5.0f, 3.0f, 0.0f);
                }
            }
            else 
                state_offboard = 0;
            publish_offboard_control_mode();
            if((state_offboard) && (Rc_CH6 > 1500))
            {
                Controller_xyz(pos_d.x, pos_d.y, pos_d.z);
            }
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
    // float m = 1.820f;
    // float b = 4.6 * pow(10,-6);
    // float omg_max = 1285.0f;
    float m = 1.545;
    float b = 4.6 * pow(10,-6);
    float omg_max = 1100;
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
    pos_d.y = pos_cur.y + delta_lon * 111320.0f * cos(phi); // 1 do kinh do = 111320.0 * cosd(trung binh)
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
    float cy = cosf(Yaw/2);
    float sy = sinf(Yaw/2);
    float cp = cosf(Pitch/2);
    float sp = sinf(Pitch/2);
    float cr = cosf(Roll/2);
    float sr = sinf(Roll/2);

    quaternion[0] = cr * cp * cy + sr * sp * sy;
    quaternion[1] = sr * cp * cy - cr * sp * sy;
    quaternion[2] = cr * sp * cy + sr * cp * sy;
    quaternion[3] = cr * cp * sy - sr * sp * cy;
    return quaternion;
}

float OffboardControl::Altitude_controller(float DesiredValueZ)
{
    float k_z = 4.0;
    float lamda_z = 3.0;
    static float tz_last = 0.0;
    static float z_d_last = 0.0;
    static float vz_d_last = 0.0;
    float vz_d = 0.0;
    float az_d = 0.0;
    float err_dot = 0.0;

    float delta_tz = this->get_clock()->now().seconds() - tz_last;
    float err_z = DesiredValueZ - pos_cur.z;

    if(delta_tz > 0)
    {
        vz_d = (DesiredValueZ - z_d_last) / delta_tz;
        az_d = (vz_d - vz_d_last) / delta_tz;
    }
    err_dot = vz_d - vel_cur.z;
    fz = (-m/(cos(Roll_current) * cos(Pitch_current))) * (az_d - g + lamda_z * err_dot + k_z * std::clamp(err_dot + lamda_z * err_z, -1.0f, 1.0f));
    
    tz_last = this->get_clock()->now().seconds();
    vz_d_last = vz_d;
    z_d_last = DesiredValueZ;
    float fz_out = -fz/f_max;
    return fz_out;
}

void OffboardControl::Controller_xyz(float DesiredValueX, float DesiredValueY, float DesiredValueZ)
{
    float k_x = 4.0;
    float lamda_x = 2.0;
    static float tx_last = 0.0;
    float delta_tx = this->get_clock()->now().seconds() - tx_last;

    static float x_d_last = 0.0;
    static float vx_d_last = 0.0;
    float vx_d = 0.0;
    float ax_d = 0.0;
    float err_x_dot = 0.0;
    float err_x = DesiredValueX - pos_cur.x;
    if(delta_tx > 0)
    {
        vx_d = (DesiredValueX - x_d_last) / delta_tx;
        ax_d = (vx_d - vx_d_last) / delta_tx;
    }
    err_x_dot = vx_d - vel_cur.x;
    float ux1 = (-m/fz) * (ax_d + lamda_x * err_x_dot + k_x * std::clamp(err_x_dot + lamda_x * err_x, -1.0f, 1.0f));
    tx_last = this->get_clock()->now().seconds();
    vx_d_last = vx_d;
    x_d_last = DesiredValueX;

    float k_y = 4.0;
    float lamda_y = 2.0;
    static float ty_last = 0.0;
    float delta_ty = this->get_clock()->now().seconds() - ty_last;

    static float y_d_last = 0.0;
    static float vy_d_last = 0.0;
    float vy_d = 0.0;
    float ay_d = 0.0;
    float err_y_dot = 0.0;
    float err_y = DesiredValueY - pos_cur.y;
    if(delta_ty > 0)
    {
        vy_d = (DesiredValueY - y_d_last) / delta_ty;
        ay_d = (vy_d - vy_d_last) / delta_ty;
    }
    err_y_dot = vy_d - vel_cur.y;
    float uy1 = (-m/fz) * (ay_d + lamda_y * err_y_dot + k_y * std::clamp(err_y_dot + lamda_y * err_y, -1.0f, 1.0f));
    ty_last = this->get_clock()->now().seconds();
    vy_d_last = vy_d;
    y_d_last = DesiredValueY;

    float Roll_d  = asin(std::clamp(ux1 * sin(Yaw_d) - uy1 * cos(Yaw_d), float(asin(-M_PI/6)), float(asin(M_PI/6))));
    float Pitch_d = asin(std::clamp((ux1 * cos(Yaw_d) + uy1 * sin(Yaw_d)) / cos(Roll_d), float(asin(-M_PI/6)), float(asin(M_PI/6))));

    publish_vehicle_attitude_setpoint(Altitude_controller(DesiredValueZ), RPY_to_Quaternion(Roll_d, Pitch_d, Yaw_d));
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