#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/input_rc.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/bool.hpp>

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
        vehicle_attitude_setpoint_publisher_= this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
        servo_publisher_= this->create_publisher<std_msgs::msg::Bool>("/servo", 10);

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

        auto timer_callback = [this]() -> void {
            if((Rc_CH6 >= 1500) && (state_offboard == 0))
            {
                state_offboard = 1;
                x_center = pos_cur.x;
                y_center = pos_cur.y - 3;
                pos_d.z = pos_cur.z;
				t_start = this->get_clock()->now().seconds();
                // pos_d.x = pos_cur.x;
                // pos_d.y = pos_cur.y;
                // pos_d.z = pos_cur.z;
                Yaw_d =  Yaw_current;
                std::cout << "Offboard_mode  =" << std::endl;
            }
            else if((Rc_CH6 <= 1500) && (state_offboard == 1)) 
            {
                state_offboard = 0;
                std::cout << "Position_mode  =" << std::endl;
            }
            publish_offboard_control_mode();
            if(state_offboard == 1 && Rc_CH6 >= 1500)
            {
                timer_count ++;
				delta_t = this->get_clock()->now().seconds() - t_start;
				pos_d.x = x_center + 3 * sin(2 * M_PI / 10 * delta_t);
				pos_d.y = y_center + 3 * cos(2 * M_PI / 10 * delta_t);
                if(timer_count >= 1200 && !FLAG_SERVO)
                {
                    FLAG_SERVO = true;
                    servo(true);
                }
                Controller_xy(pos_d.x, pos_d.y);
                Altitude_controller(pos_d.z);
            }
        };
        timer_ = this->create_wall_timer(10ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr servo_publisher_;

    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;
    rclcpp::Subscription<InputRc>::SharedPtr input_rc_subscription_;

    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
    uint64_t timer_count = 0;
    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    // Parameter of Quadcopter
    float fz = 0.0f;     
    float g = 9.8f;
    float m = 1.820f;
    float b = 4.6 * pow(10,-6);
    float omg_max = 1285.0f;
    // const float m = 1.545f;
    // const float b = 4.6f * pow(10,-6);
    // const float omg_max = 1100.0f;
    const double f_max = 4 * b * pow(omg_max,2); 

    Vector3 pos_d;
    Vector3 vel_d;
    Vector3 pos_cur;
    Vector3 vel_cur;

    float Yaw_d = 0.0f;
    float Roll_current  = 0.0f;
    float Pitch_current = 0.0f;
    float Yaw_current   = 0.0f;

    float Rc_CH6 = 0.0f;
    uint8_t state_offboard = 0;
    float quaternion[4];

	float t_start = 0.0f;
	float delta_t = 0.0f;
	float x_center = 0.0f;
	float y_center = 0.0f;
    bool FLAG_SERVO = false;

    void publish_offboard_control_mode();
    void publish_vehicle_attitude_setpoint(float thrust_z);
    void servo(bool data);
    void RPY_to_Quaternion(float Roll, float Pitch, float Yaw);
    void Altitude_controller(float DesiredValueZ);
    void Controller_xy(float DesiredValueX, float DesiredValueY);
};

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

void OffboardControl::servo(bool data)
{
    std_msgs::msg::Bool msg{};
    msg.data = data;
    servo_publisher_->publish(msg);
}

void OffboardControl::publish_vehicle_attitude_setpoint(float thrust_z)
{
    VehicleAttitudeSetpoint msg{};
    msg.thrust_body[0] = 0.0f;
    msg.thrust_body[1] = 0.0f;
    msg.thrust_body[2] = thrust_z;
    msg.q_d[0] = quaternion[0];
    msg.q_d[1] = quaternion[1];
    msg.q_d[2] = quaternion[2];
    msg.q_d[3] = quaternion[3];
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_attitude_setpoint_publisher_->publish(msg);
}

void OffboardControl::RPY_to_Quaternion(float Roll, float Pitch, float Yaw)
{
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
}

void OffboardControl::Altitude_controller(float DesiredValueZ)
{
    float k_z = 3.0f;
    float lamda_z = 2.0f;
    float c_z = 0.1f;
    static float tz_last = 0.0f;
    static float z_d_last = 0.0f;
    static float vz_d_last = 0.0f;
    static float integral_z = 0.0f;
    float vz_d = 0.0f;
    float az_d = 0.0f;
    float err_dot = 0.0f;

    float delta_tz = this->get_clock()->now().seconds() - tz_last;
    float err_z = DesiredValueZ - pos_cur.z;

    if(delta_tz > 0)
    {
        vz_d = (DesiredValueZ - z_d_last) / delta_tz;
        az_d = (vz_d - vz_d_last) / delta_tz;
    } 
    err_dot = vz_d - vel_cur.z;
    integral_z = std::clamp(integral_z + c_z * err_z * delta_tz, -0.15f, 0.15f); // Limit the integral term to prevent windup
    float s_z = err_dot + lamda_z * err_z + integral_z;
    fz = (-m/(cosf(Roll_current) * cosf(Pitch_current))) * (az_d - g + lamda_z * err_dot + c_z * err_z + k_z * std::clamp(s_z, -1.0f, 1.0f));

    tz_last = this->get_clock()->now().seconds();
    vz_d_last = vz_d;
    z_d_last = DesiredValueZ;
    float fz_out = -fz/f_max;
    publish_vehicle_attitude_setpoint(fz_out);
}

void OffboardControl::Controller_xy(float DesiredValueX, float DesiredValueY)
{
    float k_x = 3.0f;
    float lamda_x = 1.5f;
    float c_x = 0.07f;
    static float tx_last = 0.0f;
    float delta_tx = this->get_clock()->now().seconds() - tx_last;

    static float x_d_last = 0.0f;
    static float vx_d_last = 0.0f;
    static float integral_x = 0.0f;
    float vx_d = 0.0f;
    float ax_d = 0.0f;
    float err_x_dot = 0.0f;
    float err_x = DesiredValueX - pos_cur.x;
    if(delta_tx > 0)
    {
        vx_d = (DesiredValueX - x_d_last) / delta_tx;
        ax_d = (vx_d - vx_d_last) / delta_tx;
    }
    err_x_dot = vx_d - vel_cur.x;
    integral_x = std::clamp(integral_x + c_x * err_x * delta_tx, -0.15f, 0.15f);
    float s_x = err_x_dot + lamda_x * err_x + integral_x;
    float ux1 = (-m/fz) * (ax_d + lamda_x * err_x_dot + c_x * err_x + k_x * std::clamp(s_x, -1.0f, 1.0f));
    tx_last = this->get_clock()->now().seconds();
    vx_d_last = vx_d;
    x_d_last = DesiredValueX;

    float k_y = 3.0f;
    float lamda_y = 1.5f;
    float c_y = 0.07f;
    static float ty_last = 0.0f;
    float delta_ty = this->get_clock()->now().seconds() - ty_last;

    static float y_d_last = 0.0f;
    static float vy_d_last = 0.0f;
    static float integral_y = 0.0f;
    float vy_d = 0.0f;
    float ay_d = 0.0f;
    float err_y_dot = 0.0f;
    float err_y = DesiredValueY - pos_cur.y;
    if(delta_ty > 0)
    {
        vy_d = (DesiredValueY - y_d_last) / delta_ty;
        ay_d = (vy_d - vy_d_last) / delta_ty;
    }
    err_y_dot = vy_d - vel_cur.y;
    integral_y = std::clamp(integral_y + c_y * err_y * delta_ty, -0.15f, 0.15f);
    float s_y = err_y_dot + lamda_y * err_y + integral_y;
    float uy1 = (-m/fz) * (ay_d + lamda_y * err_y_dot + c_y * err_y + k_y * std::clamp(s_y, -1.0f, 1.0f));
    ty_last = this->get_clock()->now().seconds();
    vy_d_last = vy_d;
    y_d_last = DesiredValueY;

    float Roll_d  = asin(std::clamp(ux1 * sinf(Yaw_d) - uy1 * cosf(Yaw_d), float(asin(-M_PI/7)), float(asin(M_PI/7))));
    float Pitch_d = asin(std::clamp((ux1 * cosf(Yaw_d) + uy1 * sinf(Yaw_d)) / cosf(Roll_d), float(asin(-M_PI/7)), float(asin(M_PI/7))));
    RPY_to_Quaternion(Roll_d, Pitch_d, Yaw_d);
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