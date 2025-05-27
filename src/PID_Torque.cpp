#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/input_rc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <math.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        vehicle_attitude_setpoint_publisher_= this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
        vehicle_thrust_setpoint_publisher_= this->create_publisher<VehicleThrustSetpoint>("/fmu/in/vehicle_thrust_setpoint", 10);
        vehicle_torque_setpoint_publisher_= this->create_publisher<VehicleTorqueSetpoint>("/fmu/in/vehicle_torque_setpoint", 10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
        
        vehicle_local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
            x_current  = msg->x;
            vx_current = msg->vx;

            y_current  = msg->y;
            vy_current = msg->vy;

            z_current  = msg->z;
            vz_current = msg->vz;
        });     

        vehicle_angular_velocity_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>("/fmu/out/vehicle_angular_velocity", qos,
        [this](const px4_msgs::msg::VehicleAngularVelocity::UniquePtr msg) {
            Roll_rate  = msg->xyz[0];
            Pitch_rate = msg->xyz[1];
            Yaw_rate   = msg->xyz[2];
        });

        vehicle_attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos,
        [this](const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
            double w = msg->q[0];
            double x = msg->q[1];
            double y = msg->q[2];
            double z = msg->q[3];

            Roll_current = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) ;
            Pitch_current = asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
            Yaw_current = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) ;
        });

        input_rc_subscription_ = this->create_subscription<px4_msgs::msg::InputRc>("/fmu/out/input_rc", qos,
        [this](const px4_msgs::msg::InputRc::UniquePtr msg) {
            Rc_CH6 = msg->values[5];
        });

        auto timer_callback = [this]() -> void {
                if(timer_count >= 20)
                {
                    timer_count = 0;
                    std::cout << "x_current               ="  << x_current << std::endl;
                    std::cout << "y_current               ="  << y_current << std::endl;
                    std::cout << "z_current               ="  << z_current << std::endl;
                    std::cout << "x_d                     ="  << x_d << std::endl;
                    std::cout << "y_d                     ="  << y_d << std::endl;
                    std::cout << "z_d                     ="  << z_d << std::endl;
                    std::cout << std::endl;
                }

                if((Rc_CH6 >= 1500) && (state_offboard == 0))
                {
                    state_offboard = 1;
                    z_d = z_current;
                    x_d = x_current;
                    y_d = y_current;
                    Yaw_hover =  Yaw_current;
                    std::cout << "Offboard_mode  =" << std::endl;
                }
                else if((Rc_CH6 <= 1500) && (state_offboard == 1)) 
                {
                    state_offboard = 0;
                    std::cout << "Position_mode  =" << std::endl;
                }
                publish_offboard_control_mode();
                if(state_offboard == 1)
                    publish_thrust_setpoint(Altitude_controller(z_d));
                timer_count ++;
            };
        
            timer_1 = this->create_wall_timer(20ms, timer_callback);
            timer_2 = this->create_wall_timer(1ms, std::bind(&OffboardControl::timer_angular_rate_callback, this));
            timer_3 = this->create_wall_timer(4ms, std::bind(&OffboardControl::timer_angle_callback, this));
        }

        void timer_angular_rate_callback()  // 1ms
        {
            std::thread angular_rate_thread([this]() {
                if(state_offboard == 1)
                {
                    publish_torque_setpoint(RollRate_controller(roll_d), PitchRate_controller(pitch_d), YawRate_controller(yaw_d));
                }
            });
            angular_rate_thread.detach();  // Detach the thread so it runs asynchronously
        }

        void timer_angle_callback()     // 4ms  
        {
            std::thread angle_thread([this]() {
                if(state_offboard == 1)
                {
                    // RollPitch = Controller_xy(x_d, y_d);
                    // roll_d = Roll_controller(RollPitch[0]);
                    // pitch_d = Pitch_controller(RollPitch[1]);
                    roll_d = Roll_controller(M_PI/6);
                    pitch_d = Pitch_controller(0.0f);
                    yaw_d = Yaw_controller(Yaw_hover);
                }
            });
            angle_thread.detach();  // Detach the thread so it runs asynchronously
        };

private:
    rclcpp::TimerBase::SharedPtr timer_1;
    rclcpp::TimerBase::SharedPtr timer_2;
    rclcpp::TimerBase::SharedPtr timer_3;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_publisher_;
    rclcpp::Publisher<VehicleThrustSetpoint>::SharedPtr vehicle_thrust_setpoint_publisher_;
    rclcpp::Publisher<VehicleTorqueSetpoint>::SharedPtr vehicle_torque_setpoint_publisher_;

    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;
    rclcpp::Subscription<InputRc>::SharedPtr input_rc_subscription_;
    rclcpp::Subscription<VehicleAngularVelocity>::SharedPtr vehicle_angular_velocity_subscription_;

    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
    uint64_t timer_count = 0;
    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    // Parameter of Quadcopter
    // float omg_max = 1285;
    float fz = 0.0f;     
    float g = 9.8066f;
    // float m = 1.820;
    // float b = 1.201e-5;
    // float d = 1.606e-7;
    // float omg_max = 985.33;
    // float Ix = 1.834e-2;
    // float Iy = 1.834e-2;
    // float Iz = 3.347e-2;
    float m = 1.545f;
    float b = 4.6e-6f;
    float d = 0.06f;
    float omg_max = 1100.0f;
    float Ix = 0.03f;
    float Iy = 0.03f;
    float Iz = 0.058f;
    const double f_max = 4 * b * pow(omg_max,2);
    const double Tx_max = 2 * d * b * pow(omg_max,2);
    const double Ty_max = 2 * d * b * pow(omg_max,2);
    const double Tz_max = 2 * d * b * pow(omg_max,2);

    float phi = 0.0f;

    float x_d = 0.0f;
    float y_d = 0.0f;
    float z_d = 0.0f;
    float Yaw_hover = 0.0f;

    float x_current = 0.0f;
    float y_current = 0.0f;
    float z_current = 0.0f;

    float vx_current = 0.0f;
    float vy_current = 0.0f;
    float vz_current = 0.0f;

    float Roll_current  = 0.0f;
    float Pitch_current = 0.0f;
    float Yaw_current   = 0.0f;

    float roll_d = 0.0f;
    float pitch_d = 0.0f;
    float yaw_d = 0.0f;

    float Roll_rate = 0.0f;
    float Pitch_rate = 0.0f;
    float Yaw_rate = 0.0f;

    float Rc_CH6 = 0.0f;
    uint8_t state_offboard = 0;

    float w_n_RollPitch = 2.1f;
    float zeta_RollPitch = 0.85f;

    float w_n_Yaw = 1.9f;
    float zeta_Yaw = 0.85f;
    std::array<float, 2> RollPitch = {0.0f, 0.0f};
    
    void publish_offboard_control_mode();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
    void publish_vehicle_attitude_setpoint(float thrust_z, std::array<float, 4> quaternion);
    std::array<float, 4> RPY_to_Quaternion(float Roll, float Pitch, float Yaw);
    float Altitude_controller(float DesiredZ);
    std::array<float, 2> Controller_xy(float DesiredX, float DesiredY);
    void xyz_setpoint(float x_desired, float y_desired, float z_desired);
    float saturation(float value, float min, float max);
    void publish_torque_setpoint(float Tx, float Ty, float Tz);
    void publish_thrust_setpoint(float Thrust);
    float Yaw_controller(float DesiredYaw);
    float Pitch_controller(float DesiredPitch);
    float Roll_controller(float DesiredRoll);
    float RollRate_controller(float DesiredRollRate);
    float PitchRate_controller(float DesiredPitchRate);
    float YawRate_controller(float DesiredYawRate);
};

void OffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = true;
    msg.direct_actuator = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_torque_setpoint(float Tx, float Ty, float Tz)
{
    VehicleTorqueSetpoint msg{};
    msg.xyz = {Tx, Ty, Tz};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_torque_setpoint_publisher_->publish(msg);
}

void OffboardControl::publish_thrust_setpoint(float Thrust)
{
    VehicleThrustSetpoint msg{};
    msg.xyz = {0.0, 0.0, Thrust};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_thrust_setpoint_publisher_->publish(msg);
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

float OffboardControl::RollRate_controller(float DesiredRollRate)
{
    float kp1 = 2 * w_n_RollPitch * zeta_RollPitch;
    float u_roll_rate = kp1 * (DesiredRollRate - Roll_rate);
    float Tx = (u_roll_rate * Ix - (Iy - Iz) * Pitch_rate * Yaw_rate);
    return Tx / Tx_max;
}

float OffboardControl::PitchRate_controller(float DesiredPitchRate)
{
    float kp1 = 2 * w_n_RollPitch * zeta_RollPitch;
    float u_pitch_rate = kp1 * (DesiredPitchRate - Pitch_rate);
    float Ty = (u_pitch_rate * Iy - (Iz - Ix) * Roll_rate * Yaw_rate);
    return Ty / Ty_max;
}

float OffboardControl::YawRate_controller(float DesiredYawRate)
{
    float kp1 = 2 * w_n_Yaw * zeta_Yaw;
    float u_yaw_rate = kp1 * (DesiredYawRate - Yaw_rate);
    float Tz = (u_yaw_rate * Iz - (Ix - Iy) * Roll_rate * Pitch_rate);
    return Tz / Tz_max;
}

float OffboardControl::Roll_controller(float DesiredRoll)
{
    float kp2 = w_n_RollPitch / (2 * zeta_RollPitch);
    return (DesiredRoll - Roll_current) * kp2;
}

float OffboardControl::Pitch_controller(float DesiredPitch)
{
    float kp2 = w_n_RollPitch / (2 * zeta_RollPitch);
    return (DesiredPitch - Pitch_current) * kp2;
}

float OffboardControl::Yaw_controller(float DesiredYaw)
{
    float kp2 = w_n_Yaw / (2 * zeta_Yaw);
    return (DesiredYaw - Yaw_current) * kp2;
}

float OffboardControl::Altitude_controller(float DesiredZ)
{
    float w_n = 3.0f;
    float zeta = 0.85f;
    float kp1 = 2 * w_n * zeta;
    float kp2 = w_n / (2 * zeta);

    float vel_z_d = saturation((DesiredZ - z_current) * kp2, -2.0f, 2.0f);
    float uz = kp1 * (vel_z_d - vz_current);
    fz = ((g - uz) * m) / (cos(Roll_current) * cos(Pitch_current));
    float fz_out = -fz/f_max;
    return fz_out;
}

std::array<float, 2> OffboardControl::Controller_xy(float DesiredX, float DesiredY)
{
    float w_n = 2.0;
    float zeta = 0.8;
    float kp1 = 2 * w_n * zeta;
    float kp2 = w_n / (2 * zeta);
    float Yaw_d = M_PI/2;

    float vel_x_d = saturation((DesiredX - x_current) * kp2, -2.0f, 2.0f);
    float vel_y_d = saturation((DesiredY - y_current) * kp2, -2.0f, 2.0f);
    float ux1 = kp1 * (vel_x_d - vx_current);
    float uy1 = kp1 * (vel_y_d - vy_current);

    float ux = -(ux1 * m) / fz;
    float uy = -(uy1 * m) / fz;

    float Roll_d  = asin(std::clamp(ux * sin(Yaw_d) - uy * cos(Yaw_d), float(asin(-M_PI/6)), float(asin(M_PI/6))));
    float Pitch_d = asin(std::clamp((ux * cos(Yaw_d) + uy * sin(Yaw_d)) / cos(Roll_d), float(asin(-M_PI/6)), float(asin(M_PI/6))));
    return {Roll_d, Pitch_d};
}

void OffboardControl::xyz_setpoint(float x_desired, float y_desired, float z_desired)
{
    x_d = x_current + x_desired;
    y_d = y_current + y_desired;
    z_d = z_current + z_desired;
}
    
float OffboardControl::saturation(float value, float min, float max)
{
    if(value > max)
        value = max;
    else if(value < min)
        value = min;
    return value;
}

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    // Create and spin the executor
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<OffboardControl>();
    executor.add_node(node);
    executor.spin();  // Run the executor with multiple threads

    rclcpp::shutdown();
    return 0;
}