#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
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
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
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
                if(timer_count >= 50)
                {
                    timer_count = 0;
                    std::cout << "Yaw_current               ="  << Yaw_current << std::endl;
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
                    Altitude_controller(z_d);
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
                    publish_torque_setpoint(0.0, 0.0, 0.0);
                    // std::cout << "angular_rate_thread   " << std::endl;
                }
            });
            angular_rate_thread.detach();  // Detach the thread so it runs asynchronously
        }

        void timer_angle_callback()     // 4ms  
        {
            std::thread angle_thread([this]() {
                if(state_offboard == 1)
                {
                    Roll_d = Roll_controller(0.0);
                    Pitch_d = Pitch_controller(0.0);
                    Yaw_d = Yaw_controller(Yaw_hover);
                    // std::cout << "angle_thread  " << std::endl;
                }
            });
            angle_thread.detach();  // Detach the thread so it runs asynchronously
        };

private:
    rclcpp::TimerBase::SharedPtr timer_1;
    rclcpp::TimerBase::SharedPtr timer_2;
    rclcpp::TimerBase::SharedPtr timer_3;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
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
    float fz = 0.0;     
    float g = 9.8;
    // float m = 1.820;
    // float b = 1.201e-5;
    // float d = 1.606e-7;
    // float omg_max = 985.33;
    // float Ix = 1.834e-2;
    // float Iy = 1.834e-2;
    // float Iz = 3.347e-2;
    float m = 1.545;
    float b = 5.84e-6;
    float d = 0.06;
    float omg_max = 1100; 
    float Ix = 0.029125;
    float Iy = 0.029125;
    float Iz = 0.055225;
    const double f_max = 4 * b * pow(omg_max,2);
    const double Tx_max = 2 * d * b * pow(omg_max,2);
    const double Ty_max = 2 * d * b * pow(omg_max,2);
    const double Tz_max = 2 * d * b * pow(omg_max,2);

    float phi = 0.0;

    float x_d = 0.0;
    float y_d = 0.0;
    float z_d = 0.0;
    float Yaw_hover = 0.0;

    float x_current = 0.0;
    float y_current = 0.0;
    float z_current = 0.0;

    float vx_current = 0.0;
    float vy_current = 0.0;
    float vz_current = 0.0;

    float Roll_current  = 0.0;
    float Pitch_current = 0.0;
    float Yaw_current   = 0.0;

    float Roll_d = 0.0;
    float Pitch_d = 0.0;
    float Yaw_d = 0.0;

    float Roll_rate = 0.0;
    float Pitch_rate = 0.0;
    float Yaw_rate = 0.0;

    float Rc_CH6 = 0.0;
    uint8_t state_offboard = 0;

    void arm();
	void disarm();
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
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

void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

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

void OffboardControl::publish_trajectory_setpoint()
{
    TrajectorySetpoint msg{};
    msg.position = {0.0, 0.0, -1.5};
    msg.yaw = 0.0; // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
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
    float w_n = 2.0;
    float zeta = 0.8;
    float kp1 = 2 * w_n * zeta;

    float u_roll_rate = kp1 * (DesiredRollRate - Roll_rate);
    float Tx = (u_roll_rate * Ix - (Iy - Iz) * Pitch_rate * Yaw_rate);
    float Tx_out = Tx / Tx_max;
    return Tx_out;
}

float OffboardControl::PitchRate_controller(float DesiredPitchRate)
{
    float w_n = 2.0;
    float zeta = 0.8;
    float kp1 = 2 * w_n * zeta;

    float u_pitch_rate = kp1 * (DesiredPitchRate - Pitch_rate);
    float Ty = (u_pitch_rate * Iy - (Iz - Ix) * Roll_rate * Yaw_rate);
    float Ty_out = Ty / Ty_max;
    return Ty_out;
}

float OffboardControl::YawRate_controller(float DesiredYawRate)
{
    float w_n = 2.0;
    float zeta = 0.8;
    float kp1 = 2 * w_n * zeta;

    float u_yaw_rate = kp1 * (DesiredYawRate - Yaw_rate);
    float Tz = (u_yaw_rate * Iz - (Ix - Iy) * Roll_rate * Pitch_rate);
    float Tz_out = Tz / Tz_max;
    return Tz_out;
}

float OffboardControl::Roll_controller(float DesiredRoll)
{
    float w_n = 2.0;
    float zeta = 0.8;
    float kp2 = w_n / (2 * zeta);

    float RollRate_d = (DesiredRoll - Roll_current) * kp2;
    return RollRate_d;
}

float OffboardControl::Pitch_controller(float DesiredPitch)
{
    float w_n = 2.0;
    float zeta = 0.8;
    float kp2 = w_n / (2 * zeta);

    float PitchRate_d = (DesiredPitch - Pitch_current) * kp2;
    return PitchRate_d;
}

float OffboardControl::Yaw_controller(float DesiredYaw)
{
    float w_n = 2.0;
    float zeta = 0.8;
    float kp2 = w_n / (2 * zeta);

    float YawRate_d = (DesiredYaw - Yaw_current) * kp2;
    return YawRate_d;
}

float OffboardControl::Altitude_controller(float DesiredZ)
{
    float w_n = 2.0;
    float zeta = 0.8;
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
    float w_n = 1.0;
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
    return {Roll_controller(Roll_d), Pitch_controller(Pitch_d)};
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