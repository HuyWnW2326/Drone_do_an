
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
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
                std::cout << "x_d         ="  << x_d << std::endl;
                std::cout << "y_d         ="  << y_d << std::endl;
                std::cout << "z_d         ="  << z_d << std::endl;
                std::cout << "x_current   ="  << x_current << std::endl;
                std::cout << "y_current   ="  << y_current << std::endl;
                std::cout << "z_current   ="  << z_current << std::endl;
                std::cout << std::endl;
            }

            if((Rc_CH6 >= 1500) && (state_offboard == 0))
            {
                state_offboard = 1;
                x_center = x_current;
                y_center = y_current - 3;
                z_d = z_current;
				t_start = this->get_clock()->now().seconds();
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
            {
				delta_t = this->get_clock()->now().seconds() - t_start;
				x_d = x_center + 3 * sin(2 * M_PI / 15 * delta_t);
				y_d = y_center + 3 * cos(2 * M_PI / 15 * delta_t);
                Altitude_controller(z_d);	
                Controller_xy(x_d, y_d);
            }
            timer_count ++;
        };
        timer_ = this->create_wall_timer(10ms, timer_callback);
    }

    void arm();
	void disarm();
    void RPY_to_Quaternion(float Roll, float Pitch, float Yaw);
    void Altitude_controller(float DesiredValueZ);
    void Controller_xy(float DesiredValueX, float DesiredValueY);
    void xyz_setpoint(float x_desired, float y_desired, float z_desired);

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_publisher_;
    rclcpp::Publisher<VehicleThrustSetpoint>::SharedPtr vehicle_thrust_setpoint_publisher_;

    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;
    rclcpp::Subscription<InputRc>::SharedPtr input_rc_subscription_;

    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
    uint64_t timer_count = 0;
    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    // Parameter of Quadcopter
    float fz = 0.0;     
    float g = 9.8;
    float m = 1.820;
    float b = 4.6 * pow(10,-6);
    float omg_max = 1285;
    const double f_max = 4 * b * pow(omg_max,2); 

    float lon_current = 0.0;
    float lat_current = 5.0;
    float alt_current = 0.0;

    float x_d = 0.0;
    float y_d = 5.0;
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

    float Rc_CH6 = 0.0;
    uint8_t state_offboard = 0;
    float quaternion[4];

	float t_start = 0.0;
	float delta_t = 0.0;
	float x_center = 0.0;
	float y_center = 0.0;

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(float x, float y, float z);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void publish_vehicle_attitude_setpoint(float thrust_z);
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
    msg.attitude = true;
    msg.direct_actuator = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint(float x, float y, float z)
{
    TrajectorySetpoint msg{};
    msg.position = {x, y, z};
    msg.yaw = Yaw_hover;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
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

void OffboardControl::publish_vehicle_attitude_setpoint(float thrust_z)
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

void OffboardControl::RPY_to_Quaternion(float Roll, float Pitch, float Yaw)
{
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
}

void OffboardControl::Altitude_controller(float DesiredValueZ)
{
    float k_z = 3.0;
    float lamda_z = 2.0;
    static float tz_last = 0.0;
    static float z_d_last = 0.0;
    static float vz_d_last = 0.0;
    float vz_d = 0.0;
    float az_d = 0.0;
    float err_dot = 0.0;

    float delta_tz = this->get_clock()->now().seconds() - tz_last;
    float err_z = DesiredValueZ - z_current;

    if(delta_tz > 0)
    {
        vz_d = (DesiredValueZ - z_d_last) / delta_tz;
        az_d = (vz_d - vz_d_last) / delta_tz;
    }
    err_dot = vz_d - vz_current;
    fz = (-m/(cos(Roll_current) * cos(Pitch_current))) * (az_d - g + lamda_z * err_dot + k_z * std::clamp(err_dot + lamda_z * err_z, -1.0f, 1.0f));
    
    tz_last = this->get_clock()->now().seconds();
    vz_d_last = vz_d;
    z_d_last = DesiredValueZ;
    float fz_out = -fz/f_max;
    publish_vehicle_attitude_setpoint(fz_out);
}

void OffboardControl::Controller_xy(float DesiredValueX, float DesiredValueY)
{
    float Yaw_d = Yaw_hover;

    float k_x = 2.0;
    float lamda_x = 1.5;
    static float tx_last = 0.0;
    float delta_tx = this->get_clock()->now().seconds() - tx_last;

    static float x_d_last = 0.0;
    static float vx_d_last = 0.0;
    float vx_d = 0.0;
    float ax_d = 0.0;
    float err_x_dot = 0.0;
    float err_x = DesiredValueX - x_current;
    if(delta_tx > 0)
    {
        vx_d = (DesiredValueX - x_d_last) / delta_tx;
        ax_d = (vx_d - vx_d_last) / delta_tx;
    }
    err_x_dot = vx_d - vx_current;
    float ux1 = (-m/fz) * (ax_d + lamda_x * err_x_dot + k_x * std::clamp(err_x_dot + lamda_x * err_x, -1.0f, 1.0f));
    tx_last = this->get_clock()->now().seconds();
    vx_d_last = vx_d;
    x_d_last = DesiredValueX;

    float k_y = 2.5;
    float lamda_y = 1.6;
    static float ty_last = 0.0;
    float delta_ty = this->get_clock()->now().seconds() - ty_last;

    static float y_d_last = 0.0;
    static float vy_d_last = 0.0;
    float vy_d = 0.0;
    float ay_d = 0.0;
    float err_y_dot = 0.0;
    float err_y = DesiredValueY - y_current;
    if(delta_ty > 0)
    {
        vy_d = (DesiredValueY - y_d_last) / delta_ty;
        ay_d = (vy_d - vy_d_last) / delta_ty;
    }
    err_y_dot = vy_d - vy_current;
    float uy1 = (-m/fz) * (ay_d + lamda_y * err_y_dot + k_y * std::clamp(err_y_dot + lamda_y * err_y, -1.0f, 1.0f));
    ty_last = this->get_clock()->now().seconds();
    vy_d_last = vy_d;
    y_d_last = DesiredValueY;

    float Roll_d  = asin(std::clamp(ux1 * sin(Yaw_d) - uy1 * cos(Yaw_d), float(asin(-M_PI/6)), float(asin(M_PI/6))));
    float Pitch_d = asin(std::clamp((ux1 * cos(Yaw_d) + uy1 * sin(Yaw_d)) / cos(Roll_d), float(asin(-M_PI/6)), float(asin(M_PI/6))));
    RPY_to_Quaternion(Roll_d, Pitch_d, Yaw_hover);
}
void OffboardControl::xyz_setpoint(float x_desired, float y_desired, float z_desired)
{
    x_d = x_current + x_desired;
    y_d = y_current + y_desired;
    z_d = z_current + z_desired;
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