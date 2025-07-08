#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/input_rc.hpp>

#include <std_msgs/msg/bool.hpp>

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
        offb_ctl_mode_pub_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        traj_sp_pub_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        servo_pub_= this->create_publisher<std_msgs::msg::Bool>("/servo", 10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
        
        vehicle_local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
            x_current  = msg->x;
            y_current  = msg->y;
            z_current  = msg->z;
        });     

        vehicle_att_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos,
        [this](const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
            double w = msg->q[0];
            double x = msg->q[1];
            double y = msg->q[2];
            double z = msg->q[3];

            yaw_cur = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) ;
        });

        input_rc_sub_ = this->create_subscription<px4_msgs::msg::InputRc>("/fmu/out/input_rc", qos,
        [this](const px4_msgs::msg::InputRc::UniquePtr msg) {
            rc_ch6 = msg->values[5];
        });

        auto timer_callback = [this]() -> void {
            if((rc_ch6 >= 1500) && (state_offboard == 0))
            {
                state_offboard = 1;
                x_center = x_current;
                y_center = y_current - 3;
                z_d = z_current;
				t_start = this->get_clock()->now().seconds();
                // x_d = x_current;
                // y_d = y_current;
                // z_d = z_current;
                yaw_d =  yaw_cur;
                std::cout << "Offboard_mode  =" << std::endl;
            }
            else if((rc_ch6 <= 1500) && (state_offboard == 1)) 
            {
                state_offboard = 0;
                std::cout << "Position_mode  =" << std::endl;
            }
            pub_offb_ctl_mode();
            if(state_offboard == 1 && rc_ch6 >= 1500)
            {
                timer_count ++;
				delta_t = this->get_clock()->now().seconds() - t_start;
				x_d = x_center + 3 * sin(2 * M_PI / 10 * delta_t);
				y_d = y_center + 3 * cos(2 * M_PI / 10 * delta_t);
                if(timer_count >= 1200 && !FLAG_SERVO)
                {
                    FLAG_SERVO = true;
                    servo(true);
                }
                pub_traj_sp(x_d, y_d, z_d);
            }
        };
        timer_ = this->create_wall_timer(10ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offb_ctl_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_sp_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr servo_pub_;

    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_pos_sub_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_att_sub_;
    rclcpp::Subscription<InputRc>::SharedPtr input_rc_sub_;

    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
    uint64_t timer_count = 0;

    // Parameter of Quadcopter
    float x_current = 0.0;
    float y_current = 0.0;
    float z_current = 0.0;

    float x_d = 0.0;
    float y_d = 0.0;
    float z_d = 0.0;
    float yaw_cur = 0.0;
    float yaw_d = 0.0;

    float rc_ch6 = 0.0;
    uint8_t state_offboard = 0;
    bool FLAG_SERVO = false;

	float t_start = 0.0;
	float delta_t = 0.0;
	float x_center = 0.0;
	float y_center = 0.0;

    void pub_offb_ctl_mode();
    void pub_traj_sp(float x, float y, float z);
    void servo(bool data);
};

void OffboardControl::servo(bool data)
{
    std_msgs::msg::Bool msg{};
    msg.data = data;
    servo_pub_->publish(msg);
}

void OffboardControl::pub_offb_ctl_mode()
{
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.direct_actuator = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offb_ctl_mode_pub_->publish(msg);
}

void OffboardControl::pub_traj_sp(float x, float y, float z)
{
    TrajectorySetpoint msg{};
    msg.position = {x, y, z};
    msg.yaw = yaw_d;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    traj_sp_pub_->publish(msg);
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