#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/input_rc.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <math.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace geometry_msgs::msg;

enum state
{
    MANUAL = 0,
    POSITION = 2,
    OFFBOARD = 14,
    TAKEOFF = 17,
    LANDING = 18,
    POINT1,
    POINT2,
    POINT3,
    RETURN,
    READY,
    ARMED
};

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        offb_ctl_mode_pub_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        vehicle_cmd_pub_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        vehicle_att_sp_pub_= this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
        UAV_pos_pub_= this->create_publisher<geometry_msgs::msg::Point>("/uav_position", 10);
        UAV_state_pub_= this->create_publisher<std_msgs::msg::UInt32>("/uav_state", 10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
        
        vehicle_local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
            pos_cur.x = msg->x;
            pos_cur.y = msg->y;
            pos_cur.z = msg->z;

            vel_cur.x = msg->vx;
            vel_cur.y = msg->vy;
            vel_cur.z = msg->vz;
        });     

        vehicle_global_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>("/fmu/out/vehicle_global_position", qos,
        [this](const px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg) {
            lon_cur = msg->lon;
            lat_cur = msg->lat;
            alt_cur = msg->alt;
        });

        vehicle_att_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos,
        [this](const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
            float w = msg->q[0];
            float x = msg->q[1];
            float y = msg->q[2];
            float z = msg->q[3];

            roll_cur = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) ;
            pitch_cur = asinf(std::clamp(2.0f * (w * y - z * x), -1.0f, 1.0f));
            yaw_cur = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) ;
        });

        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", qos,
        [this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
            nav_state = msg->nav_state;
            arming_state = msg->arming_state == 2;
        });

        input_rc_sub_ = this->create_subscription<px4_msgs::msg::InputRc>("/fmu/out/input_rc", qos,
        [this](const px4_msgs::msg::InputRc::UniquePtr msg) {
            rc_ch6 = msg->values[5];
        });

        point1_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/point1", qos,
        [this](const geometry_msgs::msg::Point::UniquePtr msg) {
            lat_target[0] = msg->x;
            lon_target[0] = msg->y;
        });

        point2_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/point2", qos,
        [this](const geometry_msgs::msg::Point::UniquePtr msg) {
            lat_target[1] = msg->x;
            lon_target[1] = msg->y;
        });

        point3_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/point3", qos,
        [this](const geometry_msgs::msg::Point::UniquePtr msg) {
            lat_target[2] = msg->x;
            lon_target[2] = msg->y;
            is_takeoff = msg->z;
        });

        failsafeFlag_sub_ = this->create_subscription<px4_msgs::msg::FailsafeFlags>("/fmu/out/failsafe_flags", qos,
        [this](const px4_msgs::msg::FailsafeFlags::UniquePtr msg) {
            if(state_drone == 1 && !msg->global_position_invalid)
            {
                state_drone = 2;
                UAV_state_pub(state_drone);       // Send state 2 of UAV to ready for takeoff
            }
        });

        auto timer_callback = [this]() -> void {
            timer_count++;          // Timer for controlling UAV behavior
            timer_pub_pos++;        // Timer for publishing UAV position      
            if(timer_pub_pos >= 500)
            {
                timer_pub_pos = 0;
                UAV_pos_pub(lat_cur, lon_cur);       // Publish UAV current position
            }
            if(rc_ch6 > 1500 && is_takeoff && state_drone != 1)
            {
                pub_offb_ctl_mode();
                switch (STEP)
                {
                case READY:
                    if(nav_state != MANUAL)
                        this->manual();
                    else 
                        STEP = MANUAL;
                    break;
                case MANUAL:
                    if(!arming_state)
                        this->arm();
                    else 
                        STEP = ARMED;
                    break;
                case ARMED:
                    if(nav_state != OFFBOARD)
                        this->offboard();
                    else 
                        STEP = OFFBOARD;
                    break;
                case OFFBOARD:
                    switch(STEP_OFFBOARD)
                    {
                    case OFFBOARD:
                        // Set xyz desired postion for takeoff
                        STEP_OFFBOARD = TAKEOFF;
                        pos_d.x = pos_cur.x;
                        pos_d.y = pos_cur.y;
                        pos_d.z = pos_cur.z - 7.0f;
                        yaw_d = yaw_cur;
                        timer_count = 0;
                        break;
                    case TAKEOFF:
                        // If z position of UAV is less than 0.5 from the desired position or 10s time out
                        if((pos_cur.z < pos_d.z + 0.5)||(timer_count >= 500))
                        {
                            STEP_OFFBOARD = POINT1;
                            pos_global_2_local(lat_target[0], lon_target[0]);
                            // xyz_setpoint(-5.0f, 0.0f, 0.0f);
                            timer_count = 0;
                        }
                        break;
                    case POINT1:
                        if(((pow(pos_cur.x - pos_d.x, 2) + pow(pos_cur.y - pos_d.y, 2) < 0.25f) && !FLAG_POINT1) || (timer_count >= 1500))
                        {
                            FLAG_POINT1 = true;
                            state_drone = 10;
                            UAV_state_pub(state_drone);       // Send state 10 of UAV to indicate that it reached point 1
                            timer_count = 0;
                        }
                        else if(FLAG_POINT1 && (timer_count >= 750))
                        {
                            STEP_OFFBOARD = POINT2;
                            pos_global_2_local(lat_target[1], lon_target[1]);
                            // xyz_setpoint(0.0f, 3.0f, 0.0f);
                            timer_count = 0;
                            UAV_state_pub(11);
                        }
                        break;
                    case POINT2:
                        if(((pow(pos_cur.x - pos_d.x, 2) + pow(pos_cur.y - pos_d.y, 2) < 0.25f) && !FLAG_POINT2) || (timer_count >= 1500))
                        {
                            FLAG_POINT2 = true;
                            state_drone = 20;
                            UAV_state_pub(state_drone);       // Send state 20 of UAV to indicate that it reached point 2
                            timer_count = 0;
                        }
                        else if(FLAG_POINT2 && (timer_count >= 750))
                        {
                            STEP_OFFBOARD = POINT3;
                            pos_global_2_local(lat_target[2], lon_target[2]);
                            // xyz_setpoint(-5.0f, 2.0f, 0.0f);
                            timer_count = 0;
                            UAV_state_pub(11);
                        }
                        break;
                    case POINT3:
                        if(((pow(pos_cur.x - pos_d.x, 2) + pow(pos_cur.y - pos_d.y, 2) < 0.25f) && !FLAG_POINT3) || (timer_count >= 1500))
                        {
                            FLAG_POINT3 = true;
                            state_drone = 30;
                            UAV_state_pub(state_drone);       // Send state 30 of UAV to indicate that it reached point 3
                            timer_count = 0;
                        }
                        else if(FLAG_POINT3 && (timer_count >= 750))
                        {
                            STEP_OFFBOARD = RETURN;
                            xyz_setpoint(-pos_cur.x, -pos_cur.y, 0.0f);
                            timer_count = 0;
                            UAV_state_pub(11);
                        }
                        break;
                    case RETURN:
                        if(((pow(pos_cur.x - pos_d.x, 2) + pow(pos_cur.y - pos_d.y, 2) < 0.25f) && !FLAG_RETURN) || (timer_count >= 2000))
                        {
                            FLAG_RETURN = true;
                            state_drone = 40;
                            UAV_state_pub(state_drone);       // Send state 40 of UAV to indicate that it reached return point
                            timer_count = 0;
                        }
                        else if(FLAG_RETURN && (timer_count >= 150))
                        {
                            STEP = LANDING;
                            timer_count = 0;
                        }
                        break;
                    }
                    xyz_controller(pos_d.x, pos_d.y, pos_d.z);
                    break;
                case LANDING:
                    this->landing();
                    break;
                }
            }
            else 
            {
                this->position();
                FLAG_POINT1 = false;
                FLAG_POINT2 = false;
                FLAG_POINT3 = false;
                FLAG_RETURN = false;
                STEP = READY;
                STEP_OFFBOARD = OFFBOARD;
            }
        };
        timer_ = this->create_wall_timer(20ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offb_ctl_mode_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_cmd_pub_;
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr vehicle_att_sp_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr UAV_pos_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr UAV_state_pub_;

    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_pos_sub_;
    rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr vehicle_global_pos_sub_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_att_sub_;
    rclcpp::Subscription<InputRc>::SharedPtr input_rc_sub_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point1_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point2_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point3_sub_;
    rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr failsafeFlag_sub_;

    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
    std::atomic<uint64_t> t_start;
    std::atomic<uint64_t> t_end;

    uint64_t timer_count = 0;
    uint64_t timer_pub_pos = 0;   //!< counter for the number of setpoints sent

    // Parameter of Quadcopter
    float fz = 0.0f;     
    float g = 9.8f;
    float m = 1.820f;
    float b = 4.6 * pow(10,-6);
    float omg_max = 1285.0f;
    // float m = 1.545f;
    // float b = 4.6f * pow(10,-6);
    // float omg_max = 1100.0f;
    const double f_max = 4.0f * b * pow(omg_max,2); 

    float lon_cur = 106.770942f;
    float lat_cur = 10.850548f;
    float alt_cur = 0.0f;

    float lon_target[3];
    float lat_target[3];

    float delta_lon = 0.0f;
    float delta_lat = 0.0f;
    float phi = 0.0f;

    Vector3 pos_d;
    Vector3 pos_cur;
    Vector3 vel_d;
    Vector3 vel_cur;

    float roll_cur  = 0.0f;
    float pitch_cur = 0.0f;
    float yaw_cur   = 0.0f;
    float yaw_d = 0.0f;

    float rc_ch6 = 0.0f;    
    uint8_t state_drone = 1;

    uint8_t nav_state = 0;
    uint8_t STEP = READY;
    uint8_t STEP_OFFBOARD = OFFBOARD;

    bool arming_state = 0;
    bool FLAG_POINT1 = false;
    bool FLAG_POINT2 = false;
    bool FLAG_POINT3 = false;
    bool FLAG_RETURN = false;
    bool is_takeoff = false;

    void arm();
    void disarm();
    void offboard();
    void manual();
    void position();
    void landing();

    void pub_offb_ctl_mode();
    void pub_vehicle_cmd(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
    void pub_vehicle_att_sp(float thrust, std::array<float, 4> quaternion);
    void UAV_pos_pub(float lat, float lon);
    void UAV_state_pub(uint32_t state_cur);

    std::array<float, 4> rpy_to_quaternion(float roll, float pitch, float yaw);
    float alt_controller(float desired_z);
    void xyz_controller(float desired_x, float desired_y, float desired_z);
    void xyz_setpoint(float x_desired, float y_desired, float z_desired);
    std::array<float ,2> constrain_xy(float vel_x, float vel_y, float vel_max);
    void pos_global_2_local(float lat_target, float lon_target);
    float saturation(float value, float min, float max);
};

void OffboardControl::landing()
{
    VehicleCommand msg{};
    msg.command = VehicleCommand::VEHICLE_CMD_NAV_LAND;  // MAV_CMD_NAV_LAND
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_cmd_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Landing command send");
}

void OffboardControl::arm()
{
    pub_vehicle_cmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void OffboardControl::disarm()
{
    pub_vehicle_cmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void OffboardControl::offboard()
{
    pub_vehicle_cmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    RCLCPP_INFO(this->get_logger(), "Offboard command send");
}

void OffboardControl::position()
{
    pub_vehicle_cmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 3.0f);
    // RCLCPP_INFO(this->get_logger(), "Position command send");
}

void OffboardControl::manual()
{
    pub_vehicle_cmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 1.0f);
    RCLCPP_INFO(this->get_logger(), "Manual command send");
}

void OffboardControl::pos_global_2_local(float lat_target, float lon_target)
{
    phi = (lat_target + lat_cur) / 2 * (M_PI /180);
    delta_lat = (lat_target - lat_cur);   // Don vi do
    delta_lon = (lon_target - lon_cur);   // Don vi do
    pos_d.x = pos_cur.x + delta_lat * 111320.0f;   // 1 do = 111320.0 m
    pos_d.y = pos_cur.y + delta_lon * 111320.0f * cosf(phi); // 1 do kinh do = 111320.0 * cosd(trung binh)
}

void OffboardControl::UAV_pos_pub(float lat, float lon)
{
    geometry_msgs::msg::Point msg{};
    msg.x = lat;
    msg.y = lon;
    UAV_pos_pub_->publish(msg);
}

void OffboardControl::UAV_state_pub(uint32_t state_cur)
{
    std_msgs::msg::UInt32 msg{};
    msg.data = state_cur;
    UAV_state_pub_->publish(msg);
}

void OffboardControl::pub_offb_ctl_mode()
{
    OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = true;
    msg.direct_actuator = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offb_ctl_mode_pub_->publish(msg);
}

void OffboardControl::pub_vehicle_cmd(uint16_t command, float param1, float param2)
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
    vehicle_cmd_pub_->publish(msg);
}

void OffboardControl::pub_vehicle_att_sp(float thrust, std::array<float, 4> quaternion)
{
    VehicleAttitudeSetpoint msg{};
    msg.thrust_body[0] = 0.0f;
    msg.thrust_body[1] = 0.0f;
    msg.thrust_body[2] = thrust;
    msg.q_d[0] = quaternion[0];
    msg.q_d[1] = quaternion[1];
    msg.q_d[2] = quaternion[2];
    msg.q_d[3] = quaternion[3];
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_att_sp_pub_->publish(msg);
}

std::array<float, 4> OffboardControl::rpy_to_quaternion(float roll, float pitch, float yaw)
{
    std::array<float, 4> quaternion;
    float cy = cosf(yaw/2);
    float sy = sinf(yaw/2);
    float cp = cosf(pitch/2);
    float sp = sinf(pitch/2);
    float cr = cosf(roll/2);
    float sr = sinf(roll/2);

    quaternion[0] = cr * cp * cy + sr * sp * sy;
    quaternion[1] = sr * cp * cy - cr * sp * sy;
    quaternion[2] = cr * sp * cy + sr * cp * sy;
    quaternion[3] = cr * cp * sy - sr * sp * cy;
    return quaternion;
}

float OffboardControl::alt_controller(float desired_z)
{
    float k_z = 2.5f;
    float lamda_z = 2.0f;
    float c_z = 0.07f;
    static float tz_last = 0.0f;
    static float z_d_last = 0.0f;
    static float vz_d_last = 0.0f;
    static float integral_z = 0.0f;
    float vz_d = 0.0f;
    float az_d = 0.0f;
    float err_dot = 0.0f;

    float delta_tz = this->get_clock()->now().seconds() - tz_last;
    float err_z = desired_z - pos_cur.z;

    if(delta_tz > 0)
    {
        vz_d = (desired_z - z_d_last) / delta_tz;
        az_d = (vz_d - vz_d_last) / delta_tz;
    } 
    err_dot = vz_d - vel_cur.z;
    integral_z = std::clamp(integral_z + err_z * delta_tz, -0.2f, 0.2f); // Limit the integral term to prevent windup
    float s_z = err_dot + lamda_z * err_z + c_z * integral_z;
    fz = (-m/(cosf(roll_cur) * cosf(pitch_cur))) * (az_d - g + lamda_z * err_dot + c_z * err_z + k_z * std::clamp(s_z, -1.0f, 1.0f));

    tz_last = this->get_clock()->now().seconds();
    vz_d_last = vz_d;
    z_d_last = desired_z;
    float fz_out = -fz/f_max;
    if(STEP_OFFBOARD == TAKEOFF)
        fz_out = saturation(-fz/f_max, -0.7f, 0.7f);
    return fz_out;
}

void OffboardControl::xyz_controller(float desired_x, float desired_y, float desired_z)
{
    float k_x = 2.5f;
    float lamda_x = 1.3f;
    float c_x = 0.07f;
    static float tx_last = 0.0f;
    float delta_tx = this->get_clock()->now().seconds() - tx_last;

    static float x_d_last = 0.0f;
    static float vx_d_last = 0.0f;
    static float integral_x = 0.0f;
    float vx_d = 0.0f;
    float ax_d = 0.0f;
    float err_x_dot = 0.0f;
    float err_x = desired_x - pos_cur.x;
    if(delta_tx > 0)
    {
        vx_d = (desired_x - x_d_last) / delta_tx;
        ax_d = (vx_d - vx_d_last) / delta_tx;
    }
    err_x_dot = vx_d - vel_cur.x;
    integral_x = std::clamp(integral_x + err_x * delta_tx, -1.0f, 1.0f);
    float s_x = err_x_dot + lamda_x * err_x + c_x * integral_x;
    float ux1 = (-m/fz) * (ax_d + lamda_x * err_x_dot + c_x * err_x + k_x * std::clamp(s_x, -1.0f, 1.0f));
    tx_last = this->get_clock()->now().seconds();
    vx_d_last = vx_d;
    x_d_last = desired_x;

    float k_y = 2.5f;
    float lamda_y = 1.3f;
    float c_y = 0.07f;
    static float ty_last = 0.0f;
    float delta_ty = this->get_clock()->now().seconds() - ty_last;

    static float y_d_last = 0.0f;
    static float vy_d_last = 0.0f;
    static float integral_y = 0.0f;
    float vy_d = 0.0f;
    float ay_d = 0.0f;
    float err_y_dot = 0.0f;
    float err_y = desired_y - pos_cur.y;
    if(delta_ty > 0)
    {
        vy_d = (desired_y - y_d_last) / delta_ty;
        ay_d = (vy_d - vy_d_last) / delta_ty;
    }
    err_y_dot = vy_d - vel_cur.y;
    integral_y = std::clamp(integral_y + err_y * delta_ty, -1.0f, 1.0f);
    float s_y = err_y_dot + lamda_y * err_y + c_y * integral_y;
    float uy1 = (-m/fz) * (ay_d + lamda_y * err_y_dot + c_y * err_y + k_y * std::clamp(s_y, -1.0f, 1.0f));
    ty_last = this->get_clock()->now().seconds();
    vy_d_last = vy_d;
    y_d_last = desired_y;

    float roll_d  = asinf(std::clamp(ux1 * sinf(yaw_d) - uy1 * cosf(yaw_d), float(asinf(-M_PI/7)), float(asinf(M_PI/7))));
    float pitch_d = asinf(std::clamp((ux1 * cosf(yaw_d) + uy1 * sinf(yaw_d)) / cosf(roll_d), float(asinf(-M_PI/7)), float(asinf(M_PI/7))));
    pub_vehicle_att_sp(alt_controller(desired_z), rpy_to_quaternion(roll_d, pitch_d, yaw_d));
}

void OffboardControl::xyz_setpoint(float x_desired, float y_desired, float z_desired)
{
    pos_d.x = pos_cur.x + x_desired;
    pos_d.y = pos_cur.y + y_desired;
    pos_d.z = pos_cur.z + z_desired;
}

std::array<float ,2> OffboardControl::constrain_xy(float vel_x, float vel_y, float vel_max)
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