#include <memory>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class TrajectoryRecorder : public rclcpp::Node {
public:
    TrajectoryRecorder(std::ofstream& odom_out, std::ofstream& imu_out)
        : Node("trajectory_recorder"), odom_out_(odom_out), imu_out_(imu_out)

    {

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rosbot_base_controller/odom", 10,
            std::bind(&TrajectoryRecorder::odom_callback, this, _1));

        global_x_ = 0;
        global_y_ = 0;
        global_z_ = 0;
        last_time_ = 0;
        first_ = true;
        first_odom_ = true;

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_broadcaster/imu", 10,
            std::bind(&TrajectoryRecorder::imu_callback, this, _1));

        odom_filtered_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered_gr4", 10,
            std::bind(&TrajectoryRecorder::odom_callback, this, _1));


        imu_x_ = 0;
        imu_z_ = 0;
        imu_theta_ = 0;
        first_imu_ = true;

    }

private:

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_filtered_subscription_;

    std::ofstream& odom_out_;
    std::ofstream& imu_out_;
    std::ofstream odom_filtered_out_;

    double imu_x_;
    double imu_z_;
    double imu_theta_;
    rclcpp::Time last_imu_time_;
    bool first_imu_;

    void imu_callback(const sensor_msgs::msg::Imu& msg) {
        rclcpp::Time current_time = msg.header.stamp;

        if (first_imu_) {
            last_imu_time_ = current_time;
            first_imu_ = false;

            std::cout << "[imu] x " << imu_x_ << ", y "<< imu_z_ << std::endl;
            imu_out_ << imu_x_ << " " << imu_z_ << "\n";
            imu_out_.flush();
        }

        double delta_t = (current_time - last_imu_time_).seconds();


        double ax = msg.linear_acceleration.x;
        double ay = msg.linear_acceleration.y;
        double az = msg.linear_acceleration.z;


        double vx = ax * delta_t;
        double vy = ay * delta_t;
        double vz = az * delta_t;


        double v = std::sqrt(vx * vx + vy * vy + vz * vz);

        double distance = v * delta_t;


        double delta_theta = msg.angular_velocity.z;

        imu_x_ += distance * std::sin(imu_theta_ + delta_theta / 2 * delta_t);
        imu_z_ += distance * std::cos(imu_theta_ + delta_theta / 2 * delta_t);

        imu_theta_ += delta_theta * delta_t;

        std::cout << "[imu] x " << imu_x_ << ", y "<< imu_z_ << std::endl;
        imu_out_ << imu_x_ << " " << imu_z_ << "\n";
        imu_out_.flush();

        last_imu_time_ = current_time;
    }


    void odom_callback(const nav_msgs::msg::Odometry& msg) {
        geometry_msgs::msg::PoseWithCovariance pose = msg.pose;
        double x = pose.pose.position.x;
        double y = pose.pose.position.y;
        double z = pose.pose.position.z;

        if (first_odom_) {
            first_odom_ = false;
            odom_offset_x_ = pose.pose.position.x;
            odom_offset_y_ = pose.pose.position.y;
            odom_offset_z_ = pose.pose.position.z;
        }
        else {
            x = x - odom_offset_x_;
            y = y - odom_offset_y_;
            z = z - odom_offset_z_;
        }

        if (odom_out_.good()) {
            std::cout << "[Odom] x " << x << ", y "<< y << std::endl;
            odom_out_ << x << " " << y << " " << z << std::endl;
        }
    }

    void odom_filtered_callback(const nav_msgs::msg::Odometry& msg) {
        geometry_msgs::msg::PoseWithCovariance pose = msg.pose;
        double x = pose.pose.position.x;
        double y = pose.pose.position.y;
        double z = pose.pose.position.z;

        if (first_odom_) {
            first_odom_ = false;
            odom_offset_x_ = pose.pose.position.x;
            odom_offset_y_ = pose.pose.position.y;
            odom_offset_z_ = pose.pose.position.z;
        }
        else {
            
            x = x - odom_offset_x_;
            y = y - odom_offset_y_;
            z = z - odom_offset_z_;
        }

        if (odom_filtered_out_.good()) {
            std::cout << "[Odom Filtered] x " << x << ", y "<< y << std::endl;
            odom_filtered_out_ << x << " " << y << " " << z << std::endl;
        }
    }

    double global_x_;
    double global_y_;
    double global_z_;

    double odom_offset_x_ = 0.0;
    double odom_offset_y_ = 0.0;
    double odom_offset_z_ = 0.0;

    double last_vel_x_ = 0.0;
    double last_vel_y_ = 0.0;
    double last_vel_z_ = 0.0;

    bool first_;
    bool first_odom_;

    double last_time_ = 0;
};


int main(int argc, char** argv) {
    std::ofstream odom_out("odom.txt");
    std::ofstream imu_out("imu.txt");


    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryRecorder>(odom_out, imu_out);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}