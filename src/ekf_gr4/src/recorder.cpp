#include <memory>
#include <fstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;
using std::sqrt;
using std::pow;
using std::sin;
using std::cos;

class TrajectoryRecorder : public rclcpp::Node
{
public:
    TrajectoryRecorder(std::ofstream& odom_out, std::ofstream& imu_out, std::ofstream& imu_debug_out, std::ofstream& odom_filter_out) : 
        Node("trajectory_recorder"), odom_out_(odom_out), imu_out_(imu_out), imu_debug_out_(imu_debug_out), odom_filter_out_(odom_filter_out)
    {
        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "rosbot_base_controller/odom", 10,
            [&](const nav_msgs::msg::Odometry& msg){

                isMoving = x_odom != msg.pose.pose.position.x || y_odom != msg.pose.pose.position.y;

                
                if (first_odom_) {

                    first_odom_ = false;
                    odom_offset_x_ = msg.pose.pose.position.x;
                    odom_offset_y_ = msg.pose.pose.position.y;
                
                } else if (isMoving) {
                    
                    // to make sure that the position is in the scope and the original point is 0,0
                    double x = msg.pose.pose.position.x - odom_offset_x_;
                    double y = msg.pose.pose.position.y - odom_offset_y_;
                    
                    std::cout << "[Odom] x " << x << ", y "<< y << std::endl;
                    odom_out_ << x << " "<< y << std::endl;
                    odom_out_.flush();

                    isMoving = true;
                    
                } else {
                    
                    isMoving = false;
                }
                
                x_odom = msg.pose.pose.position.x;
                y_odom = msg.pose.pose.position.y;               
            }
        );

        subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_broadcaster/imu", 10,
            [&](const sensor_msgs::msg::Imu& msg){

             	double t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
                double ax = msg.linear_acceleration.x;
                double ay = msg.linear_acceleration.y;
                double az = msg.linear_acceleration.z;	
                double omega = msg.angular_velocity.z;
                double xn, yn, rotateAngular;

                if (first_imu_) {

                    first_imu_ = false;

                    berechnung_imu(t_now, ax, ay, az, xn, yn, rotateAngular, omega);

                    imu_offset_x_ = xn;
                    imu_offset_y_ = yn;
                
                } else if(isMoving) {

                    log_imu_debug(msg);

                    berechnung_imu(t_now, ax, ay, az, xn, yn, rotateAngular, omega);

                    xn = xn - imu_offset_x_;
                    yn = yn - imu_offset_y_;

					std::cout <<"[IMU] x " << xn << ", y " << yn << std::endl;
					imu_out << xn << " " << yn << std::endl;
	                imu_out_.flush();
	
					x_vor = xn;
					y_vor = yn;
					rotateAngular_vor = rotateAngular;
				}

                t_vor = t_now;
            }
        );

        subscription_odom_filter_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered_gr4", 10,
            [&](const nav_msgs::msg::Odometry& msg){
                if (isMoving) {
                    
                    // to make sure that the position is in the scope and the original point is 0,0
                    double x = msg.pose.pose.position.x - odom_offset_x_;
                    double y = msg.pose.pose.position.y - odom_offset_y_;
                    
                    std::cout << "[Odom Filtered] x " << x << ", y "<< y << std::endl;
                    odom_filter_out << x << " "<< y << std::endl;
                    odom_filter_out.flush();
                }
            }
        );
    }

    
    private:
    
    void berechnung_imu(double t_now, double ax, double ay, double az, double &xn, double &yn, double &rotateAngular, double omega)
    {
        double delta_t = t_now - t_vor;
        double v = sqrt(pow(ax * delta_t, 2.0) + pow(ay * delta_t, 2.0) + pow(az * delta_t, 2.0));
        xn = x_vor + (v * delta_t * sin(rotateAngular_vor));
        yn = y_vor + (v * delta_t * cos(rotateAngular_vor));
        rotateAngular = rotateAngular_vor + (omega * pow(delta_t, 2.0));
    }

    void log_imu_debug(const sensor_msgs::msg::Imu& msg)
    {
        imu_debug_out_ << "[IMU DEBUG] Orientation: ("
               << msg.orientation.x << ", "
               << msg.orientation.y << ", "
               << msg.orientation.z << ", "
               << msg.orientation.w << "), "
               << "Angular Velocity: ("
               << msg.angular_velocity.x << ", "
               << msg.angular_velocity.y << ", "
               << msg.angular_velocity.z << "), "
               << "Linear Acceleration: ("
               << msg.linear_acceleration.x << ", "
               << msg.linear_acceleration.y << ", "
               << msg.linear_acceleration.z << ")"
               << std::endl;

        imu_debug_out_.flush();
    }

    // You can use this streams to output
    // the coordinates of your computed
    // poses
    std::ofstream& odom_out_;
    std::ofstream& imu_out_;
    std::ofstream& imu_debug_out_;
    std::ofstream& odom_filter_out_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_filter_;
	
    double odom_offset_x_ = 0.0;
    double odom_offset_y_ = 0.0;
    double imu_offset_x_ = 0.0;
    double imu_offset_y_ = 0.0;

    bool isMoving = false;
    bool first_imu_ = true;
    bool first_odom_ = true;
    
    double x_odom = 0, y_odom = 0;
	double t_vor = 0;
	double x_vor = 0, y_vor = 0;
	double rotateAngular_vor = 0;
};

int main(int argc, char** argv)
{
    std::ofstream odom_out("odometry.txt", std::ios::out | std::ios::trunc);
    std::ofstream imu_out("imu.txt", std::ios::out | std::ios::trunc);
    std::ofstream imu_debug_out_("imu_debug.txt", std::ios::out | std::ios::trunc);
    std::ofstream odom_filter_out("odometry_filtered.txt", std::ios::out | std::ios::trunc);


    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryRecorder>(odom_out, imu_out, imu_debug_out_, odom_filter_out));
    rclcpp::shutdown();
}