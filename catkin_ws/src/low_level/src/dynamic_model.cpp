// Dynamic model node for Autonomous Campus Robot
// David Linn, 10/17/19, dlinn@hmc.edu

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "low_level/theta_dot_lr.h"

#define WHEEL_RAD .1905 // meters
#define AXLE_LEN .762 // meters

class DynamicModel {
public:
    // Converts linear velocity (v) and angular velocity (w) to
    // theta_dot_left and theta_dot_right for differential drive robot
    // assuming no slipping
    low_level::theta_dot_lr model(float v, float w) {
        low_level::theta_dot_lr output;
        // Linear velocity v is average of wheel velocities v_l and v_r
        // Angular velocity w is (v_r-v_l)/L [Positive = steering left, L=axle length]
        // Substituting, we have v_r = v+Lw/2 and v_l = v-Lw/2
        // We then find theta_dot_l = v_l/r and theta_dot_r = v_r/r [Units radians/s, r=wheel radius]
        float ang_component = (AXLE_LEN*w)/2;
        output.theta_dot_left = (v - ang_component)/WHEEL_RAD;
        output.theta_dot_right = (v + ang_component)/WHEEL_RAD;
        return output;
    }
    
    DynamicModel() {
        pub_ = n_.advertise<low_level::theta_dot_lr>("ll_control", 1);
        sub_ = n_.subscribe("turtle1/cmd_vel", 1, &DynamicModel::callback, this);
    }

    void callback(const geometry_msgs::Twist &input) {
        low_level::theta_dot_lr ll_control_msg = model(input.linear.x, input.angular.z);
        pub_.publish(ll_control_msg);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "DynamicModel");

    DynamicModel node;

    ros::spin();
    // Replace above line with below if we need faster updating
    //while(true) {
    //  ros::Rate(YOUR_DESIRED_RATE).sleep(); ros::spinOnce();
    //}

    return 0;
}