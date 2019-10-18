// Dynamic model node for Autonomous Campus Robot
// David Linn, 10/17/19, dlinn@hmc.edu

#include <ros/ros.h>
#include "my_msgs/theta_dot_lr.h"

class DynamicModel {
public:
    // Converts linear velocity (v) and rotational velocity (w) to
    // theta_dot_left and theta_dot_right for differential drive robot
    my_msgs::theta_dot_lr model(float v, float w) {
        my_msgs::theta_dot_lr output;
        output.theta_dot_left = ;
        output.theta_dot_right = ;
        return output;
    }
    
    DynamicModel() {
        pub_ = n_.advertise<my_msgs::theta_dor_lr>("ll_control", 1);
        sub_ = n_.subscribe("turtle1/cmd_vel", 1, &DynamicModel::callback, this);
    }

    void callback(const geometry_msgs::Twist &input) {
        my_msgs::theta_dot_lr ll_control_msg = model(input.v, input.w);
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