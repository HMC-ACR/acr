// Motor controller node for Autonomous Campus Robot
// David Linn, 10/17/19, dlinn@hmc.edu

#include <ros/ros.h>
#include "low_level/theta_dot_lr.h"
#include <algorithm>
// C library headers
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#define LEFT_MOTOR_ID 1
#define RIGHT_MOTOR_ID 2

class MotorController {
public:
    void motor(int which, int power) {
        unsigned char command, magnitude;
        power = std::max(std::min(power, 127), -127);
        magnitude = abs(power) >> 1;
        
        if (which == 1)
        {
            command = power < 0 ? 63 - magnitude : 64 + magnitude;
        }
        else if (which == 2)
        {
            command = power < 0 ? 191 - magnitude : 192 + magnitude;
        }
        
        command = std::max(std::min((int)command, 254), 1);
        write(serial_port, &command, 1);
    }
    
    int mapSpeedToMotorVal(double speed){
        int val = speed * 127 / 10.83;
        return (int) std::max(std::min(val, 127), -127);
    }
    
    MotorController() {
        sub_ = n_.subscribe("ll_control", 1, &MotorController::callback, this);
        configSerial();
    }

    void callback(const low_level::theta_dot_lr& msg) {
        motor(LEFT_MOTOR_ID, mapSpeedToMotorVal(msg.theta_dot_left));
        motor(RIGHT_MOTOR_ID, mapSpeedToMotorVal(msg.theta_dot_right));
    }

    void configSerial() {
        serial_port = open("/dev/ttyTHS2", O_RDWR);
        if (serial_port < 0) {
            printf("Error %i from open: %s\n", errno, strerror(errno));
        }
        // Create new termios struct, we call it 'tty' for convention
        struct termios tty;
        memset(&tty, 0, sizeof tty);

        // Read in existing settings, and handle any error
        if(tcgetattr(serial_port, &tty) != 0) {
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        }
        tty.c_cflag &= ~PARENB; // Clear parity bit
        tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication
        tty.c_cflag |= CS8; // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
        tty.c_lflag &= ~ICANON; // Disable canonical mode
        tty.c_lflag &= ~ECHO; // Disable echo
        tty.c_lflag &= ~ECHOE; // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo
        tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        tty.c_cc[VTIME] = 0;    //No blocking reads
        tty.c_cc[VMIN] = 0;
        // Set in/out baud rate to be 9600
        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);
        // Save tty settings, also checking for error
        if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
            printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        }
    }

private:
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    int serial_port;
};

int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "MotorController");

    MotorController node;

    ros::spin();
    // Replace above line with below if we need faster updating
    //while(true) {
    //  ros::Rate(YOUR_DESIRED_RATE).sleep(); ros::spinOnce();
    //}


    return 0;
}
