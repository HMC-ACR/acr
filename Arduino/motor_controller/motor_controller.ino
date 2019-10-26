#include <ros.h>
#include <low_level/theta_dot_lr.h>
#include <SabertoothSimplified.h>

#define LEFT_MOTOR_ID 1
#define RIGHT_MOTOR_ID 2
SabertoothSimplified ST; // We'll name the Sabertooth object ST.
                        // For how to configure the Sabertooth, see the DIP Switch Wizard for
                        //   http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm
                        // Be sure to select Simplified Serial Mode for use with this library.
                        // This sample uses a baud rate of 9600.
                        //
                        // Connections to make:
                        //   Arduino TX->1  ->  Sabertooth S1
                        //   Arduino GND    ->  Sabertooth 0V
                        //   Arduino VIN    ->  Sabertooth 5V (OPTIONAL, if you want the Sabertooth to power the Arduino)
                        //
                        // If you want to use a pin other than TX->1, see the SoftwareSerial example.
ros::NodeHandle nh;

// Mapping from wheel rotational velocity (radians per second) to motor controller value
// TODO(AQP): Emperically determine and implement mapping
// Cap the speed/value [-127, 127]
// Rough mapping -- w_max = V_max/r = 1.788 m/s / .1651 meters = 10.83 radians/s
int mapSpeedToMotorVal(double speed){
  speed = speed * 127 / 10.83;
  return (int) max(min(speed, 127), -127);
}

void callback(low_level::theta_dot_lr& msg) {
  //char buf[16];
  //nh.loginfo("Setting right = ");
  //nh.loginfo(itoa(mapSpeedToMotorVal(msg.theta_dot_right), buf, 10));
  if (msg.theta_dot_left == 0 && msg.theta_dot_right == 0) {
    ST.stop();
  }
  ST.motor(LEFT_MOTOR_ID, mapSpeedToMotorVal(msg.theta_dot_left));
  ST.motor(RIGHT_MOTOR_ID, mapSpeedToMotorVal(msg.theta_dot_right));
}

ros::Subscriber<low_level::theta_dot_lr> sub("ll_control", &callback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  SabertoothTXPinSerial.begin(9600);
  ST.stop();
}

void loop() {
  nh.spinOnce();
  delay(1);
}
