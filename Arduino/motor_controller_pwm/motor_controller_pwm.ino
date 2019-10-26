#include <ros.h>
#include <low_level/theta_dot_lr.h>
#include <std_msgs/String.h>
#include <SabertoothSimplified.h>

#define LEFT_MOTOR_PIN 5
#define RIGHT_MOTOR_PIN 6
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
  int motorPwr = speed * 127 / 10.83 + 127;
  return (int) max(min(motorPwr, 255), 0);
}
  std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);


void callback(low_level::theta_dot_lr& msg) {
  //char buf[16];
  //nh.loginfo("Setting right = ");
  //nh.loginfo(itoa(mapSpeedToMotorVal(msg.theta_dot_right), buf, 10));
  //ST.motor(LEFT_MOTOR_ID, mapSpeedToMotorVal(msg.theta_dot_left));
  //ST.motor(RIGHT_MOTOR_ID, mapSpeedToMotorVal(msg.theta_dot_right));



  analogWrite(LEFT_MOTOR_PIN, mapSpeedToMotorVal(msg.theta_dot_left));
  analogWrite(RIGHT_MOTOR_PIN, mapSpeedToMotorVal(msg.theta_dot_right));
  delay(1);
  //analogWrite(LEFT_MOTOR_PIN, 127);
  //analogWrite(RIGHT_MOTOR_PIN, 127);
  //delay(1);
}

ros::Subscriber<low_level::theta_dot_lr> sub("ll_control", &callback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  //SabertoothTXPinSerial.begin(9600);
  //ST.stop();
  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  analogWrite(LEFT_MOTOR_PIN, 127);
  analogWrite(RIGHT_MOTOR_PIN, 127); 
  
}

void loop() {
  //analogWrite(LEFT_MOTOR_PIN, 127);
  //analogWrite(RIGHT_MOTOR_PIN, 127);
  char hello[13] = "callback";
  str_msg.data = hello;
  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(1);
}
