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
void setup() {
  SabertoothTXPinSerial.begin(9600);
  Serial.begin(9600);
  setMotorVal(LEFT_MOTOR_ID, 0);
  setMotorVal(RIGHT_MOTOR_ID, 0);
}

void loop() {
  // Read input from serial
  String strIn = readSerial();
  // Process input from serial and calls the corresponding function
  processSerial(strIn);
}

// Mapping from rotational velocity to motor controller value
// TODO(AQP): Emperically determine and implement mapping1
// Currently cap the speed/value [-127, 127]
int mapSpeedToMotorVal(double speed){
  return (int) max(min(speed, 127), -127);
}

void setMotorVal(int motor, int value) {
  // motor_val -127 (full reverse) to 127 (full forward)
  ST.motor(motor, value);
}

String readSerial() {
   String strIn = "";
   if (Serial.available() > 0) {
     strIn = Serial.readString();
   }
   return strIn;
}

void processSerial(String strIn) {
  // Processes the command information and sets the relevant motor/steer values.
  char motorSide = strIn.charAt(0);
  double motorSpeed = strIn.substring(2).toInt();
  int value = mapSpeedToMotorVal(motorSpeed);
  if (motorSide == 'L') {
   setMotorVal(LEFT_MOTOR_ID, value);
  } else if (motorSide == 'R') {
   setMotorVal(RIGHT_MOTOR_ID, value);
  }
}
