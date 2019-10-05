#include <SabertoothSimplified.h>
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
  //delay(7000); //Plug TX pin into Arduino during this time
}
void loop() {
  //Serial.println("ready");
  // Read input from serial
  String strIn = readSerial();
  // Process input from serial and calls the corresponding function
  processSerial(strIn);
}
void setMotorVal(int motor_val) {
  // motor_val -127 to 127
  ST.motor(2, motor_val);
}
void setSteerVal(int steer_val) {
   // steer_val -127 to 127
   ST.motor(1, steer_val);
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
  char controlType = strIn.charAt(0);
  int controlVal = strIn.substring(2).toInt();
  if (controlType == 'V' && controlVal>-128 && controlVal<128) {
   setMotorVal(controlVal);
   Serial.println("got drive");
  } else if (controlType == 'W' && controlVal>-128 && controlVal<128) {
   setSteerVal(controlVal);
  } //else {
   //setMotorVal(0);
   //setSteerVal(0);
  //}
}
