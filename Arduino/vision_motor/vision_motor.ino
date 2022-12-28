#if defined(ESP8266) or defined(ESP32) or defined(ROSSERIAL_ARDUINO_TCP) 
  #include "ArduinoTcpHardware.h" 
#else 
  #include "ArduinoHardware.h" 
#endif 
#include <ros.h>
#include <vision_msgs/ArduinoSrv.h>
using vision_msgs::ArduinoSrv;

void callback(const ArduinoSrv::Request &req, ArduinoSrv::Response &res);
bool cmd_release();
bool cmd_forward();
bool cmd_stop();
void manual_control(char input);
void count();
int IN1Pin = 32; //motor direction, (0:backward, 1:forward)
int IN2Pin = 33; //Break, (0:rotate, 1:break)
int ENPin = 11; //motor speed 0~255
int HA = 2; //홀센서A
int HB = 3; //홀센서B
int cnt = 0;
ros::NodeHandle nh;
ros::ServiceServer<ArduinoSrv::Request, ArduinoSrv::Response> server("marking_srv", &callback);


void setup() {
  nh.initNode();
  nh.advertiseService(server);
  pinMode(IN1Pin, OUTPUT);
  pinMode(IN2Pin, OUTPUT);
  pinMode(HA, INPUT_PULLUP);
  pinMode(HB, INPUT_PULLUP);
  analogWrite(ENPin, 50); //set motor speed 30
  attachInterrupt(digitalPinToInterrupt(HA), count, RISING);
  digitalWrite(13, LOW);
  cmd_stop();
}

void callback(const ArduinoSrv::Request &req, ArduinoSrv::Response &res)
{
  if (req.holes == 0) { cmd_stop(); }
  else if (req.holes > 0) { cmd_forward(); }
}

bool cmd_release()
{
  digitalWrite(IN2Pin, 0);
  digitalWrite(IN1Pin, 0);
  return true;
}

bool cmd_forward() {
  cnt = 0;
  while (cnt < 104) {
    digitalWrite(IN2Pin, 0);
    digitalWrite(IN1Pin, 1);
    Serial.println(cnt);
  }
  cmd_stop();
  cnt = 0;
  return true;
}

bool cmd_stop()
{
  digitalWrite(IN2Pin, 1);
  return true;
}

void count() { cnt++; }


void loop(){
  nh.spinOnce();
  delay(1);
}
