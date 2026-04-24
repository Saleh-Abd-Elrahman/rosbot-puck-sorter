#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>

static const int LED_PIN = 13;
static const int SERVO_PIN = 9;
static const int LOAD_PIN = A0;
static const unsigned long LOAD_PUBLISH_PERIOD_MS = 1000;

ros::NodeHandle nh;
Servo servo;
std_msgs::Float32 load_msg;

void servo_cb(const std_msgs::UInt16& cmd_msg) {
  uint16_t angle = cmd_msg.data;
  if (angle > 180) {
    angle = 180;
  }
  servo.write(angle);
  digitalWrite(LED_PIN, HIGH - digitalRead(LED_PIN));
}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);
ros::Publisher pub_servo_load("servoLoad", &load_msg);

float read_load_ma() {
  // Matches the conversion shown in the lecture slide.
  return (((analogRead(LOAD_PIN) * 5.0f) / 1023.0f) / 2.0f);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  servo.attach(SERVO_PIN);
  servo.write(0);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_servo_load);
}

void loop() {
  static unsigned long last_pub_ms = 0;
  unsigned long now = millis();

  if (now - last_pub_ms >= LOAD_PUBLISH_PERIOD_MS) {
    load_msg.data = read_load_ma();
    pub_servo_load.publish(&load_msg);
    last_pub_ms = now;
  }

  nh.spinOnce();
  delay(5);
}
