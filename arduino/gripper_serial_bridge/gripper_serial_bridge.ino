#include <Servo.h>

static const int SERVO_PIN = 9;
static const long BAUD_RATE = 115200;
static const int OPEN_ANGLE_DEG = 20;
static const int CLOSE_ANGLE_DEG = 95;

Servo gripperServo;
String rxLine;
int currentAngleDeg = OPEN_ANGLE_DEG;

int clampAngle(int angleDeg) {
  if (angleDeg < 0) {
    return 0;
  }
  if (angleDeg > 180) {
    return 180;
  }
  return angleDeg;
}

void writeAngle(int angleDeg) {
  currentAngleDeg = clampAngle(angleDeg);
  gripperServo.write(currentAngleDeg);
}

void replyOk(const String &message) {
  Serial.print("OK ");
  Serial.println(message);
}

void replyErr(const String &message) {
  Serial.print("ERR ");
  Serial.println(message);
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  if (line.equalsIgnoreCase("PING")) {
    replyOk("PONG");
    return;
  }

  if (line.equalsIgnoreCase("OPEN")) {
    writeAngle(OPEN_ANGLE_DEG);
    replyOk(String("ANGLE ") + currentAngleDeg);
    return;
  }

  if (line.equalsIgnoreCase("CLOSE")) {
    writeAngle(CLOSE_ANGLE_DEG);
    replyOk(String("ANGLE ") + currentAngleDeg);
    return;
  }

  if (line.startsWith("ANGLE ") || line.startsWith("angle ")) {
    String valueText = line.substring(6);
    int angleDeg = valueText.toInt();
    writeAngle(angleDeg);
    replyOk(String("ANGLE ") + currentAngleDeg);
    return;
  }

  replyErr(String("UNKNOWN ") + line);
}

void setup() {
  Serial.begin(BAUD_RATE);
  gripperServo.attach(SERVO_PIN);
  writeAngle(OPEN_ANGLE_DEG);
  rxLine.reserve(48);
}

void loop() {
  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      handleCommand(rxLine);
      rxLine = "";
      continue;
    }
    rxLine += ch;
  }
}
