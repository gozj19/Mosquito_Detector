#include <ESP32Servo.h>

#define ROTATION_SERVO_PIN 14  // MG996R 서보 연결 핀

Servo rotationServo;

void setup() {
  rotationServo.attach(ROTATION_SERVO_PIN);
}

void loop() {
  // 순차적으로 여러 각도로 이동하며 테스트
  rotationServo.write(0);
  delay(1500);

  rotationServo.write(90);
  delay(1500);

  rotationServo.write(180);  // 중심
  delay(1500);

  rotationServo.write(270);
  delay(1500);

  rotationServo.write(360);
  delay(1500);
}
