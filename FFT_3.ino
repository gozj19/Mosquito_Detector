#include <arduinoFFT.h>
#include <ESP32Servo.h>

#define SAMPLES 256
#define SAMPLING_FREQUENCY 8000

#define ROTATION_SERVO_PIN 14  // MG996R - 회전용
#define SPRAY_SERVO_PIN 27     // SG90 - 스프레이용

const int micPins[3] = {32, 33, 34}; // MIC0~2 사용

double vReal[SAMPLES];
double vImag[SAMPLES];

Servo rotationServo;  // 회전용 (MG996R)
Servo sprayServo;     // 스프레이용 (SG90)

ArduinoFFT<double> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

int sprayCount[3] = {0, 0, 0}; // 방향별 최대 2회

void setup() {
  rotationServo.attach(ROTATION_SERVO_PIN);
  sprayServo.attach(SPRAY_SERVO_PIN);
  rotationServo.write(90); // MG996R: 정지 상태
  sprayServo.write(0);     // SG90: 초기 위치
}

void rotateToMic(int micIndex) {
  int angle = 90;
  if (micIndex == 0) angle = 0;
  else if (micIndex == 1) angle = 90;
  else if (micIndex == 2) angle = 180;

  rotationServo.write(angle); // MG996R 회전
  delay(500);                 // 회전 대기
  rotationServo.write(90);   // 정지
}

void spray() {
  sprayServo.write(60); // 누름 동작
  delay(300);
  sprayServo.write(0);  // 원위치
  delay(300);
}

void loop() {
  unsigned long microseconds;
  unsigned int sampling_period_us = 1000000 / SAMPLING_FREQUENCY;

  for (int mic = 0; mic < 3; mic++) {
    for (int i = 0; i < SAMPLES; i++) {
      microseconds = micros();
      vReal[i] = analogRead(micPins[mic]);
      vImag[i] = 0;
      while (micros() - microseconds < sampling_period_us);
    }

    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();

    double resolution = (double)SAMPLING_FREQUENCY / SAMPLES;
    int peakIndex1 = 6, peakIndex2 = 6, peakIndex3 = 6;
    double peakVal1 = 0, peakVal2 = 0, peakVal3 = 0;

    for (int i = 6; i < SAMPLES / 2; i++) {
      double val = vReal[i];
      if (val > peakVal1) {
        peakVal3 = peakVal2; peakIndex3 = peakIndex2;
        peakVal2 = peakVal1; peakIndex2 = peakIndex1;
        peakVal1 = val; peakIndex1 = i;
      } else if (val > peakVal2) {
        peakVal3 = peakVal2; peakIndex3 = peakIndex2;
        peakVal2 = val; peakIndex2 = i;
      } else if (val > peakVal3) {
        peakVal3 = val; peakIndex3 = i;
      }
    }

    double freq1 = peakIndex1 * resolution;
    double freq2 = peakIndex2 * resolution;
    double freq3 = peakIndex3 * resolution;

    int inRange = 0;
    if (freq1 >= 400 && freq1 <= 800) inRange++;
    if (freq2 >= 400 && freq2 <= 800) inRange++;
    if (freq3 >= 400 && freq3 <= 800) inRange++;

    if (inRange >= 2) {
      rotateToMic(mic);    // 회전
      delay(300);          // 안정 대기
      spray();             // 스프레이 누름
      sprayCount[mic]++;
      delay(200);
      rotationServo.write(90); // 회전 서보 정지 (제자리 복귀 상태)
      delay(400);
      break;
    }
  }
}
