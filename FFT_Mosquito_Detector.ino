#include <arduinoFFT.h>
#include <ESP32Servo.h>

#define SAMPLES 512
#define SAMPLING_FREQUENCY 8000

#define ROTATION_SERVO_PIN 14   // 360도 회전 서보 핀
#define SPRAY_SERVO_PIN 27      // 스프레이용 SG90 서보 핀

double vReal[4][SAMPLES];
double vImag[4][SAMPLES];

ArduinoFFT<double> FFT[] = {
  ArduinoFFT<double>(vReal[0], vImag[0], SAMPLES, SAMPLING_FREQUENCY),
  ArduinoFFT<double>(vReal[1], vImag[1], SAMPLES, SAMPLING_FREQUENCY),
  ArduinoFFT<double>(vReal[2], vImag[2], SAMPLES, SAMPLING_FREQUENCY),
  ArduinoFFT<double>(vReal[3], vImag[3], SAMPLES, SAMPLING_FREQUENCY)
};

const int micPins[4] = {32, 33, 34, 25};
Servo rotationServo;
Servo sprayServo;

void setup() {
  Serial.begin(115200);
  rotationServo.attach(ROTATION_SERVO_PIN);
  sprayServo.attach(SPRAY_SERVO_PIN);
}

void rotateToMic(int micIndex) {
  // 360도 서보 방향 설정 (예시)
  if (micIndex == 0) rotationServo.write(90);      // 정지
  else if (micIndex == 1) rotationServo.write(120); // 오른쪽
  else if (micIndex == 2) rotationServo.write(180); // 더 오른쪽
  else if (micIndex == 3) rotationServo.write(60);  // 왼쪽

  delay(500); // 회전 대기
  rotationServo.write(90); // 정지
}

void spray() {
  sprayServo.write(0);
  delay(300);
  sprayServo.write(60);
  delay(300);
  sprayServo.write(0);
}

void loop() {
  unsigned long microseconds;
  unsigned int sampling_period_us = 1000000 / SAMPLING_FREQUENCY;

  int detectedMic = -1;
  double maxPeakValue = 0;

  // 샘플 수집
  for (int i = 0; i < SAMPLES; i++) {
    microseconds = micros();
    for (int j = 0; j < 4; j++) {
      vReal[j][i] = analogRead(micPins[j]);
      vImag[j][i] = 0;
    }
    while (micros() - microseconds < sampling_period_us);
  }

  // 마이크별 FFT 분석 및 고조파 판별
  for (int j = 0; j < 4; j++) {
    FFT[j].windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT[j].compute(FFT_FORWARD);
    FFT[j].complexToMagnitude();

    double resolution = (double)SAMPLING_FREQUENCY / SAMPLES;
    int peakIndex1 = 6; double peakVal1 = 0;
    int peakIndex2 = 6; double peakVal2 = 0;
    int peakIndex3 = 6; double peakVal3 = 0;

    for (int i = 6; i < SAMPLES / 2; i++) {
      double val = vReal[j][i];
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

    Serial.print("[MIC"); Serial.print(j); Serial.println("] Top 3 Frequencies:");
    Serial.print("1: "); Serial.print(freq1); Serial.print(" Hz ("), Serial.print(peakVal1); Serial.println(")");
    Serial.print("2: "); Serial.print(freq2); Serial.print(" Hz ("), Serial.print(peakVal2); Serial.println(")");
    Serial.print("3: "); Serial.print(freq3); Serial.print(" Hz ("), Serial.print(peakVal3); Serial.println(")");

    int inRangeCount = 0;
    if (freq1 >= 400 && freq1 <= 800) inRangeCount++;
    if (freq2 >= 400 && freq2 <= 800) inRangeCount++;
    if (freq3 >= 400 && freq3 <= 800) inRangeCount++;

    if (inRangeCount >= 2) {
      Serial.println("→ [모기소리 감지됨: Top3 중 2개 이상 400~600Hz]");
      if (peakVal1 > maxPeakValue) {
        detectedMic = j;
        maxPeakValue = peakVal1;
      }
    } else {
      Serial.println("→ [신호 없음]");
    }

    Serial.println();
  }

  if (detectedMic != -1) {
    rotateToMic(detectedMic);
    spray();
  }

  delay(1000);
}
