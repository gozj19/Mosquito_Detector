#include <arduinoFFT.h>            // FFT (주파수 분석)를 위한 라이브러리
#include <ESP32Servo.h>            // ESP32용 서보 제어 라이브러리

#define SAMPLES 512                // FFT 샘플 개수
#define SAMPLING_FREQUENCY 8000   // 샘플링 주파수 (Hz)

#define ROTATION_SERVO_PIN 14     // 360도 회전 서보 제어 핀
#define SPRAY_SERVO_PIN 27        // 스프레이 SG90 서보 제어 핀

// 마이크 별 FFT 입력 배열 선언
double vReal[4][SAMPLES];
double vImag[4][SAMPLES];

// 마이크 4개 각각 FFT 인스턴스 생성
ArduinoFFT<double> FFT[] = {
  ArduinoFFT<double>(vReal[0], vImag[0], SAMPLES, SAMPLING_FREQUENCY),
  ArduinoFFT<double>(vReal[1], vImag[1], SAMPLES, SAMPLING_FREQUENCY),
  ArduinoFFT<double>(vReal[2], vImag[2], SAMPLES, SAMPLING_FREQUENCY),
  ArduinoFFT<double>(vReal[3], vImag[3], SAMPLES, SAMPLING_FREQUENCY)
};

// 마이크 입력 핀 배열 (아날로그)
const int micPins[4] = {32, 33, 34, 25};

// 서보 객체 선언
Servo rotationServo;
Servo sprayServo;

void setup() {
  Serial.begin(115200);                            // 시리얼 통신 시작
  rotationServo.attach(ROTATION_SERVO_PIN);       // 회전 서보 연결
  sprayServo.attach(SPRAY_SERVO_PIN);             // 스프레이 서보 연결
}

// 마이크 방향으로 360도 서보 회전
void rotateToMic(int micIndex) {
  // 마이크 위치에 따라 회전 방향 지정
  if (micIndex == 0) rotationServo.write(90);       // 정지 (기본값)
  else if (micIndex == 1) rotationServo.write(120); // 약간 오른쪽
  else if (micIndex == 2) rotationServo.write(180); // 더 오른쪽
  else if (micIndex == 3) rotationServo.write(60);  // 왼쪽

  delay(500);                     // 회전 시간 대기
  rotationServo.write(90);       // 정지
}

// SG90으로 스프레이 분사
void spray() {
  sprayServo.write(0);           // 초기 위치
  delay(300);
  sprayServo.write(60);         // 분사 방향 회전 (버튼 누르기 등)
  delay(300);
  sprayServo.write(0);          // 원위치
}

void loop() {
  unsigned long microseconds;
  unsigned int sampling_period_us = 1000000 / SAMPLING_FREQUENCY;

  int detectedMic = -1;         // 모기소리 감지된 마이크 번호
  double maxPeakValue = 0;      // 가장 강한 peak 값

  // --- 1. 마이크 데이터 샘플 수집 ---
  for (int i = 0; i < SAMPLES; i++) {
    microseconds = micros();
    for (int j = 0; j < 4; j++) {
      vReal[j][i] = analogRead(micPins[j]);  // 마이크 아날로그 입력값
      vImag[j][i] = 0;
    }
    while (micros() - microseconds < sampling_period_us); // 샘플링 주기 유지
  }

  // --- 2. 마이크별 FFT 분석 ---
  for (int j = 0; j < 4; j++) {
    FFT[j].windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);   // 해밍 윈도우 적용
    FFT[j].compute(FFT_FORWARD);                          // FFT 계산
    FFT[j].complexToMagnitude();                          // 복소수 → 진폭 변환

    double resolution = (double)SAMPLING_FREQUENCY / SAMPLES;  // 주파수 해상도

    // Top 3 peak 주파수 및 세기 찾기
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

    // 시리얼 출력
    Serial.print("[MIC"); Serial.print(j); Serial.println("] Top 3 Frequencies:");
    Serial.print("1: "); Serial.print(freq1); Serial.print(" Hz ("); Serial.print(peakVal1); Serial.println(")");
    Serial.print("2: "); Serial.print(freq2); Serial.print(" Hz ("); Serial.print(peakVal2); Serial.println(")");
    Serial.print("3: "); Serial.print(freq3); Serial.print(" Hz ("); Serial.print(peakVal3); Serial.println(")");

    // 모기 주파수 범위 감지 (400~800Hz) Top 3 중 2개 이상
    // 모기 주파수 범위는 400~600Hz 인데 MAX4486 마이크 센서가 기준 주파수보다 소리를 더 크게 잡아서 범위를 늘림 
    // ex) 500Hz 발생시 MAX4486은 약 600Hz로 인식
    int inRangeCount = 0;
    if (freq1 >= 400 && freq1 <= 800) inRangeCount++;
    if (freq2 >= 400 && freq2 <= 800) inRangeCount++;
    if (freq3 >= 400 && freq3 <= 800) inRangeCount++;

    if (inRangeCount >= 2) {
      Serial.println("→ [모기 소리 감지됨: Top3 중 2개 이상 400~800Hz]");
      if (peakVal1 > maxPeakValue) {
        detectedMic = j;         // 가장 강한 마이크 선택
        maxPeakValue = peakVal1;
      }
    } else {
      Serial.println("→ [신호 없음]");
    }

    Serial.println();
  }

  // --- 3. 회전 및 스프레이 동작 실행 ---
  if (detectedMic != -1) {
    rotateToMic(detectedMic);  // 해당 방향 회전
    spray();                   // 스프레이 분사
  }

  delay(1000); // 다음 루프까지 대기
}
