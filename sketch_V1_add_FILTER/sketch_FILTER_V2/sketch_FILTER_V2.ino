#include <Wire.h>
#include <MPU9250.h>
#include <LCDI2C_Multilingual.h>
#include <BluetoothSerial.h>
#include <SimpleKalmanFilter.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Ticker.h>

// MPU9250, LCD, Bluetooth 객체 생성
MPU9250 mpu;
LCDI2C_Symbols lcd(0x27, 16, 2);
BluetoothSerial SerialBT;

// 핀 설정 및 변수 초기화
int ledPin = 2;
int selectButtonPin = 15, zeroButtonPin = 4;
int lastSelectButtonState = HIGH, lastZeroButtonState = HIGH;
unsigned long lastDebounceTime = 0, debounceDelay = 50;
int pwmValue = 0;
bool increasing = true;

float rollOffset = 0, pitchOffset = 0, yawOffset = 0;
int selectedValue = 0;

// Kalman 필터 객체 생성
SimpleKalmanFilter rollKalmanFilter(2, 2, 0.01);
SimpleKalmanFilter pitchKalmanFilter(2, 2, 0.01);
SimpleKalmanFilter yawKalmanFilter(2, 2, 0.01);

// 필터 가중치 설정 (상보 필터용)
float alpha = 0.98;

// 이동 평균 필터용 변수
const int windowSize = 5;
float rollWindow[windowSize] = {0}, pitchWindow[windowSize] = {0}, yawWindow[windowSize] = {0};
int windowIndex = 0;
float rollSum = 0, pitchSum = 0, yawSum = 0;

// FreeRTOS 태스크 핸들
TaskHandle_t imuTaskHandle = NULL, lcdTaskHandle = NULL, ledTaskHandle = NULL;
Ticker ticker;

// 상보 필터 함수
float complementaryFilter(float gyro, float accel, float dt) {
  return alpha * (gyro * dt) + (1 - alpha) * accel;
}

// 이동 평균 필터 함수
float movingAverageFilter(float *window, float *sum, float newValue) {
  *sum -= window[windowIndex]; // 기존 값을 합에서 제외
  window[windowIndex] = newValue; // 새로운 값을 창에 추가
  *sum += newValue; // 합에 새로운 값 추가
  windowIndex = (windowIndex + 1) % windowSize; // 창의 인덱스 갱신
  return *sum / windowSize; // 창 안의 값의 평균 반환
}

// FreeRTOS 태스크를 깨우는 함수
void wakeUpTasks() {
  if (imuTaskHandle != NULL) xTaskNotifyGive(imuTaskHandle);
  if (lcdTaskHandle != NULL) xTaskNotifyGive(lcdTaskHandle);
  if (ledTaskHandle != NULL) xTaskNotifyGive(ledTaskHandle);
}

// 스위치 처리 함수
void handleSwitches() {
  int selectReading = digitalRead(selectButtonPin);
  int zeroReading = digitalRead(zeroButtonPin);

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (selectReading != lastSelectButtonState) {
      lastDebounceTime = millis();
      if (selectReading == LOW) selectedValue = (selectedValue + 1) % 3;
    }
    if (zeroReading != lastZeroButtonState) {
      lastDebounceTime = millis();
      if (zeroReading == LOW) {
        if (selectedValue == 0) rollOffset = mpu.getRoll();
        else if (selectedValue == 1) pitchOffset = mpu.getPitch();
        else if (selectedValue == 2) yawOffset = mpu.getYaw();
      }
    }
  }
  lastSelectButtonState = selectReading;
  lastZeroButtonState = zeroReading;
}

// IMU 데이터 읽기 및 필터 적용 함수
void readIMUData() {
  if (mpu.update()) {
    float dt = 0.01; // 적절한 시간 차이로 설정

    // 자이로 및 가속도 데이터를 사용하여 보정된 롤, 피치, 요 값을 계산
    float gyroRoll = mpu.getGyroX();
    float accelRoll = mpu.getAccX(); 
    float complementaryRoll = complementaryFilter(gyroRoll, accelRoll, dt);

    float gyroPitch = mpu.getGyroY();
    float accelPitch = mpu.getAccY();
    float complementaryPitch = complementaryFilter(gyroPitch, accelPitch, dt);

    float gyroYaw = mpu.getGyroZ();
    float accelYaw = mpu.getAccZ(); // 주의: 실제로는 요 값은 자이로에서만 사용 가능
    float complementaryYaw = complementaryFilter(gyroYaw, accelYaw, dt);

    Serial.printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", complementaryRoll, complementaryPitch, complementaryYaw);
    SerialBT.printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", complementaryRoll, complementaryPitch, complementaryYaw);
  } else {
    Serial.println("IMU Update Failed");
  }
}


// LCD 업데이트 함수
void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);

  switch (selectedValue) {
    case 0: lcd.print("Roll: "), lcd.print(rollKalmanFilter.updateEstimate(mpu.getRoll()) - rollOffset, 2); break;
    case 1: lcd.print("Pitch: "), lcd.print(pitchKalmanFilter.updateEstimate(mpu.getPitch()) - pitchOffset, 2); break;
    case 2: lcd.print("Yaw: "), lcd.print(yawKalmanFilter.updateEstimate(mpu.getYaw()) - yawOffset, 2); break;
  }

  lcd.setCursor(0, 1);
  lcd.print("PWM: ");
  lcd.print(pwmValue);
}

// LED 업데이트 함수
void updateLED() {
  if (increasing) {
    pwmValue += 5;
    if (pwmValue >= 255) {
      pwmValue = 255;
      increasing = false;
    }
  } else {
    pwmValue -= 5;
    if (pwmValue <= 0) {
      pwmValue = 0;
      increasing = true;
    }
  }

  ledcWrite(2, pwmValue);
  SerialBT.printf("PWM Value: %d\n", pwmValue);
}

// IMU 태스크 함수
void imuTask(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    readIMUData();
  }
}

// LCD 태스크 함수
void lcdTask(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    updateLCD();
  }
}

// LED 태스크 함수
void ledTask(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    updateLED();
  }
}

// setup 함수
void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(ledPin, OUTPUT);
  pinMode(selectButtonPin, INPUT_PULLUP);
  pinMode(zeroButtonPin, INPUT_PULLUP);

  ledcAttach(2, 5000, 8);
  lcd.init();
  lcd.backlight();
  mpu.setup(0x68);
  SerialBT.begin("ESP32_BT");

  xTaskCreate(imuTask, "IMU Task", 2048, NULL, 1, &imuTaskHandle);
  xTaskCreate(lcdTask, "LCD Task", 2048, NULL, 1, &lcdTaskHandle);
  xTaskCreate(ledTask, "LED Task", 2048, NULL, 1, &ledTaskHandle);

  ticker.attach_ms(40, wakeUpTasks);
}

// loop 함수
void loop() {
  handleSwitches();
}
