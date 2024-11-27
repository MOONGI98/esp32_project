#include <Wire.h>
#include <MPU9250.h>
#include <LCDI2C_Multilingual.h>
#include <BluetoothSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Ticker.h>

// MPU9250, LCD, Bluetooth 객체 생성
MPU9250 mpu;
LCDI2C_Symbols lcd(0x27, 16, 2);
BluetoothSerial SerialBT;

// 핀 설정 및 변수 초기화
int ledPin = 2;
int selectButtonPin = 14 , zeroButtonPin = 27;
int lastSelectButtonState = HIGH, lastZeroButtonState = HIGH;
unsigned long lastDebounceTime = 0, debounceDelay = 50;
int pwmValue = 0;
bool increasing = true;

float rollOffset = 0, pitchOffset = 0, yawOffset = 0;
int selectedValue = 0;

// 칼만 필터 변수
float Q_angle = 0.001, Q_gyro = 0.003, R_angle = 0.03;
float roll_angle = 0, roll_bias = 0, P_roll[2][2] = {{0, 0}, {0, 0}};
float pitch_angle = 0, pitch_bias = 0, P_pitch[2][2] = {{0, 0}, {0, 0}};
float yaw_angle = 0, yaw_bias = 0, P_yaw[2][2] = {{0, 0}, {0, 0}};

// 상보 필터의 가중치 설정
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
  *sum -= window[windowIndex];
  window[windowIndex] = newValue;
  *sum += newValue;
  windowIndex = (windowIndex + 1) % windowSize;
  return *sum / windowSize;
}

// 일반 칼만 필터 함수
float kalmanFilter(float newAngle, float newRate, float dt, float &angle, float &bias, float P[2][2]) {
  float rate = newRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_gyro * dt;

  float S = P[0][0] + R_angle;
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  float y = newAngle - angle;
  angle += K[0] * y;
  bias += K[1] * y;

  P[0][0] -= K[0] * P[0][0];
  P[0][1] -= K[0] * P[0][1];
  P[1][0] -= K[1] * P[0][0];
  P[1][1] -= K[1] * P[0][1];

  return angle;
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
        if (selectedValue == 0) rollOffset = roll_angle;
        else if (selectedValue == 1) pitchOffset = pitch_angle;
        else if (selectedValue == 2) yawOffset = yaw_angle;
      }
    }
  }
  lastSelectButtonState = selectReading;
  lastZeroButtonState = zeroReading;
}

// IMU 데이터 읽기 및 필터 적용 함수
void readIMUData() {
  if (mpu.update()) {
    float dt = 0.01;

    // 자이로 및 가속도 데이터를 상보 필터와 이동 평균 필터로 처리 후 칼만 필터에 전달
    float gyroRoll = mpu.getGyroX();
    float accelRoll = mpu.getAccX();
    float complementaryRoll = complementaryFilter(gyroRoll, accelRoll, dt);
    float filteredRoll = movingAverageFilter(rollWindow, &rollSum, complementaryRoll);
    roll_angle = kalmanFilter(filteredRoll, gyroRoll, dt, roll_angle, roll_bias, P_roll) - rollOffset;

    float gyroPitch = mpu.getGyroY();
    float accelPitch = mpu.getAccY();
    float complementaryPitch = complementaryFilter(gyroPitch, accelPitch, dt);
    float filteredPitch = movingAverageFilter(pitchWindow, &pitchSum, complementaryPitch);
    pitch_angle = kalmanFilter(filteredPitch, gyroPitch, dt, pitch_angle, pitch_bias, P_pitch) - pitchOffset;

    float gyroYaw = mpu.getGyroZ();
    float accelYaw = mpu.getAccZ();
    float complementaryYaw = complementaryFilter(gyroYaw, accelYaw, dt);
    float filteredYaw = movingAverageFilter(yawWindow, &yawSum, complementaryYaw);
    yaw_angle = kalmanFilter(filteredYaw, gyroYaw, dt, yaw_angle, yaw_bias, P_yaw) - yawOffset;

    Serial.printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll_angle, pitch_angle, yaw_angle);
    SerialBT.printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll_angle, pitch_angle, yaw_angle);
  } else {
    Serial.println("IMU Update Failed");
  }
}

// LCD 업데이트 함수
void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);

  switch (selectedValue) {
    case 0: lcd.print("Roll: "), lcd.print(roll_angle, 2); break;
    case 1: lcd.print("Pitch: "), lcd.print(pitch_angle, 2); break;
    case 2: lcd.print("Yaw: "), lcd.print(yaw_angle, 2); break;
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

  xTaskCreate(imuTask, "IMU Task", 2048, NULL, 3, &imuTaskHandle);
  xTaskCreate(lcdTask, "LCD Task", 2048, NULL, 2, &lcdTaskHandle);
  xTaskCreate(ledTask, "LED Task", 2048, NULL, 1, &ledTaskHandle);

  ticker.attach_ms(40, wakeUpTasks);
}

// loop 함수
void loop() {
  handleSwitches();
}
