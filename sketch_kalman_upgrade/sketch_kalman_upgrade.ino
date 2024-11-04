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
int selectButtonPin = 15, zeroButtonPin = 4;
int lastSelectButtonState = HIGH, lastZeroButtonState = HIGH;
unsigned long lastDebounceTime = 0, debounceDelay = 50;
int pwmValue = 0;
bool increasing = true;

float rollOffset = 0, pitchOffset = 0, yawOffset = 0;
int selectedValue = 0;

// 칼만 필터를 위한 변수
float Q_angle = 0.001;  // 각도에 대한 프로세스 노이즈 공분산
float Q_bias = 0.003;   // 바이어스에 대한 프로세스 노이즈 공분산
float R_measure = 0.03; // 측정 노이즈 공분산

// 각도와 바이어스를 저장할 변수
float rollAngle = 0, rollBias = 0;
float pitchAngle = 0, pitchBias = 0;
float yawAngle = 0, yawBias = 0;

// 칼만 필터 상태 공분산
float rollP[2][2] = {{0, 0}, {0, 0}};
float pitchP[2][2] = {{0, 0}, {0, 0}};
float yawP[2][2] = {{0, 0}, {0, 0}};

// FreeRTOS 태스크 핸들
TaskHandle_t imuTaskHandle = NULL, lcdTaskHandle = NULL, ledTaskHandle = NULL;
Ticker ticker;

// 칼만 필터 함수
float kalmanFilter(float newAngle, float newRate, float dt, float &angle, float &bias, float P[2][2]) {
  // Predict
  float rate = newRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Update
  float S = P[0][0] + R_measure;
  float K[2]; // 칼만 이득
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  float y = newAngle - angle; // Angle difference
  angle += K[0] * y;
  bias += K[1] * y;

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

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
        if (selectedValue == 0) rollOffset = rollAngle;
        else if (selectedValue == 1) pitchOffset = pitchAngle;
        else if (selectedValue == 2) yawOffset = yawAngle;
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

    // 보정된 롤, 피치, 요 값을 계산
    rollAngle = kalmanFilter(mpu.getRoll(), mpu.getGyroX(), dt, rollAngle, rollBias, rollP);
    pitchAngle = kalmanFilter(mpu.getPitch(), mpu.getGyroY(), dt, pitchAngle, pitchBias, pitchP);
    yawAngle = kalmanFilter(mpu.getYaw(), mpu.getGyroZ(), dt, yawAngle, yawBias, yawP);

    Serial.printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", rollAngle - rollOffset, pitchAngle - pitchOffset, yawAngle - yawOffset);
    SerialBT.printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", rollAngle - rollOffset, pitchAngle - pitchOffset, yawAngle - yawOffset);
  } else {
    Serial.println("IMU Update Failed");
  }
}

// LCD 업데이트 함수
void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);

  switch (selectedValue) {
    case 0: lcd.print("Roll: "), lcd.print(rollAngle - rollOffset, 2); break;
    case 1: lcd.print("Pitch: "), lcd.print(pitchAngle - pitchOffset, 2); break;
    case 2: lcd.print("Yaw: "), lcd.print(yawAngle - yawOffset, 2); break;
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
