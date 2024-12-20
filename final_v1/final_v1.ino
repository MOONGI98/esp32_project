#include <Wire.h>
#include <MPU9250.h>
#include <LCDI2C_Multilingual.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Ticker.h>
#include "BluetoothSerial.h"
#include <SimpleKalmanFilter.h>

// Bluetoonth 영점 조절 제어 전역변수 11/24 
String received_command = "R";    


MPU9250 mpu;
LCDI2C_Symbols lcd(0x27, 16, 2);

int ledPin = 2;
int pwmChannel = 0;
int pwmValue = 0;
bool increasing = true;

int selectButtonPin = 14;
int zeroButtonPin = 27;
int selectButtonState = HIGH;
int zeroButtonState = HIGH;
int lastSelectButtonState = HIGH;
int lastZeroButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

float rollOffset = 0;
float pitchOffset = 0;
float yawOffset = 0;

int selectedValue = 0;

// Kalman Filter 객체 생성 (측정 불확실성, 프로세스 불확실성, 측정 잡음) kalman filter 수정 필요
SimpleKalmanFilter rollKalmanFilter(2, 2, 0.01);
SimpleKalmanFilter pitchKalmanFilter(2, 2, 0.01);
SimpleKalmanFilter yawKalmanFilter(2, 2, 0.01);

Ticker ticker;
BluetoothSerial SerialBT;

TaskHandle_t imuTaskHandle = NULL;
TaskHandle_t lcdTaskHandle = NULL;
TaskHandle_t ledTaskHandle = NULL;

void wakeUpTasks() {
  if (imuTaskHandle != NULL) xTaskNotifyGive(imuTaskHandle);
  if (lcdTaskHandle != NULL) xTaskNotifyGive(lcdTaskHandle);
  if (ledTaskHandle != NULL) xTaskNotifyGive(ledTaskHandle);
}

void handleSwitches() {
  int selectReading = digitalRead(selectButtonPin);

  if (selectReading != lastSelectButtonState) lastDebounceTime = millis();
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (selectReading != selectButtonState) {
      selectButtonState = selectReading;
      if (selectButtonState == LOW) {
        selectedValue = (selectedValue + 1) % 3;
      }
    }
  }
  lastSelectButtonState = selectReading;

  int zeroReading = digitalRead(zeroButtonPin);

  if (zeroReading != lastZeroButtonState) lastDebounceTime = millis();
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (zeroReading != zeroButtonState) {
      zeroButtonState = zeroReading;
      if (zeroButtonState == LOW) {
        switch (selectedValue) {
          case 0: rollOffset = mpu.getRoll(); break;
          case 1: pitchOffset = mpu.getPitch(); break;
          case 2: yawOffset = mpu.getYaw(); break;
        }
      }
    }
  }
  lastZeroButtonState = zeroReading;
}

void imuTask(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (mpu.update()) {
      float rawRoll = mpu.getRoll();
      float rawPitch = mpu.getPitch();
      float rawYaw = mpu.getYaw();

      float filteredRoll = rollKalmanFilter.updateEstimate(rawRoll);
      float filteredPitch = pitchKalmanFilter.updateEstimate(rawPitch);
      float filteredYaw = yawKalmanFilter.updateEstimate(rawYaw);

      filteredRoll -= rollOffset;
      filteredPitch -= pitchOffset;
      filteredYaw -= yawOffset;

      Serial.print("Roll: "); Serial.print(filteredRoll, 2);
      Serial.print(", Pitch: "); Serial.print(filteredPitch, 2);
      Serial.print(", Yaw: "); Serial.println(filteredYaw, 2);

      SerialBT.print("Roll: "); SerialBT.print(filteredRoll, 2); SerialBT.println("");
      SerialBT.print(", Pitch: "); SerialBT.print(filteredPitch, 2); SerialBT.println("");
      SerialBT.print(", Yaw: "); SerialBT.println(filteredYaw, 2); SerialBT.println("");
    } else {
      Serial.println("IMU Update Failed");
    }
  }
}

void lcdTask(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    lcd.clear();
    lcd.setCursor(0, 0);

    switch (selectedValue) {
      case 0: 
          lcd.print("Roll: "); 
          lcd.print(rollKalmanFilter.updateEstimate(mpu.getRoll()) - rollOffset, 2); 
          break;
      case 1: 
          lcd.print("Pitch: "); 
          lcd.print(pitchKalmanFilter.updateEstimate(mpu.getPitch()) - pitchOffset, 2); 
          break;
      case 2: 
          lcd.print("Yaw: "); 
          lcd.print(yawKalmanFilter.updateEstimate(mpu.getYaw()) - yawOffset, 2); 
          break;
 }

    lcd.setCursor(0, 1);
    lcd.print("PWM: ");
    lcd.print(pwmValue);
  }
}

void ledTask(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

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

    Serial.print("PWM Value: ");
    Serial.println(pwmValue);

    SerialBT.print("PWM Value: ");
    SerialBT.println(pwmValue);
  }
}

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
  xTaskCreate(lcdTask, "LCD Task", 2048, NULL, 2, &lcdTaskHandle);
  xTaskCreate(ledTask, "LED Task", 2048, NULL, 3, &ledTaskHandle);

  ticker.attach_ms(40, wakeUpTasks);
}

void loop() {
  handleSwitches();

   // Bluetooth 모니터에 명령줄을 입력할 때 11.24
  if (SerialBT.available()) {
    received_command = SerialBT.readStringUntil('\n');
    received_command.trim();  // 공백제거 
    Serial.println("블루투스 모니터에 입력한 command: " + received_command); //Serial 모니터에서 확인가능
  }
  // command 값이 R일 때 Offset 값이 들어가야 하는 부분  11.24
  if (received_command == "R") {
    rollOffset = mpu.getRoll(); 
    pitchOffset = mpu.getPitch(); // 
    yawOffset = mpu.getYaw(); // 
    SerialBT.println("Reset Completed");
    received_command = "";  // command 리셋
  }

}