#include <Wire.h>
#include <MPU9250.h>
#include <LCDI2C_Multilingual.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Ticker.h>
#include "BluetoothSerial.h"  // Bluetooth Serial 라이브러리 추가

MPU9250 mpu;
LCDI2C_Symbols lcd(0x27, 16, 2);  // I2C 주소 0x27의 16x2 LCD

int ledPin = 2;  // LED 핀 번호 
int pwmChannel = 0;  // PWM 채널
int pwmValue = 0;  // PWM 값 (0~255)
bool increasing = true;  // PWM 증가/감소 플래그

int selectButtonPin = 14;  // 선택 스위치 핀 번호
int zeroButtonPin = 17;     // 영점 조정 스위치 핀 번호
int selectButtonState = HIGH;  // 선택 스위치의 현재 상태
int zeroButtonState = HIGH;    // 영점 조정 스위치의 현재 상태
int lastSelectButtonState = HIGH;  // 선택 스위치의 마지막 상태
int lastZeroButtonState = HIGH;    // 영점 조정 스위치의 마지막 상태
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;  // 디바운스 시간 (50ms)

// 영점 조정용 변수
float rollOffset = 0;
float pitchOffset = 0;
float yawOffset = 0;

// 현재 선택된 값 (0: Roll, 1: Pitch, 2: Yaw)
int selectedValue = 0;

// Ticker 객체 생성
Ticker ticker;
BluetoothSerial SerialBT;  // Bluetooth Serial 객체

TaskHandle_t imuTaskHandle = NULL;
TaskHandle_t lcdTaskHandle = NULL;
TaskHandle_t ledTaskHandle = NULL;

// 타이머 인터럽트로 호출될 task
void wakeUpTasks() {
  // 각 Task에 Notification 보내기 (각 Task 깨우기)
  if (imuTaskHandle != NULL) {
    xTaskNotifyGive(imuTaskHandle);
  }
  if (lcdTaskHandle != NULL) {
    xTaskNotifyGive(lcdTaskHandle);
  }
  if (ledTaskHandle != NULL) {
    xTaskNotifyGive(ledTaskHandle);
  }
}

// 스위치 입력 처리 함수
void handleSwitches() {
  // 값 선택 스위치 처리
  int selectReading = digitalRead(selectButtonPin);

  // 디바운스 처리
  if (selectReading != lastSelectButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (selectReading != selectButtonState) {
      selectButtonState = selectReading;
      if (selectButtonState == LOW) {
        // 선택 스위치가 눌렸을 때
        selectedValue = (selectedValue + 1) % 3;  // Roll -> Pitch -> Yaw -> 다시 Roll
      }
    }
  }
  lastSelectButtonState = selectReading;

  // 영점 조정 스위치 처리
  int zeroReading = digitalRead(zeroButtonPin);
  
  if (zeroReading != lastZeroButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (zeroReading != zeroButtonState) {
      zeroButtonState = zeroReading;
      if (zeroButtonState == LOW) {
        // 영점 조정 스위치가 눌렸을 때
        switch (selectedValue) {
          case 0:
            rollOffset = mpu.getRoll();  // Roll 영점 조정
            break;
          case 1:
            pitchOffset = mpu.getPitch();  // Pitch 영점 조정
            break;
          case 2:
            yawOffset = mpu.getYaw();  // Yaw 영점 조정
            break;
        }
      }
    }
  }
  lastZeroButtonState = zeroReading;
}

// IMU 데이터 처리 TASK
void imuTask(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Task가 Notification을 받을 때까지 대기

    if (mpu.update()) {
      // Roll, Pitch, Yaw 값을 시리얼 모니터에 출력
      Serial.print("Roll: "); Serial.print(mpu.getRoll() - rollOffset, 2);
      Serial.print(", Pitch: "); Serial.print(mpu.getPitch() - pitchOffset, 2);
      Serial.print(", Yaw: "); Serial.println(mpu.getYaw() - yawOffset, 2);

      // Bluetooth로 Roll, Pitch, Yaw 값을 전송
      SerialBT.print("Roll: "); SerialBT.print(mpu.getRoll() - rollOffset, 2);
      SerialBT.print(", Pitch: "); SerialBT.print(mpu.getPitch() - pitchOffset, 2);
      SerialBT.print(", Yaw: "); SerialBT.println(mpu.getYaw() - yawOffset, 2);
    } else {
      Serial.println("IMU Update Failed");
    }
  }
}

// LCD 값 출력 TASK
void lcdTask(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Task가 Notification을 받을 때까지 대기

    lcd.clear();
    lcd.setCursor(0, 0);  // 첫 번째 줄의 시작 위치로 이동

    switch (selectedValue) {
      case 0:
        lcd.print("Roll: ");
        lcd.print(mpu.getRoll() - rollOffset, 2);  // Roll 값 표시
        break;
      case 1:
        lcd.print("Pitch: ");
        lcd.print(mpu.getPitch() - pitchOffset, 2);  // Pitch 값 표시
        break;
      case 2:
        lcd.print("Yaw: ");
        lcd.print(mpu.getYaw() - yawOffset, 2);  // Yaw 값 표시
        break;
    }

    lcd.setCursor(0, 1);  // 두 번째 줄의 시작 위치로 이동
    lcd.print("PWM: ");
    lcd.print(pwmValue);  // PWM 값 표시
  }
}

// LED 밝기 제어 TASK
void ledTask(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Task가 Notification을 받을 때까지 대기

    // PWM 값 업데이트 (LED 밝기 제어)
    if (increasing) {
      pwmValue += 5;  // PWM 값 증가
      if (pwmValue >= 255) {
        pwmValue = 255;
        increasing = false;  // 최대 밝기 도달 시 감소로 전환
      }
    } else {
      pwmValue -= 5;  // PWM 값 감소
      if (pwmValue <= 0) {
        pwmValue = 0;
        increasing = true;  // 최소 밝기 도달 시 증가로 전환
      }
    }

    // PWM으로 LED의 밝기를 제어
    ledcWrite(2, pwmValue);

    // 현재 PWM 값을 시리얼 모니터로 출력
    Serial.print("PWM Value: ");
    Serial.println(pwmValue);

    // Bluetooth로 PWM 값 전송
    SerialBT.print("PWM Value: ");
    SerialBT.println(pwmValue);
  }
}

void setup() {
  Serial.begin(115200);  // 시리얼 통신 초기화
  Wire.begin();  // I2C 통신 초기화
  pinMode(ledPin, OUTPUT);
  pinMode(selectButtonPin, INPUT_PULLUP);  // 선택 스위치 핀 설정
  pinMode(zeroButtonPin, INPUT_PULLUP);    // 영점 조정 스위치 핀 설정

  // PWM 설정
  ledcAttach(2, 5000, 8);
 
  // LCD 초기화
  lcd.init();
  lcd.backlight();

  // MPU9250 초기화
  mpu.setup(0x68);

  // Bluetooth Serial 시작
  SerialBT.begin("ESP32_BT");  // 블루투스 장치 이름 설정

  // FreeRTOS Task 생성
  xTaskCreate(imuTask, "IMU Task", 2048, NULL, 1, &imuTaskHandle);
  xTaskCreate(lcdTask, "LCD Task", 2048, NULL, 1, &lcdTaskHandle);
  xTaskCreate(ledTask, "LED Task", 2048, NULL, 1, &ledTaskHandle);

  // 40ms마다 wakeUpTasks 함수를 호출하도록 설정
  ticker.attach_ms(40, wakeUpTasks);
}

void loop() {
  // tact 스위치 입력 처리
  handleSwitches();

  // FreeRTOS가 실행, loop는 빈 상태.
}