#include <Wire.h>
#include <MPU9250.h>
#include <LCDI2C_Multilingual.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Ticker.h>

MPU9250 mpu;
LCDI2C_Symbols lcd(0x27, 16, 2);  // I2C 주소 0x27의 16x2 LCD

int ledPin = 2;  // LED 핀 번호 
int pwmChannel = 0;  // PWM 채널
int pwmValue = 0;  // PWM 값 (0~255)
bool increasing = true;  // PWM 증가/감소 플래그

int switchPin1 = 14; // 로커 스위치 1번 핀 (NO)
int switchPin2 = 27; // 로커 스위치 2번 핀 (NC)

Ticker ticker;

TaskHandle_t imuTaskHandle = NULL;
TaskHandle_t lcdTaskHandle = NULL;
TaskHandle_t ledTaskHandle = NULL;

int displayMode = 0; // 0: Roll, 1: Pitch, 2: Yaw

// 타이머 인터럽트로 호출될 함수
void wakeUpTasks() {
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

// IMU 데이터 처리 TASK
void imuTask(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Task가 Notification을 받을 때까지 대기

    if (mpu.update()) {
      // Roll, Pitch, Yaw 값을 시리얼 모니터에 출력
      Serial.print("Roll: "); Serial.print(mpu.getRoll(), 2);
      Serial.print(", Pitch: "); Serial.print(mpu.getPitch(), 2);
      Serial.print(", Yaw: "); Serial.println(mpu.getYaw(), 2);
    } else {
      Serial.println("Failed to read data from MPU9250.");
    }
  }
}

// LCD에 선택된 각도 표시 TASK
void lcdTask(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Task가 Notification을 받을 때까지 대기

    // 스위치 상태를 읽어 displayMode 설정
    int switchState1 = digitalRead(switchPin1);
    int switchState2 = digitalRead(switchPin2);

    if (switchState1 == LOW) {
      displayMode = 0;  // Roll 모드
    } else if (switchState2 == LOW) {
      displayMode = 1;  // Pitch 모드
    } else {
      displayMode = 2;  // Yaw 모드
    }

    lcd.clear();
    lcd.setCursor(0, 0);

    if (displayMode == 0) {
      lcd.print("Roll: "); lcd.print(mpu.getRoll(), 2);
    } else if (displayMode == 1) {
      lcd.print("Pitch: "); lcd.print(mpu.getPitch(), 2);
    } else if (displayMode == 2) {
      lcd.print("Yaw: "); lcd.print(mpu.getYaw(), 2);
    }
  }
}

// LED 밝기 제어 TASK
void ledTask(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Task가 Notification을 받을 때까지 대기

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
  }
}

void setup() {
  Serial.begin(115200);  // 시리얼 통신 초기화
  Wire.begin();  // I2C 통신 초기화
  pinMode(ledPin, OUTPUT);

  // PWM 설정
  ledcAttach(2, 5000, 8);  // 채널 0, 5kHz 주파수, 8비트 해상도

  // 스위치 핀 초기화
  pinMode(switchPin1, INPUT_PULLUP); // 내부 풀업 저항 활성화
  pinMode(switchPin2, INPUT_PULLUP); // 내부 풀업 저항 활성화

  // LCD 초기화
  lcd.init();
  lcd.backlight();

  // MPU9250 초기화
  if (!mpu.setup(0x68)) {  // I2C 주소 0x68로 초기화 시도
    Serial.println("MPU9250 initialization failed. Please check wiring or I2C address.");
    while (1);  // 무한 루프 (초기화 실패 시 프로그램 멈춤)
  } else {
    Serial.println("MPU9250 initialized successfully!");
  }

  // FreeRTOS Task 생성
  xTaskCreate(imuTask, "IMU Task", 2048, NULL, 1, &imuTaskHandle);
  xTaskCreate(lcdTask, "LCD Task", 2048, NULL, 1, &lcdTaskHandle);
  xTaskCreate(ledTask, "LED Task", 2048, NULL, 1, &ledTaskHandle);

  // 40ms마다 wakeUpTasks 함수를 호출하도록 설정
  ticker.attach_ms(4000, wakeUpTasks);
}

void loop() {
  // FreeRTOS가 실행되므로 loop는 빈 상태로 둡니다.
}
