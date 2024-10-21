#include <Wire.h>
#include <MPU9250.h>
#include <LCDI2C_Multilingual.h>
#include <BluetoothSerial.h>
#include <SimpleKalmanFilter.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Ticker.h>

MPU9250 mpu;
LCDI2C_Symbols lcd(0x27, 16, 2);
BluetoothSerial SerialBT;

int ledPin = 2;
int selectButtonPin = 15, zeroButtonPin = 4;
int lastSelectButtonState = HIGH, lastZeroButtonState = HIGH;
unsigned long lastDebounceTime = 0, debounceDelay = 50;

float rollOffset = 0, pitchOffset = 0, yawOffset = 0;
int selectedValue = 0, pwmValue = 0;
bool increasing = true;

SimpleKalmanFilter rollKalmanFilter(2, 2, 0.01);
SimpleKalmanFilter pitchKalmanFilter(2, 2, 0.01);
SimpleKalmanFilter yawKalmanFilter(2, 2, 0.01);

TaskHandle_t imuTaskHandle = NULL, lcdTaskHandle = NULL, ledTaskHandle = NULL;
Ticker ticker;

void wakeUpTasks() {
  if (imuTaskHandle != NULL) xTaskNotifyGive(imuTaskHandle);
  if (lcdTaskHandle != NULL) xTaskNotifyGive(lcdTaskHandle);
  if (ledTaskHandle != NULL) xTaskNotifyGive(ledTaskHandle);
}

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

void readIMUData() {
  if (mpu.update()) {
    float roll = rollKalmanFilter.updateEstimate(mpu.getRoll()) - rollOffset;
    float pitch = pitchKalmanFilter.updateEstimate(mpu.getPitch()) - pitchOffset;
    float yaw = yawKalmanFilter.updateEstimate(mpu.getYaw()) - yawOffset;

    Serial.printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll, pitch, yaw);
    SerialBT.printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll, pitch, yaw);
  } else {
    Serial.println("IMU Update Failed");
  }
}

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

void imuTask(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    readIMUData();
  }
}

void lcdTask(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    updateLCD();
  }
}

void ledTask(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    updateLED();
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
  xTaskCreate(lcdTask, "LCD Task", 2048, NULL, 1, &lcdTaskHandle);
  xTaskCreate(ledTask, "LED Task", 2048, NULL, 1, &ledTaskHandle);

  ticker.attach_ms(40, wakeUpTasks);
}

void loop() {
  handleSwitches();
}
