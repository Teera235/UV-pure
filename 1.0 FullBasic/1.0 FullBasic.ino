#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int lcd_I2C_address = 0x27;  
const int waterLevelPin = 32;
const int phSensorPin = 33;
const int temperaturePin = 34;
const int tdsSensorPin = 35;

#define SDA_PIN 21
#define SCL_PIN 22

LiquidCrystal_I2C lcd(lcd_I2C_address, 20, 4);

float phCalibrationFactor = 3.5;
float temperatureCalibrationFactor = 0.0;

#define VREF 3.3  
#define SCOUNT 30
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float tdsValue = 0;
float averageVoltage = 0;

const int relay1Pin = 26;
const int relay2Pin = 27;
unsigned long pumpStartTime = 0;
unsigned long valveOpenTime = 0;
bool isWaterHigh = false;
bool isWaterLow = false;
bool isValveOpen = false;

void setup() {
  Serial.begin(9600);
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  pinMode(waterLevelPin, INPUT);
  pinMode(phSensorPin, INPUT);
  pinMode(temperaturePin, INPUT);
  pinMode(tdsSensorPin, INPUT);
  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);
  digitalWrite(relay1Pin, LOW);
  digitalWrite(relay2Pin, HIGH);
}

void loop() {
  int waterLevel = digitalRead(waterLevelPin);

  if (waterLevel == LOW) {
    digitalWrite(relay1Pin, HIGH); // เปิดปั้มน้ำ
    digitalWrite(relay2Pin, LOW);  // ปิดวาว
    isWaterLow = true;
    isWaterHigh = false;
    isValveOpen = false;
  } else if (waterLevel == HIGH && !isWaterHigh) {
    digitalWrite(relay1Pin, LOW);  // ปิดปั้มน้ำ
    digitalWrite(relay2Pin, LOW);  // ปิดวาว
    pumpStartTime = millis();
    isWaterHigh = true;
    isWaterLow = false;
    isValveOpen = false;
  }

  if (isWaterHigh && millis() - pumpStartTime > 180000 && !isValveOpen) {
    digitalWrite(relay2Pin, HIGH); // เปิดว้าว
    valveOpenTime = millis();
    isValveOpen = true;
  }

  if (isValveOpen && millis() - valveOpenTime > 60000) {
    digitalWrite(relay2Pin, LOW);  // ปิดวาว
    isValveOpen = false;
    isWaterHigh = false;
  }

  int phRawValue = analogRead(phSensorPin);
  float phVoltage = phRawValue * VREF / 4095.0;
  float ph = phCalibrationFactor - phVoltage;
  int temperatureRawValue = analogRead(temperaturePin);
  float temperature = temperatureRawValue * temperatureCalibrationFactor;
  readTdsSensor(&temperature);
  displayLCD(waterLevel, ph, temperature, tdsValue);
  displaySerial(waterLevel, ph, temperature, tdsValue);

  delay(1000);
}

void readTdsSensor(float* currentTemp) {
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(tdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4095.0;
    float compensationCoefficient = 1.0 + 0.02 * (*currentTemp - 25.0);
    float compensationVolatge = averageVoltage / compensationCoefficient;
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5;
  }
}

void displayLCD(int waterLevel, float ph, float temperature, float tdsValue) {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Water Level: ");
  lcd.print(waterLevel == HIGH ? "High" : "Low");

  lcd.setCursor(0, 1);
  lcd.print("pH  : ");
  lcd.print(ph, 1);

  lcd.setCursor(0, 2);
  lcd.print("Temp: ");
  lcd.print(temperature, 1);
  lcd.print(" C");

  lcd.setCursor(0, 3);
  lcd.print("TDS : ");
  lcd.print(tdsValue, 0);
  lcd.print(" ppm");

  delay(3000);  
}

void displaySerial(int waterLevel, float ph, float temperature, float tdsValue) {
  Serial.print("Water Level: ");
  Serial.println(waterLevel == HIGH ? "High" : "Low");
  Serial.print("pH: ");
  Serial.println(ph);
  Serial.print("Temp: ");
  Serial.println(temperature);
  Serial.print("TDS : ");
  Serial.println(tdsValue);
}

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}
