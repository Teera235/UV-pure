#define BLYNK_TEMPLATE_ID "TMPL6-JUaRrfb"
#define BLYNK_TEMPLATE_NAME "UV pure"

#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

char auth[] = "nll1tEz840rKwsUbJ0-SAe-jebSYhAPT";
char ssid[] = "Dorm2911 2G";
char pass[] = "29112911";

#define ONE_WIRE_BUS 2 
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

const int lcd_I2C_address = 0x27;
const int waterLevelPin = 32;
const int phSensorPin = 33;
const int tdsSensorPin = 35;

#define SDA_PIN 21
#define SCL_PIN 22

LiquidCrystal_I2C lcd(lcd_I2C_address, 20, 4);

#define VREF 3.3  
#define SCOUNT 30
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float tdsValue = 0;
float averageVoltage = 0;

const int relay1Pin = 26;
const int relay2Pin = 27;
const int relay3Pin = 25;
unsigned long pumpStartTime = 0;
unsigned long valveOpenTime = 0;
bool isWaterHigh = false;
bool isWaterLow = false;
bool isValveOpen = false;
String currentState = "";

const float C = 25.85; 
const float m = -6.80; 

void setup(void) {
  Serial.begin(9600);
  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.init();
  lcd.backlight();
  pinMode(waterLevelPin, INPUT);
  pinMode(phSensorPin, INPUT);
  pinMode(tdsSensorPin, INPUT);
  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);
  pinMode(relay3Pin, OUTPUT); 
  digitalWrite(relay1Pin, LOW);
  digitalWrite(relay2Pin, HIGH);
  digitalWrite(relay3Pin, HIGH);  
  sensors.begin();

  Blynk.begin(auth, ssid, pass);
}

void loop(void) {
  Blynk.run();

  int waterLevel = digitalRead(waterLevelPin);

  if (!isWaterHigh && waterLevel == LOW) {
    digitalWrite(relay1Pin, LOW);
    digitalWrite(relay2Pin, HIGH);
    digitalWrite(relay3Pin, HIGH); 
    isWaterLow = true;
    isWaterHigh = false;
    isValveOpen = false;
    currentState = "เติมน้ำเข้าถัง";
  } else if (!isWaterHigh && waterLevel == HIGH) {
    digitalWrite(relay1Pin, HIGH);
    digitalWrite(relay2Pin, HIGH);
    digitalWrite(relay3Pin, LOW); 
    pumpStartTime = millis();
    isWaterHigh = true;
    isWaterLow = false;
    isValveOpen = false;
    currentState = "บ่ม 3 นาที";
  }

  if (isWaterHigh && millis() - pumpStartTime > 180000 && !isValveOpen) {
    digitalWrite(relay2Pin, LOW);
    digitalWrite(relay3Pin, HIGH);
    valveOpenTime = millis();
    isValveOpen = true;
    currentState = "ระบายน้ำ 1 นาที";
  }

  if (isValveOpen && millis() - valveOpenTime > 60000) {
    digitalWrite(relay2Pin, HIGH);
    isValveOpen = false;
    isWaterHigh = false;  
    currentState = "";
  }

  float ph = readPhSensor();
  
  sensors.requestTemperatures(); 
  float tempC = sensors.getTempCByIndex(0); 

  Blynk.virtualWrite(V3, tempC);

  readTdsSensor(&tempC);
  displayLCD(waterLevel, ph, tempC, tdsValue);
  displaySerial(waterLevel, ph, tempC, tdsValue);
  sendToBlynk(waterLevel, ph, tempC, tdsValue);

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

float readPhSensor() {
  long phTot = 0;
  for(int x = 0; x < 10; x++) {
    phTot += analogRead(phSensorPin);
    delay(10);
  }
  float phAvg = phTot / 10;
  float phVoltage = phAvg * (VREF / 4095.0);
  return phVoltage * m + C;
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
  Serial.print("State: ");
  Serial.println(currentState);
}

void sendToBlynk(int waterLevel, float ph, float temperature, float tdsValue) {
  Blynk.virtualWrite(V0, waterLevel == HIGH ? "High" : "Low");
  Blynk.virtualWrite(V1, waterLevel == HIGH ? "High" : "Low");

  Blynk.virtualWrite(V2, ph);
  Blynk.virtualWrite(V3, temperature);
  Blynk.virtualWrite(V4, tdsValue);

  Blynk.virtualWrite(V5, !digitalRead(relay1Pin)); 
  Blynk.virtualWrite(V6, !digitalRead(relay2Pin)); 
  Blynk.virtualWrite(V8, !digitalRead(relay3Pin)); 

  Blynk.virtualWrite(V7, currentState);
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