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

float phCalibrationFactor = 7.5;
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
const int relay3Pin = 25;

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
  
  int phRawValue = analogRead(phSensorPin);
  float phVoltage = phRawValue * VREF / 4095.0;
  float ph = phCalibrationFactor - phVoltage;

  sensors.requestTemperatures(); 
  float tempC = sensors.getTempCByIndex(0); 

  Blynk.virtualWrite(V3, tempC);

  readTdsSensor(&tempC);
  displayLCD(ph, tempC, tdsValue);
  displaySerial(ph, tempC, tdsValue);
  sendToBlynk(ph, tempC, tdsValue);

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    controlRelay(input);
  }

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

void displayLCD(float ph, float temperature, float tdsValue) {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("pH  : ");
  lcd.print(ph, 1);

  lcd.setCursor(0, 1);
  lcd.print("Temp: ");
  lcd.print(temperature, 1);
  lcd.print(" C");

  lcd.setCursor(0, 2);
  lcd.print("TDS : ");
  lcd.print(tdsValue, 0);
  lcd.print(" ppm");

  delay(500);  
}

void displaySerial(float ph, float temperature, float tdsValue) {
  Serial.print("pH: ");
  Serial.println(ph);
  Serial.print("Temp: ");
  Serial.println(temperature);
  Serial.print("TDS : ");
  Serial.println(tdsValue);
}

void sendToBlynk(float ph, float temperature, float tdsValue) {
  Blynk.virtualWrite(V2, ph);
  Blynk.virtualWrite(V3, temperature);
  Blynk.virtualWrite(V4, tdsValue);
  Blynk.virtualWrite(V5, !digitalRead(relay1Pin)); 
  Blynk.virtualWrite(V6, !digitalRead(relay2Pin)); 
  Blynk.virtualWrite(V8, !digitalRead(relay3Pin)); 
}

void controlRelay(String input) {
  input.trim();
  if (input == "10") {
    digitalWrite(relay1Pin, HIGH);
  } else if (input == "11") {
    digitalWrite(relay1Pin, LOW);
  } else if (input == "30") {
    digitalWrite(relay2Pin, HIGH);
  } else if (input == "31") {
    digitalWrite(relay2Pin, LOW);
  } else if (input == "20") {
    digitalWrite(relay3Pin, HIGH);
  } else if (input == "21") {
    digitalWrite(relay3Pin, LOW);
  } else {
    Serial.println("****************************************");
  }
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
