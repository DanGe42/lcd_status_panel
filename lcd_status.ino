#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_LiquidCrystal.h"
#include "RTClib.h"

#define TZ_PDT -7
#define TZ_PST -8
#define TZ_OFFSET TZ_PDT

#define BRIGHTNESS_ADJUST_PIN A0
#define LCD_BRIGHTNESS_PIN 3

Adafruit_BME280 bme; // I2C
// Connect via i2c, default address #0 (A0-A2 not jumpered)
Adafruit_LiquidCrystal lcd(0);
RTC_DS1307 rtc;

void setup() {
  pinMode(LCD_BRIGHTNESS_PIN, OUTPUT);
  
  lcd.clear();
  Serial.begin(9600);
  
  // set up the LCD's number of rows and columns:
  lcd.begin(16, 2);
  lcd.setBacklight(HIGH);
  lcd.print("Starting...");
  lcd.setCursor(0, 0);

  // Set up LCD brightness control
  pinMode(LCD_BRIGHTNESS_PIN, OUTPUT);

  if (!bme.begin()) {
    lcd.clear();
    lcd.print("ERROR: BME280");
    while (1);
  }

  if (!rtc.begin()) {
    lcd.clear();
    lcd.print("ERROR: RTC");
    while (1);
  }

  if (!rtc.isrunning()) {
    lcd.clear();
    lcd.print("ERROR: RTC");
    lcd.setCursor(0, 1);
    lcd.print("Set date and time");
    while (1);
  }
}

void loop() {
  // Adjust brightness
  int brightnessSetting = analogRead(BRIGHTNESS_ADJUST_PIN);
  analogWrite(LCD_BRIGHTNESS_PIN, analogToPWM(brightnessSetting));
  
  lcd.setCursor(0, 0);

  DateTime now = withTimeZoneOffset(rtc.now(), TZ_OFFSET);
  lcd.print(formatDateTime(now));

  lcd.setCursor(0, 1);
  printBmeData(lcd, bme);

  delay(1000);
}

// TODO: rewrite this with char[] for memory-efficiency
String formatDateTime(DateTime dt) {
  String dtString = formatInt(dt.month());
  dtString += '/';
  dtString += formatInt(dt.month());
  dtString += ' ';
  dtString += formatInt(dt.hour());
  dtString += ':';
  dtString += formatInt(dt.minute());
  return dtString;
}

DateTime withTimeZoneOffset(DateTime dt, int hours) {
  return dt + TimeSpan(0, hours, 0, 0);
}

void printBmeData(Adafruit_LiquidCrystal lcd, Adafruit_BME280 bme) {
  float tempF = celsiusToFahrenheit(bme.readTemperature());
  float pressureAtm = pascalsToAtm(bme.readPressure());
  float humidity = bme.readHumidity();

  lcd.print(formatBmeData(tempF, pressureAtm, humidity));
}

float celsiusToFahrenheit(float celsius) {
  return celsius * 9 / 5 + 32;
}

float pascalsToAtm(float pascals) {
  return pascals / 101325.0;
}

// TODO: some day, just page-flip the LCD so we can display more data
String formatBmeData(float tempF, float pressureAtm, float humidity) {
  String bmeString = String((int) tempF, DEC);
  bmeString += " F ";
  bmeString += String(pressureAtm, 2);
  bmeString += " at ";
  bmeString += String((int) humidity, DEC);
  bmeString += '%';
  return bmeString;
}

String formatInt(int val) {
  String valStr = String(val, DEC);
  if (valStr.length() == 1) {
    return "0" + valStr;
  } else {
    return valStr;
  }
}

// Analog input comes in between 0 and 1023. This function scales to
// the range between 0 and 255, fit for PWM output.
int analogToPWM(int analogInput) {
  return (int) (analogInput / 1024.0 * 256);
}
