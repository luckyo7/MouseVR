#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("ESP32-S3 booted");

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init();
  tft.setRotation(0);

  tft.fillScreen(TFT_BLUE);
  delay(1000);
  tft.fillScreen(TFT_RED);
  delay(1000);
  tft.fillScreen(TFT_GREEN);
  delay(1000);

  tft.fillScreen(0x04FF);
  tft.setTextColor(TFT_WHITE, 0x04FF);
  tft.drawString("Hello, Waveshare!", 30, 40, 2);
}

int number = 0;

void loop() {
  String displayNumber = String(number);
  if (displayNumber.length() == 1) {
    displayNumber = "0" + displayNumber;
  }

  Serial.println("Number: " + displayNumber);
  tft.drawString(displayNumber, 55, 80, 6);

  number++;
  if (number > 99) number = 0;

  delay(1000);
}
