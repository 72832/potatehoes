#include <Wire.h> #include <LiquidCrystal_I2C.h> #include <FastLED.h>#define NUM_LEDS 30#define DATA_PIN 6LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line displayCRGB leds[NUM_LEDS];void setup() {  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  }void loop() {  lcd.init();  // Print a message to the LCD.  lcd.backlight();  lcd.setCursor(3,0);  lcd.print("Hello, world!");  lcd.setCursor(2,1);  lcd.print("Ywrobot Arduino!");   lcd.setCursor(0,2);  lcd.print("Arduino LCM IIC 2004");   lcd.setCursor(2,3);  lcd.print("Power By Ec-yuan!");  for (int dot = 0; dot < NUM_LEDS; dot++) {    leds[dot] = CRGB::Blue;    FastLED.show();    // clear this led for the next time around the loop    leds[dot] = CRGB::Blue;    delay(30);  }    for (int dot = 0; dot < NUM_LEDS; dot++) {    leds[dot] = CRGB::Green;    FastLED.show();    // clear this led for the next time around the loop    leds[dot] = CRGB::Green;    delay(30);  }}