#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#define leds 24
#define PIN 4

Adafruit_NeoPixel strip = Adafruit_NeoPixel(leds, PIN, NEO_GRB);

int offset = 0;
int pix;
int speed = 1000;

void setup()
{
  strip.begin();
  strip.setBrightness(100);
  strip.clear();
  Serial.begin(115200);
}

void loop() 
{
  uint32_t color = strip.Color(0,0,255);
    for(int i = 0; i < leds; ++i){
      strip.setPixelColor(i, color);
    }
    strip.show();
    delay(speed);
  offset++;
}
