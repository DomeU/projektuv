/*************************************************** 
  This is a library for the Si1145 UV/IR/Visible Light Sensor

  Designed specifically to work with the Si1145 sensor in the
  adafruit shop
  ----> https://www.adafruit.com/products/1777

  These sensors use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_SI1145.h>

#include <Adafruit_NeoPixel.h>


// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            8

Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, PIN, NEO_GRB + NEO_KHZ800);

Adafruit_SI1145 uv = Adafruit_SI1145();

void setup() {
  Serial.begin(9600);
  
  Serial.println("Adafruit SI1145 test");
    strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  
  if (! uv.begin()) {
    Serial.println("Didn't find Si1145");
    while (1);
  }

  Serial.println("OK!");
}

void loop() {
  Serial.println("===================");
  Serial.print("Vis: "); Serial.println(uv.readVisible());
  Serial.print("IR: "); Serial.println(uv.readIR());
  
  // Uncomment if you have an IR LED attached to LED pin!
  //Serial.print("Prox: "); Serial.println(uv.readProx());

  float UVindex = uv.readUV();
  // the index is multiplied by 100 so to get the
  // integer index, divide by 100!
  UVindex /= 100.0;  
  Serial.print("UV: ");  Serial.println(UVindex);
  if (UVindex < 3 && 0 < UVindex){ // unter 3 grün
    colorWipe(strip.Color(0,255,0),300);
  }
   if (UVindex < 6 && 3 < UVindex){ // unter 6 gelb
    colorWipe(strip.Color(255,255,0),300);
  }
   if (UVindex < 8 && 6 < UVindex){ // unter 8 orange
    colorWipe(strip.Color(255,69,0),300);
  }
   if (UVindex < 11 && 8 < UVindex){ // unter 11 rot
    colorWipe(strip.Color(255,0,0),300);
  }
   if (11 < UVindex){ // über 11 lila
    colorWipe(strip.Color(148,0,211),300);
  }
 
  
  

  delay(2000);
}

 
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}
