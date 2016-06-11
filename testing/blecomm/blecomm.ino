/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
//#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include <Wire.h>
#include <Adafruit_SI1145.h>

//include <Adafruit_NeoPixel.h>

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    PIN                       Which pin on the Arduino is connected to the NeoPixels?
    NUMPIXELS                 How many NeoPixels are attached to the Arduino?
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE     1

    #define PIN                     8
    //#define PIN2                    6
 //   #define NUMPIXELS               1
        
/*=========================================================================*/



// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
 Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
uint8_t readPacketOverTime(Adafruit_BLE *ble,uint16_t time);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

// UV SENSOR

Adafruit_SI1145 uv = Adafruit_SI1145();
/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  //while (!Serial);  // required for Flora & Micro
  delay(500);

  // initialize UV sensor
  while(!uv.begin()){
    Serial.println("Didn't find Si1145");
   delay(1000);
  }
  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

   /* Wait for connection */
  while (! ble.isConnected()) {
      delay(50);
      Serial.print("waiting for BLE connection \n");
  }

  Serial.println(F("***********************"));

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("***********************"));


}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/


int n = 0;
int *numPtr = &n;

float sum = 0.0;
float* sumPointer = &sum;
int interval = 0;
int* intervalPointer = &interval;

int avgCount = 0;
boolean observe;

void loop(void)
{
  listenBLE(intervalPointer);
  updateUV(numPtr,sumPointer);
}


void updateUV(int *numPtr , float *sumPtr) {
  //Serial.println("===================");
  //Serial.print("Vis: "); Serial.println(uv.readVisible());
  //Serial.print("IR: "); Serial.println(uv.readIR());
  // the index is multiplied by 100 so to get the
  // integer index, divide by 100!

    float UVindex = uv.readUV();
    UVindex /= 100;
  // Uncomment if you have an IR LED attached to LED pin!
  //Serial.print("Prox: "); Serial.println(uv.readProx());
  // check how many samples we have
  if (*numPtr == avgCount) {
    float avgUV = (*sumPtr / *numPtr);
     char code = 'A';
    sendValues(code,avgUV);// 30 samples -> 1 min data -> send
    *numPtr = 0; 
    *sumPtr = 0.0;
  } else {
    *numPtr += 1;
    *sumPtr += (UVindex);
    if (observe){
      char code = 'O';
      sendValues(code,UVindex);
    }
  }
  Serial.print("UV: ");  Serial.println(UVindex);
  Serial.print("sum "); Serial.println(*sumPtr);
  Serial.print("n :"); Serial.println(*numPtr);
}

// duration in ms
void listenBLE(int *duration){
 if (*duration == 0){
  while(!readBLE()){
    delay(1);
  }
 }
 if (*duration > 0){
  uint8_t len = readPacketOverTime(&ble,*duration);
  if (len > 0)if(processData())Serial.println("updated SETTINGS");
 }
 
}
// possbile incoming commands
boolean readBLE(){
  uint8_t len = readPacket(&ble,BLE_READPACKET_TIMEOUT);
  if (len == 0) return false;
  Serial.println("datapacket lenght ");Serial.println(len);
 return processData();
}

boolean processData(){
   // recieved UVI PARAMETERS
  // Example P10000010
  if (packetbuffer[1]='P'){
    
    Serial.println(packetbuffer[0]);
    Serial.println(packetbuffer[1]);
    Serial.println(packetbuffer[2]);
    Serial.println(packetbuffer[3]);
    Serial.println(packetbuffer[4]);
    Serial.println(packetbuffer[5]);
    Serial.println(packetbuffer[6]);
    Serial.println(packetbuffer[7]);
    Serial.println(packetbuffer[8]);
    Serial.println(packetbuffer[9]);
    Serial.println((char)packetbuffer[6]);
    Serial.println((char)packetbuffer[7]);
    Serial.println((char)packetbuffer[8]);
    Serial.println((char)packetbuffer[9]);
    char INTERVAL_MESS[4],VALUE[4];
  
    
    INTERVAL_MESS[0]= (char)packetbuffer[2];
    INTERVAL_MESS[1]= (char)packetbuffer[3];
    INTERVAL_MESS[2]= (char)packetbuffer[4];
    INTERVAL_MESS[3]= (char)packetbuffer[5];
    Serial.println(INTERVAL_MESS);
    interval = getInteger(INTERVAL_MESS);
    Serial.println("interval set to ");Serial.println(interval);
    memset (INTERVAL_MESS,0,4);
    INTERVAL_MESS[0]= (char)packetbuffer[6];
    INTERVAL_MESS[1]= (char)packetbuffer[7];
    INTERVAL_MESS[2]= (char)packetbuffer[8];
    INTERVAL_MESS[3]= (char)packetbuffer[9]; 
    Serial.println(INTERVAL_MESS);
    avgCount = getInteger(INTERVAL_MESS);
      Serial.println("avg set to ");Serial.println(avgCount);
      return true;
  }
  if (packetbuffer[1] == 'O'){
    if (packetbuffer[2] == '1') observe = true; else observe = false;
     return true;
  }
  return false;
  }

// send data
boolean sendValues(char code, float value){
  /*
  const char* finalCode = (const char*)code;
  char buf[5];
  Serial.println(value);
dtostrf(value, 4, 3, buf);
Serial.print("egal");
Serial.println(buf);
 char* data = appendCharArrayToChar(buf,finalCode);
 Serial.println(data);*/
 // WORKAROUND
  ble.print(code);
  ble.print(value);
  Serial.print("sent:"); 
Serial.print(code);
  Serial.println(value);
}

// no check for index out of bound
int getInteger(char input[]){
  char output[4];
      if (input[0] != '0'){
        output[0] = input[0];
        output[1] = input[1];
        output[2] = input[2];
        output[3] = input[3];
      } else if (input[1] != '0'){
        output[0] = input[1];
        output[1] = input[2];
        output[2] = input[3];
      } else if (input[2] != '0'){
        output[0] = input[2];
        output[1] = input[3];
      } else output[0] = input[3];
      Serial.println(output);
      return atoi(output);
  }

char* appendCharArrayToChar(char* array,  const char* a){
    size_t len = strlen(array);

    char ret[len+2];

strcpy(ret,a);
strcat( ret,array);
 return ret;
}
  



