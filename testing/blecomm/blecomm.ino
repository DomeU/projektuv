#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include <Wire.h>
#include <Adafruit_SI1145.h>
/*=========================================================================*/

    #define FACTORYRESET_ENABLE     1

    #define PIN                     8
/*=========================================================================*/



// Create the bluefruit object

 Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

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
   Serial.println(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      Serial.println(F("Couldn't factory reset"));
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

    float UVindex = uv.readUV();
    UVindex /= 100;
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
  if (packetbuffer[1] == 'P'){
    
    char BUF[4];
    // get INTERVAL , first 4 chars
    BUF[0]= (char)packetbuffer[2];
    BUF[1]= (char)packetbuffer[3];
    BUF[2]= (char)packetbuffer[4];
    BUF[3]= (char)packetbuffer[5];

    interval = getInteger(BUF);
    Serial.println("interval set to ");Serial.println(interval);
    memset (BUF,0,4);
    // get AVGCOUNT
    BUF[0]= (char)packetbuffer[6];
    BUF[1]= (char)packetbuffer[7];
    BUF[2]= (char)packetbuffer[8];
    BUF[3]= (char)packetbuffer[9]; 
    avgCount = getInteger(BUF);
      Serial.println("avg set to ");Serial.println(avgCount);
      // reset pointers
       n = 0; 
       sum = 0.0;
    memset (packetbuffer,0,BUFSIZE);
    
     return true;
  }
  
  // OBSERVE 
  // EXAMPLE O1
  if (packetbuffer[1] == 'O'){
    if (packetbuffer[2] == '1') {
      observe = true;
      Serial.println("observe: true");
      return true;
    } else {
      observe = false;
      Serial.println("observe: false");
      return true;
    }    
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
// index offset 2 because of ! and P/O
int getInteger(char input[]){
  Serial.println("in method");
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
      return atoi(output);
  }

char* appendCharArrayToChar(char* array,  const char* a){
    size_t len = strlen(array);
    char ret[len+2];
    strcpy(ret,a);
    strcat( ret,array);
    return ret;
}
  



