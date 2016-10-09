#include <Arduino.h>
#include <SPI.h>

// BLE Libs
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
// Sensor Libs
#include <Wire.h>
#include <Adafruit_SI1145.h>
// EEPROM functions
#include <EEPROM.h>
/*=========================================================================*/

    #define FACTORYRESET_ENABLE     1
// onboard LED
    #define PIN                     8
/*=========================================================================*/



// Create the bluefruit object

 Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

// defines and variables for parsing incoming data
#define PACKET_UVIPARAM_LEN             (10)
#define PACKET_UVIOPTIONS_LEN            (4)

//    READ_BUFSIZE            Size of the read buffer for incoming packets
#define READ_BUFSIZE                    (20)

/* Buffer to hold incoming characters */
uint8_t packetbuffer[READ_BUFSIZE+1];


// UV SENSOR
Adafruit_SI1145 uv = Adafruit_SI1145();
// VARIABLES AND POINTER
int num = 0;
int *numPointer = &num; // points at variable for storing how much samples were taken
double sum = 0.0;
double* sumPointer = &sum; // points at the sum of all samples added up
// Parameter
int interval = 0;
int* intervalPointer = &interval; // points at the interval between meassurements

int avgCount = 0;
// observe flags
boolean observeUV = false; // observe UV Index
boolean observeT = true; // observe Time-left
boolean observeD = true; // observe current MED

// UV specific 
double ed = 0.0; // current erythema dosis
//double *edPointer = &ed; // pointer to ED

double lastAvg = -1.0;
double lastUVI = -1.0;

int skintype = 0; // store selected skintype 

// Called after boot up, initializes connected UVI Sensor and BLE Module
void setup(void)
{
  Serial.begin(115200); // starts serial communication 
  // remove if working without serial monitor
  while (!Serial) {
   delay(500);
  };  // required for Flora & Micro
  
  delay(500);

  // initialize UV sensor
  while(!uv.begin()){
    //TODO: What can we do if sensor is not found ? (retry, set LED to specific color etc..)
    Serial.println("Didn't find Si1145");
   delay(1000);
  }

  initBLE();

}

// LOOP, beeing constantly executed while running
// TODO: maybe add some checks if everything is still connected and if not some representation of the state
void loop(void)
{
  listenBLE(intervalPointer);
  updateUV(numPointer,sumPointer);
}

// BLE COMMUNICATION
/* Initialise the module, Factory reset */
void initBLE(){

  Serial.print(F("Initialising the Bluefruit LE module: "));
  if ( !ble.begin(VERBOSE_MODE) )
  {
    //TODO: What can we do if module is not found ? (retry, set LED to specific color etc..)
   Serial.println(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );
  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if (!ble.factoryReset() ){
      Serial.println(F("Couldn't factory reset"));
    }
  }
 
  /* Disable command echo from Bluefruit */
  ble.echo(false);
  ble.verbose(false);  // debug info is a little annoying after this point!
  
// ab hier READY FOR CONNECTION (evlt LED color changen) 
/* Wait for connection */
  while (!ble.isConnected()) {
      
      Serial.println("waiting for BLE connection ..");
      delay(100);
  }// Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
    Serial.println(ble.setMode(BLUEFRUIT_MODE_DATA));
}

/*
 * listen for incoming BLE packages over a duration in milliseconds, if duration is 0 listens until packets are recieved
 */
void listenBLE(int *duration){
  // check if still connected
  Serial.println("hier bin ich ");
  // TODO extra command that swiches in cmd mode, checks and swiches back 
     delay(100);
     
   if (*duration > 0){
    Serial.println("hier bin ich asd");
    uint16_t listeningTime = *duration;
    while (listeningTime > 0){
      uint8_t len = readPacket(0);
      if (len > 0){
       if(processData()){
         Serial.println("updated SETTINGS"); // success response
       }
      }
      listeningTime--;
      delay(1);
      Serial.println(listeningTime);
    }
       /*uint8_t len = readPacketOverTime(listeningTime);
      if (len > 0){
       if(processData()){
         Serial.println("updated SETTINGS"); // success response
       }
      }*/
      
  }
  
  if (*duration == 0){
    Serial.println("hier bin ich aseee");
    // first time connecting since program running ,will be waiting for first recieved data 
    while(!readBLE()){
    delay(1);
    Serial.println("stuck here");
   }
  } 
}

/*
 * checks next data packet and calls processData() if lenght > 0
 */
boolean readBLE(){
  uint8_t len = readPacket(BLE_READPACKET_TIMEOUT);
  if (len == 0) return false;
  Serial.println("datapacket lenght ");Serial.println(len);
 return processData();
}

/*
 * read recieved data packet and do corresponding actions
 */
boolean processData(){
  boolean success = false;
   // recieved UVI PARAMETERS
  // Example !P10000010
  if (packetbuffer[1] == 'P'){ 
    char BUF[4];
    // get INTERVAL , first 4 chars
    BUF[0]= (char)packetbuffer[2];
    BUF[1]= (char)packetbuffer[3];
    BUF[2]= (char)packetbuffer[4];
    BUF[3]= (char)packetbuffer[5];
    interval = getInteger(BUF);
    // clear array
    memset (BUF,0,4);
    
    Serial.println("interval set to ");Serial.println(interval);
    
    // get AVGCOUNT
    BUF[0]= (char)packetbuffer[6];
    BUF[1]= (char)packetbuffer[7];
    BUF[2]= (char)packetbuffer[8];
    BUF[3]= (char)packetbuffer[9]; 
    avgCount = getInteger(BUF);
    Serial.println("avg set to ");Serial.println(avgCount);
    // clear array
    memset (BUF,0,4);
    // reset pointers
       num = 0; 
       sum = 0.0;
       
       success = true;
  }
  
  // OBSERVE 
  // EXAMPLE !O1
  if (packetbuffer[1] == 'O'){
    if (packetbuffer[2] == '1') {
      observeUV = true;
      Serial.println("observe: true");
      success = true;
    } else {
      observeUV = false;
      Serial.println("observe: false");
      success = true;
    }    
  }

  //SKINTYPE SET
  // EXAMPLE !S4
  if (packetbuffer[1] == 'S'){
    int val = packetbuffer[2];
    if (val > 0 && val <= 4){
      skintype = val;
      success = true;
    }
  }

// TIME OBSERVE
// EXAMPLE !T1
if (packetbuffer[1] == 'T'){
    if (packetbuffer[2] == '1') {
      observeT = true;
      Serial.println("observe: true");
      success = true;
    } else {
      observeT = false;
      Serial.println("observe: false");
      success = true;
    }   
} 

// ERYTHEM DOSIS OBSERVE
// EXAMPLE !D1
if (packetbuffer[1] == 'D'){
    if (packetbuffer[2] == '1') {
      observeD = true;
      Serial.println("observe: true");
      success = true;
    } else {
      observeD = false;
      Serial.println("observe: false");
      success = true;
    }    
  }
  
// clear incoming buffer!
    memset (packetbuffer,0,BUFSIZE);
    return success;
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


void updateUV(int *numPtr , double *sumPtr) {

    double UVindex = uv.readUV();
    // adjust recieved value
    UVindex /= 100;
  // check how many samples we have
  if (*numPtr == avgCount) {
    // avg samples reached
    double avgUV = (*sumPtr / *numPtr);
    //store lastAvg
    lastAvg = avgUV;
    // send avg to device
     char code = 'A';
    sendValues(code,avgUV);// 30 samples -> 1 min data -> send
    // reset counters
    *numPtr = 0; 
    *sumPtr = 0.0;
  } else {
    *numPtr += 1;
    *sumPtr += (UVindex);
    // observe for uv index turned on , send new values
    if (observeUV){
      char code = 'O';
      sendValues(code,UVindex);
    }
  }
  // store UVI
  lastUVI = UVindex;
  // calculate ED
  //calcED(UVindex);
  //double tLeft = getTimeLeft();
  //if (tLeft != 0 && observeT){
    // sendValues
  //}
  
  //debug
  Serial.print("UV: ");  Serial.println(UVindex);
  Serial.print("sum "); Serial.println(*sumPtr);
  Serial.print("n :"); Serial.println(*numPtr);
}


/*
 * Calculates the actual erythem dosis
 */
void calcED(double uvindex){
  if (skintype == 0){ // no skintype set, no ed
      return;
  }
  // duration = intervall in sec
  double sec = interval / 1000;
  ed += (uvindex * 0.025 * sec); // = 0,025 W/m^2 * seconds -> Joule
  // store in EEPROM in case of power off
  int eAdress = 0 ;
  EEPROM.put(eAdress,ed);
  if (observeD){
    // sendValues
     Serial.print("observeD: ");  Serial.println(observeD);
  }
}
/*
 * Calculates time left in seconds as double before reaching MED using the last measured UV-Index to calculate until first avg values exist
 */
double getTimeLeft(){
  if (skintype == 0){ // no skintype set, no ed
      return 0;
  }
  // maximum dosis according to skintype
  int med = getMED(skintype);
  double secondsLeft = 0;
   // check for valid med
  if (med != 0) {
    // calculate the difference between current erythem dosis and maximum to see how many seconds are left predicting with the newest average UV Index
    double dif = med - ed;
    // dosis reached already
    if ( dif <= 0 ){
      // WARNEN balb
    }
    // 2/3 des med erreicht - warnen
    // use newest average value if exists (default would be -1.0)
    if (lastAvg != -1.0){
      //get W/m^2 per second from average UVI
      double k = lastAvg * 0.025 * 1; // erythem dosis for 1 second
      secondsLeft = dif / k ;
    } else if (lastUVI != -1.0){
      //get W/m^2 per second from last meassured UVI
      double k = lastUVI * 0.025 * 1; // erythem dosis for 1 second 
      double secondsLeft = dif / k ;
    } else {
      // all values are 0, not able to calculate time 
    }
  }
  return secondsLeft;
}

int getMED(int skin){
  if (skin == 1) return 200;
  if (skin == 2) return 250;
  if (skin == 3) return 350;
  if (skin == 4) return 450;
  return 0;
}

/**************************************************************************/
/*!
    @brief  Waits for incoming data and parses it
*/
/**************************************************************************/
uint8_t readPacket(uint16_t timeout) 
{
  uint16_t origtimeout = timeout, replyidx = 0;
  memset(packetbuffer, 0, READ_BUFSIZE);

  while (timeout--) {
    if (replyidx >= 20) break;
    if ((packetbuffer[1] == 'P') && (replyidx == PACKET_UVIPARAM_LEN)){
      ble.flush();
      break;
    }
      
    if ((packetbuffer[1] == 'O') && (replyidx == PACKET_UVIOPTIONS_LEN)){
      ble.flush();
      break;
    }
    
    while (ble.available()) {
      char c =  ble.read();
      Serial.println(c);
      if (c == '!') {
        replyidx = 0;
      }
      packetbuffer[replyidx] = c;
      replyidx++;
      timeout = origtimeout;
    }
    
    if (timeout == 0) break;
    delay(1);
  }

 

  if (!replyidx)  // no data or timeout 
    return 0;
  if (packetbuffer[0] != '!')  // doesn't start with '!' packet beginning
    return 0;
  
  // check checksum!

  
  // checksum passed!
  return replyidx;
}

// will listen for the given time in any case, and return packets length afterwards if a packet was recieved
// might not work with multiple packets
uint8_t readPacketOverTime(uint16_t duration) 
{
  uint16_t  replyidx = 0;
  Serial.println(duration);
  memset(packetbuffer, 0, READ_BUFSIZE);

  while (duration > 0) {
    if (replyidx >= 20) break;
    if ((packetbuffer[1] == 'P') && (replyidx == PACKET_UVIPARAM_LEN))break;
    if ((packetbuffer[1] == 'O') && (replyidx == PACKET_UVIOPTIONS_LEN))break;
   Serial.println("vor q");
     while (ble.available()) {
      Serial.println("vor q");
      char c =  ble.read();
      Serial.println(c);
      if (c == '!') {
        replyidx = 0;
      }
      packetbuffer[replyidx] = c;
      replyidx++;
    }
     delay(1);
    duration--;
    if (duration == 0) break;
   Serial.println(duration);
  }
  // wait remaining time
    while(duration > 0){
      delay(1);
      duration--;
    }
  

  packetbuffer[replyidx] = 0;  // null term

  if (!replyidx)  // no data or timeout 
    return 0;
  if (packetbuffer[0] != '!')  // doesn't start with '!' packet beginning
    return 0;
  
  // check checksum!
  
  // checksum passed!
  return replyidx;
}


/*int getDecimalCount(double num){
  int count = 0;
  num = abs(num);
int remainder = int(0.5 + 10000000 * (num - int(num)));

while ( remainder % 10 == 0 )
{
    remainder = remainder / 10;
    ++count;
}
return count;
}

char* appendCharArrayToChar(char* array,  const char* a){
    size_t len = strlen(array);
    char ret[len+2];
    strcpy(ret,a);
    strcat( ret,array);
    return ret;
} */
  



