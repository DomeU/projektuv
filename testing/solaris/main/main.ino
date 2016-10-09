#include <Arduino.h>
#include <SPI.h>
// BLE Libs
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
// Sensor Libs
#include <Wire.h>
#include <Adafruit_SI1145.h>

// defines and variables for parsing incoming data
#define FACTORYRESET_ENABLE     1

#define PACKET_UVIPARAM_LEN             (10)
#define PACKET_UVIOPTIONS_LEN            (4)

//    READ_BUFSIZE            Size of the read buffer for incoming packets
#define READ_BUFSIZE                    (20)


/* Buffer to hold incoming characters */
uint8_t packetbuffer[READ_BUFSIZE+1];

 Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);
 // UV SENSOR
 Adafruit_SI1145 uv = Adafruit_SI1145();
 
void setup() {
  Serial.begin(115200); // starts serial communication 
  // remove if working without serial monitor
  while (!Serial) {
   delay(500);
  };  // required for Flora & Micro
  // initialize UV sensor
  while(!uv.begin()){
    //TODO: What can we do if sensor is not found ? (retry, set LED to specific color etc..)
    Serial.println("Didn't find Si1145");
   delay(1000);
  }
  // init BLE Module
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
  // set to Data mode 
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);
}

// VARIABLES AND POINTER
int num = 0;
int *numPointer = &num; // points at variable for storing how much samples were taken
double sum = 0.0;
double* sumPointer = &sum; // points at the sum of all samples added up
// Parameter
int interval = 1000;
int* intervalPointer = &interval; // points at the interval between meassurements

int avgCount = 20;
// observe flags
boolean observeUV = false; // observe UV Index
boolean observeT = true; // observe Time-left
boolean observeD = true; // observe current MED

// UV specific 
double ed = 0.0; // current erythema dosis
//double *edPointer = &ed; // pointer to ED

double lastAvg = -1.0;
double* lastAvgP = &lastAvg;
double lastUvi = -1.0;
double* lastUviP = &lastUvi;

int skintype = 1; // store selected skintype 

// CALCULATING FUNCTIONS // 

/* UVI data will be read and ErythemDosis and TimeLeft calculated */
void updateUV(int *numPtr , double *sumPtr, double *lA, double *lU) {
    double UVindex = uv.readUV();
    // adjust recieved value
    UVindex /= 100;
  // check how many samples we have
  if (*numPtr == avgCount) {
    // avg samples reached
    double avgUV = (*sumPtr / *numPtr);
    //store lastAvg
    *lA = avgUV;
    // send avg to device
     char code = 'A';
    //sendValues(code,avgUV);// 30 samples -> 1 min data -> send
    // reset counters
    *numPtr = 0; 
    *sumPtr = 0.0;
  } else {
    *numPtr += 1;
    *sumPtr += (UVindex);
    // observe for uv index turned on , send new values
    if (observeUV){
      char code = 'O';
      //sendValues(code,UVindex);
    }
  }
  // store UVI
  *lU = UVindex;
  // calculate ED
  calcED(UVindex);
  double tLeft = getTimeLeft();
  if (tLeft != 0 && observeT){
    // sendValues
    Serial.print("timeleft: ");Serial.println(tLeft);
  }
  
  //debug
  Serial.print("UV: ");  Serial.println(UVindex);
  Serial.print("sum "); Serial.println(*sumPtr);
  Serial.print("n :"); Serial.println(*numPtr);
  Serial.print("lastAvg :"); Serial.println(*lA);
  Serial.print("lastUvi :"); Serial.println(*lU);
  Serial.println("---------------------------------------");
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
  //int eAdress = 0 ;
  //EEPROM.put(eAdress,ed);
  if (observeD){
    // sendValues
     Serial.print("observeD: ");  printDouble(ed,10000);
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
  // methodenaufruf durch globale variable sparen
  int med = getMED(skintype);
  double secondsLeft = 0;
   // check for valid med
  if (med != 0) {
    // calculate the difference between current erythem dosis and maximum to see how many seconds are left predicting with the newest average UV Index
    double dif = med - ed;
    Serial.print("dif:");printDouble(dif,10000);
    // dosis reached already
    if ( dif <= 0 ){
      // WARNEN balb
    }
    // 2/3 des med erreicht - warnen
    // use newest average value if exists (default would be -1.0)
    if (*lastAvgP != -1.0){
      Serial.print("avg:");
      //get W/m^2 per second from average UVI
      double k = *lastAvgP * 0.025 * 1; // erythem dosis for 1 second
      secondsLeft = dif / k ;
    } else if (*lastUviP != -1.0){
      
      //get W/m^2 per second from last meassured UVI
      double k = *lastUviP * 0.025 * 1; // erythem dosis for 1 second 
      secondsLeft = dif / k ;//Serial.print("lastUVI:");Serial.println(secondsLeft);
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

// COMMUNICATION AND PROCESSING //
// workaround

void printDouble( double val, unsigned int precision){
// prints val with number of decimal places determine by precision
// NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
// example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

   Serial.print (int(val));  //prints the int part
   Serial.print("."); // print the decimal point
   unsigned int frac;
   if(val >= 0)
     frac = (val - int(val)) * precision;
   else
      frac = (int(val)- val ) * precision;
   int frac1 = frac;
   while( frac1 /= 10 )
       precision /= 10;
   precision /= 10;
   while(  precision /= 10)
       Serial.print("0");

   Serial.println(frac,DEC) ;
}

void listenBL(int *duration){
  int milliseconds = *duration;
  while (milliseconds > 0){
    if (readPacket(10) > 0){
      processData();
    }
    milliseconds -= 10;
  }
}

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
  // checksum passed!
  return replyidx;
}

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



void loop() {
  updateUV(numPointer,sumPointer,lastAvgP,lastUviP);
  listenBL(intervalPointer);
}
