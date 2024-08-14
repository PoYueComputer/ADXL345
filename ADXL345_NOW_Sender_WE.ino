/*
  ADXL345_NOW_TFT9341
  ADXL 
  Reads 4096 XYZ readings in  sec
  ESP NOW Sender
  record taken time
  4 decimal places
  14 Aug 2024
    
*/

#include <SPI.h>
#include "arduinoFFT.h"
#include "Arduino.h"
#include "Wire.h"
#include <esp_wifi.h>
#include <esp_now.h>
#include <WiFi.h>
#include<ADXL345_WE.h>
#define ADXL345_I2CADDR 0x53 // 0x1D if SDO = HIGH
//ADXL345 accel(ADXL345_ALT);

ADXL345_WE myAcc = ADXL345_WE(ADXL345_I2CADDR);

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

const uint16_t samples = 4096; //This value MUST ALWAYS be a power of 2
float signalFrequency = 1800.0; 
float samplingFrequency = 3600.0;
const uint8_t amplitude = 100;
float vReal[samples];
float vImag[samples];
float result[samples];
double resultFrequency, resultMagnitude, yMax;
long startTime, endTime;
float timeTaken, Hz;
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, samples, samplingFrequency, true);


//int sda_pin = 1; 
//int scl_pin = 0; 


int ii;
float accZ[4096];
//float accX[4096], accY[4096], accZ[4096];
char macAddr[18];

//uint8_t broadcastAddress[] = {0x68, 0xB6, 0xB3, 0x2A, 0x1E, 0xA4};  //ESP32 S3
uint8_t broadcastAddress[] = {0xE0, 0x5A, 0x1B, 0xD0, 0x9A, 0xC0};    //Yellow Box

typedef struct struct_message {
  int itemNumber;   // 4096 - Start, 4097 - Finished, 0 - 4095 (Data Index)
  float frequency;  // mHz
  float zData;
} struct_message;

// Create a struct_message called myData
struct_message myData;


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
 
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  //myData.received = 0;
}


void sendNOW() {
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    //Serial.println("Sent with success");
  }
  else {
    //Serial.println("Error sending the data");
  }
} 

void readADXL345() {
  //for (ii=0; ii<samples; ii++) {vImag[ii]=0.0;}
  startTime = millis();
  for (ii=0; ii<samples; ii++) {
    xyzFloat raw = myAcc.getRawValues();
    //xyzFloat g = myAcc.getGValues();
    vReal[ii] = raw.z;   vImag[ii]=0.0;
  } 

  endTime = millis();
  timeTaken = abs (endTime - startTime);
  timeTaken = timeTaken / 1000;
  Serial.print("Time Taking ...");
  Serial.println(timeTaken,4);
  samplingFrequency = samples / timeTaken;
  signalFrequency = samplingFrequency / 2.0;  
  Serial.print("Frequency : ");  Serial.println(samplingFrequency,4);
}

void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

void setup() {

  //Wire.setPins(sda_pin, scl_pin);
  Wire.begin();
  Wire.setClock(400000);      // I2C Clock 400kHz
  Serial.begin(115200);
  pinMode(13, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);  
  pinMode(SS, OUTPUT);
  
  //sampling_period_us = round(1000000*(1.0/samplingFrequency));

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.print("[DEFAULT] ESP32 Board MAC Address: ");
  readMacAddress();
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
 
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
 
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ADXL345_Sketch - Basic Data");
  Serial.println();
  if(!myAcc.init()){
    Serial.println("ADXL345 not connected!");
  }
   
/* Choose the data rate         Hz
    ADXL345_DATA_RATE_3200    3200
    ADXL345_DATA_RATE_1600    1600
    ADXL345_DATA_RATE_800      800
    ADXL345_DATA_RATE_400      400
    ADXL345_DATA_RATE_200      200
    ADXL345_DATA_RATE_100      100
    ADXL345_DATA_RATE_50        50
    ADXL345_DATA_RATE_25        25
    ADXL345_DATA_RATE_12_5      12.5  
    ADXL345_DATA_RATE_6_25       6.25
    ADXL345_DATA_RATE_3_13       3.13
    ADXL345_DATA_RATE_1_56       1.56
    ADXL345_DATA_RATE_0_78       0.78
    ADXL345_DATA_RATE_0_39       0.39
    ADXL345_DATA_RATE_0_20       0.20
    ADXL345_DATA_RATE_0_10       0.10
*/
  myAcc.setDataRate(ADXL345_DATA_RATE_3200);
  Serial.print("Data rate: ");
  Serial.print(myAcc.getDataRateAsString());

/* In full resolution raw values the size of the raw values depend 
    on the range: 2g = 10 bit; 4g = 11 bit; 8g = 12 bit; 16g =13 bit;
    uncomment to change to 10 bit for all ranges. 
 */
  // myAcc.setFullRes(false);

/* Choose the measurement range
    ADXL345_RANGE_16G    16g     
    ADXL345_RANGE_8G      8g     
    ADXL345_RANGE_4G      4g   
    ADXL345_RANGE_2G      2g
*/ 
  myAcc.setRange(ADXL345_RANGE_16G);
  Serial.print("  /  g-Range: ");
  Serial.println(myAcc.getRangeAsString());
  Serial.println();
  delay(1000);
  Serial.println("Ready");
  myData.itemNumber = 4098;
  sendNOW();
}

void PrintVector(float *vData, uint16_t bufferSize, uint8_t scaleType)
{
  yMax = 0;
  Serial.print("Buffer Size : ");  Serial.println(bufferSize);
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
	break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
	break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
	break;
    }
    //Serial.print(i);  Serial.print(" ");
    //Serial.print(abscissa, 6);
    //if(scaleType==SCL_FREQUENCY)
      //Serial.print("Hz");
    //Serial.print(" ");

    result[i] = vData[i];
    //if (result[i] > yMax) { yMax = result[i]; }
    //Serial.println(vData[i], 4);
  }
  //Serial.println();
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);
  //if ( ( myData.itemNumber == 4096 ) or (1))  {
  if ( myData.itemNumber == -1 )  {    
    digitalWrite(LED_BUILTIN, HIGH);
    readADXL345();

  /* Print the results of the simulated sampling according to time */
  //Serial.println("Data:");
  PrintVector(vReal, samples, SCL_TIME);
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);	/* Weigh data */
  Serial.println("Weighed data:");
  PrintVector(vReal, samples, SCL_TIME);
  FFT.compute(FFTDirection::Forward); /* Compute FFT */
  Serial.println("Computed Real values:");
  PrintVector(vReal, samples, SCL_INDEX);
  Serial.println("Computed Imaginary values:");
  PrintVector(vImag, samples, SCL_INDEX);
  FFT.complexToMagnitude(); /* Compute magnitudes */
  Serial.println("Computed magnitudes:");
  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  float x = FFT.majorPeak();
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("Peak Hz : ");   Serial.println(x, 6); //Print out what frequency is the most dominant.
  //Serial.println(v, 6);  
  //while (1) {};
    
    myData.frequency = x;
    for (ii=0; ii<2048; ii++) {
      myData.itemNumber = ii;
      myData.zData = result[ii];    
      //Serial.print("Send : "); Serial.println(myData.itemNumber);     
      sendNOW();   delay(3); 
    }
    //Serial.print("Frequency : "); Serial.println(myData.frequency);  
    Serial.println("2048 items Completed");
    myData.itemNumber = 2049;  
    sendNOW();
    
  }
}
