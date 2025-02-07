/*
  SH1106 OLED
  ADXL 
  Reads 4096 XYZ readings in  sec
  SD Shield
  record taken time
  4 decimal places
  4 May 2021
  19 Feb 2024   NOW 
    
*/
 
#include <SPI.h>
#include <mySD.h>
#include "Arduino.h"
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"
#include <U8g2lib.h>
#include<ADXL345.h>
ADXL345 accel(ADXL345_ALT);
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

int ii;
float accX[4096], accY[4096], accZ[4096];
char fileContents[128];   // Probably can be smaller
int fileNumber;
char f5[5], f6[6], f7[7], macAddr[18];
String fTemp;
char tb[5], f1[1], f2[2], f3[3];
long startTime, endTime;
float timeTaken, Hz;

File myFile;

uint8_t broadcastAddress[] = {0xE0, 0x5A, 0x1B, 0xD0, 0x88, 0xB0};

typedef struct struct_message {
  bool  start;
  int   fileIndex;
  int   mode;
  int   valueIndex;
  float xReading;
  float yReading;
  float zReading;
  float frequency;
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

const int chipSelect = 4;

void updateFileNumber() {
    SD.remove("filenum.txt");
    myFile = SD.open("filenum.txt", FILE_WRITE);
    switch (fileNumber) {
      case 1 ... 9 :
        dtostrf(fileNumber, 1, 0, f1);
        myFile.println(f1);
        break;
      case 10 ...99 :
        dtostrf(fileNumber, 2, 0, f2);
        myFile.println(f2);
        break;
      case 100 ... 999 :
        dtostrf(fileNumber, 3, 0, f3);
        myFile.println(f3);
        break;
      default :
        myFile.println("0000");      
    }
    myFile.close();  
    Serial.println(fileNumber);    
}

void readSDFileName() {
    myFile = SD.open("filenum.txt");
    if (myFile) {
    
      // read from the file until there's nothing else in it:
    
      while (myFile.available()) {
        char aChar = myFile.read();
        if(aChar != '\n' && aChar != '\r')
        {
           fileContents[ii++] = aChar;
           fileContents[ii] = '\0'; // NULL terminate the array
        }
        else // the character is CR or LF
        {
           if(strlen(fileContents) > 0)
           {
              fileNumber = atof(fileContents);
              fileNumber++;
           }
           fileContents[0] = '\0';
        }
      }  
      // close the file:
      myFile.close();
    } else {
      // if the file didn't open, print an error:
      myFile = SD.open("filenum.txt", FILE_WRITE);
      myFile.println("1");
      fileNumber=1;
      myFile.close();
    }

    updateFileNumber();
}

void openFile() {
      switch (fileNumber) {
      case 1 ... 9 :
        dtostrf(fileNumber, 1, 0, f5);
        fTemp=String(f5)+".txt";
        fTemp.toCharArray(f5, 6);
        Serial.println(f5);
        myFile = SD.open(f5, FILE_WRITE);
        break;
      case 10 ...99 :
        dtostrf(fileNumber, 2, 0, f6);
        fTemp=String(f6)+".txt";
        fTemp.toCharArray(f6, 7);
        Serial.println(f6);
        myFile = SD.open(f6, FILE_WRITE);
        break;
      case 100 ... 999 :
        dtostrf(fileNumber, 3, 0, f7);
        fTemp=String(f7)+".txt";
        fTemp.toCharArray(f7, 8);
        Serial.println(f7);
        myFile = SD.open(f7, FILE_WRITE);
        break;
      default :
        myFile.println("0000");      
    }

    if (myFile) {
    Serial.print("Writing to SD file...");
    Hz = 4096 / timeTaken;
    myFile.print("Sampling Rate (Hz) : "); myFile.println(Hz,2); 
    myFile.println("X Y Z");
    myFile.println("Mg Mg Mg");
    for (ii=0; ii <=4095; ii++) {
      myFile.print(accX[ii],4);   myFile.print(" ");
      myFile.print(accY[ii],4);   myFile.print(" ");
      myFile.println(accZ[ii],4);
  }
  
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
  }
}

void sendAllData() {
  myData.fileIndex = fileNumber;
  for (ii=0; ii<=4095; ii++) {
    myData.valueIndex = ii;
    myData.xReading = accX[ii];
    myData.yReading = accY[ii];
    myData.zReading = accZ[ii];
    sendNOW();
    delay(10);
  }
}

void readADXL345() {
  startTime = millis();
  for (ii=0; ii<=4095; ii++) {
    if (accel.update()) {
      accX[ii] = accel.getX();
      accY[ii] = accel.getY();
      accZ[ii] = accel.getZ();
    } else {
      Serial.println("update failed");
      while(1) {
        delay(100);
      }
    }
  } 
  endTime = millis();
  timeTaken = abs (endTime - startTime);
  timeTaken = timeTaken / 1000.0;
  myData.frequency = 4096.0 / timeTaken;
  Serial.print("Time Taking ...");
  Serial.println(startTime);
  Serial.println(endTime);
  Serial.println(timeTaken,4);
}


void setup()
{
  pinMode(D3, INPUT_PULLUP);
  u8g2.begin();
  u8g2.clearBuffer();          // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    
  Wire.begin();

  Serial.begin(115200);
  
  Serial.print("Initializing SD card...");

  pinMode(SS, OUTPUT);


  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.print("MAC Address: ");

  WiFi.macAddress().toCharArray(macAddr, 18);
  Serial.println(macAddr);
    
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

  // SD.begiin(CS, MOSI, MISO, CLK); 
  if (!SD.begin(D8, D7, D6, D5)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  Serial.println("ADXL345_Sketch - Basic Data");
  Serial.println();
  byte deviceID = accel.readDeviceID();
  if (deviceID != 0) {
    Serial.print("0x");
    Serial.print(deviceID, HEX);
    Serial.println("");
  } else {
    Serial.println("read device id: failed");
    while(1) {
      delay(100);
    }
  }
   
  // Data Rate
  // - ADXL345_RATE_3200HZ: 3200 Hz
  // - ADXL345_RATE_1600HZ: 1600 Hz
  // - ADXL345_RATE_800HZ:  800 Hz
  // - ADXL345_RATE_400HZ:  400 Hz
  // - ADXL345_RATE_200HZ:  200 Hz
  // - ADXL345_RATE_100HZ:  100 Hz
  // - ADXL345_RATE_50HZ:   50 Hz
  // - ADXL345_RATE_25HZ:   25 Hz
  // - ...
  if (!accel.writeRate(ADXL345_RATE_3200HZ)) {
    Serial.println("write rate: failed");
    while(1) {
      delay(100);
    }
  }

  // Data Range
  // - ADXL345_RANGE_2G: +-2 g
  // - ADXL345_RANGE_4G: +-4 g
  // - ADXL345_RANGE_8G: +-8 g
  // - ADXL345_RANGE_16G: +-16 g
  if (!accel.writeRange(ADXL345_RANGE_8G)) {
    Serial.println("write range: failed");
    while(1) {
      delay(100);
    }
  }

  if (!accel.start()) {
    Serial.println("start: failed");
    while(1) {
      delay(100);
    }
  }  
  myData.start = false;
  myData.mode = 0;
}

void displayOn() {
  u8g2.clearBuffer(); 
  u8g2.setFont(u8g2_font_logisoso42_tn);
  u8g2.drawStr(23, 55, tb);
  u8g2.sendBuffer();      
}

void flashDisplay() {
  u8g2.clearBuffer(); 
  u8g2.sendBuffer();      
  delay(500);
  
  u8g2.clearBuffer(); 
  u8g2.setFont(u8g2_font_logisoso42_tn);
  u8g2.drawStr(23, 55, tb);
  u8g2.sendBuffer();         
  delay(500); 
  
  u8g2.clearBuffer(); 
  u8g2.sendBuffer();      
  delay(500);
  
  u8g2.clearBuffer(); 
  u8g2.setFont(u8g2_font_logisoso42_tn);
  u8g2.drawStr(23, 55, tb);
  u8g2.sendBuffer();         
  delay(500);   

  u8g2.clearBuffer(); 
  u8g2.sendBuffer();      
  delay(500);
  
  u8g2.clearBuffer(); 
  u8g2.setFont(u8g2_font_logisoso42_tn);
  u8g2.drawStr(23, 55, tb);
  u8g2.sendBuffer();         
  delay(500);   
}

void loop()
{
  readSDFileName();

  dtostrf(fileNumber, 3, 0, tb);
  displayOn();
      
  //ssd1306_printFixed(23, 20, tb, STYLE_NORMAL);    //display file number

  //Serial.println(digitalRead(D3));
  while (digitalRead(D3) == HIGH) {}
  
  flashDisplay();
       
  readADXL345();
  
  sendAllData();

  openFile();

  fileNumber++;
  
  updateFileNumber();
  
}
