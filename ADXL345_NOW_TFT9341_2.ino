/*
 * ADXL345_NOW_Sender_WE
 * ESPNOW
 * ILI9431 240x320
 * 14 Aug 2024
*/

/*
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  double *by=(double*)incomingData;
  Serial.print("rearray=: ");
  for(int i=0;i<100;i++){
    Serial.print(by[i]);
  }
  Serial.println();
} 
*/

#include "arduinoFFT.h"
#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>
#include "Arduino.h"
#include "Wire.h"
#include <esp_wifi.h>
#include <esp_now.h>
#include <WiFi.h>

TFT_eSPI tft = TFT_eSPI();                   // Invoke custom library with default width and height

/*
These values can be changed in order to evaluate the functions
*/
const uint16_t samples = 4096; //This value MUST ALWAYS be a power of 2
const float signalFrequency = 1500;
const float samplingFrequency = 3000;
const uint8_t amplitude = 100;
unsigned int sampling_period_us;
unsigned long microseconds;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/

int ii;
//float accX[4096], accY[4096], accZ[4096];
float accZ[samples];
char fileContents[128];   // Probably can be smaller
int fileNumber;
char f5[5], f6[6], f7[7];
String fTemp;
char tb[5], f1[1], f2[2], f3[3];
long startTime, endTime;
float timeTaken, Hz;

float vReal[samples];
float vImag[samples];
float result[samples];
float resultFrequency, resultMagnitude, yMax = 0.0;
float ox = -999, oy = -999; // Force them to be off screen
boolean display1 = true;
boolean update1 = true;

//ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, samples, samplingFrequency, true);

char macAddr[18];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

//#define startButton 20
#define startButton 27

#define LTBLUE    0xB6DF
#define LTTEAL    0xBF5F
#define LTGREEN   0xBFF7
#define LTCYAN    0xC7FF
#define LTRED     0xFD34
#define LTMAGENTA 0xFD5F
#define LTYELLOW  0xFFF8
#define LTORANGE  0xFE73
#define LTPINK    0xFDDF
#define LTPURPLE  0xCCFF
#define LTGREY    0xE71C

#define BLUE      0x001F
#define TEAL      0x0438
#define GREEN     0x07E0
#define CYAN      0x07FF
#define RED       0xF800
#define MAGENTA   0xF81F  //purple
#define YELLOW    0xFFE0  //green
#define ORANGE    0xFC00
#define PINK      0xF81F  //purple
#define PURPLE    0x8010
#define GREY      0xC618
#define WHITE     0xFFFF
#define BLACK     0x0000

#define DKBLUE    0x000D
#define DKTEAL    0x020C
#define DKGREEN   0x03E0
#define DKCYAN    0x03EF
#define DKRED     0x6000
#define DKMAGENTA 0x8008
#define DKYELLOW  0x8400
#define DKORANGE  0x8200
#define DKPINK    0x9009
#define DKPURPLE  0x4010
#define DKGREY    0x4A49

uint8_t broadcastAddress[] = {0x30, 0xC9, 0x22, 0x32, 0x15, 0xF0};

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
  if ( myData.itemNumber < 2048 ) { 
    result[myData.itemNumber] = myData.zData; 
    //vImag[myData.itemNumber] = 0.0;
  }
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

/*

  function to draw a cartesian coordinate system and plot whatever data you want
  just pass x and y and the graph will be drawn

  huge arguement list
  &d name of your display object
  x = x data point
  y = y datapont
  gx = x graph location (lower left)
  gy = y graph location (lower left)
  w = width of graph
  h = height of graph
  xlo = lower bound of x axis
  xhi = upper bound of x asis
  xinc = division of x axis (distance not count)
  ylo = lower bound of y axis
  yhi = upper bound of y asis
  yinc = division of y axis (distance not count)
  title = title of graph
  xlabel = x asis label
  ylabel = y asis label
  &redraw = flag to redraw graph on first call only
  color = plotted trace colour
*/

void Graph(double x, double y, byte dp,
                           double gx, double gy, double w, double h,
                           double xlo, double xhi, double xinc,
                           double ylo, double yhi, double yinc,
                           char *title, char *xlabel, char *ylabel,
                           boolean &redraw, unsigned int color) {

  double ydiv, xdiv;
  double i;
  double temp;
  int rot, newrot;

  // gcolor = graph grid colors
  // acolor = axes line colors
  // pcolor = color of your plotted data
  // tcolor = text color
  // bcolor = background color
  unsigned int gcolor = BLUE;
  unsigned int acolor = RED;
  unsigned int pcolor = color;
  unsigned int tcolor = WHITE;
  unsigned int bcolor = BLACK;

  //if (redraw == true) {
  if (true) {
    redraw = false;
    // initialize old x and old y in order to draw the first point of the graph
    // but save the transformed value
    // note my transform funcition is the same as the map function, except the map uses long and we need doubles
    //ox = (x - xlo) * ( w) / (xhi - xlo) + gx;
    //oy = (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;

    tft.setTextDatum(MR_DATUM);

    // draw y scale
    for ( i = ylo; i <= yhi; i += yinc) {
      // compute the transform
      temp =  (i - ylo) * (gy - h - gy) / (yhi - ylo) + gy;

      if (i == 0) {
        tft.drawLine(gx, temp, gx + w, temp, acolor);
        tft.setTextColor(acolor, bcolor);
        tft.drawString(xlabel, (int)(gx + w) , (int)temp, 2);
      }
      else {
        tft.drawLine(gx, temp, gx + w, temp, gcolor);
      }
      // draw the axis labels
      tft.setTextColor(tcolor, bcolor);
      // precision is default Arduino--this could really use some format control
      tft.drawFloat(i, dp, gx - 4, temp, 1);
    }

    // draw x scale
    for (i = xlo; i <= xhi; i += xinc) {

      // compute the transform
      temp =  (i - xlo) * ( w) / (xhi - xlo) + gx;
      if (i == 0) {
        tft.drawLine(temp, gy, temp, gy - h, acolor);
        tft.setTextColor(acolor, bcolor);
        tft.setTextDatum(BC_DATUM);
        tft.drawString(ylabel, (int)temp, (int)(gy - h - 8) , 2);
      }
      else {
        tft.drawLine(temp, gy, temp, gy - h, gcolor);
      }
      // draw the axis labels
      tft.setTextColor(tcolor, bcolor);
      tft.setTextDatum(TC_DATUM);
      // precision is default Arduino--this could really use some format control
      tft.drawFloat(i, dp, temp, gy + 7, 1);
    }

    //now draw the graph labels
    tft.setTextColor(tcolor, bcolor);
    tft.drawString(title, (int)(gx + w / 2) , (int)(gy - h - 30), 4);
  }

  // the coordinates are now drawn, plot the data
  // the entire plotting code are these few lines...
  // recall that ox and oy are initialized above
  //x =  (x - xlo) * ( w) / (xhi - xlo) + gx;
  //y =  (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  //tft.drawLine(ox, oy, x, y, pcolor);
  // it's up to you but drawing 2 more lines to give the graph some thickness
  //tft.drawLine(ox, oy + 1, x, y + 1, pcolor);
  //tft.drawLine(ox, oy - 1, x, y - 1, pcolor);
  //ox = x;
  //oy = y;

}

void Trace(double x,  double y,  byte dp,
           double gx, double gy,
           double w, double h,
           double xlo, double xhi, double xinc,
           double ylo, double yhi, double yinc,
           char *title, char *xlabel, char *ylabel,
           boolean &update1, unsigned int color)
{
  double ydiv, xdiv;
  double i;
  double temp;
  int rot, newrot;

  //unsigned int gcolor = DKBLUE;   // gcolor = graph grid color
  unsigned int acolor = RED;        // acolor = main axes and label color
  unsigned int pcolor = color;      // pcolor = color of your plotted data
  unsigned int tcolor = GREY;       // tcolor = text color
  unsigned int bcolor = BLACK;      // bcolor = background color

  // initialize old x and old y in order to draw the first point of the graph
  // but save the transformed value
  // note my transform funcition is the same as the map function, except the map uses long and we need doubles
  if (update1) {
    update1 = false;
    
    ox = (x - xlo) * ( w) / (xhi - xlo) + gx;
    oy = (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;

    if ((ox < gx) || (ox > gx+w)) {update1 = true; return;}
    if ((oy < gy-h) || (oy > gy)) {update1 = true; return;}
    

    tft.setTextDatum(MR_DATUM);

    // draw y scale
    for ( i = ylo; i <= yhi; i += yinc) {
      // compute the transform
      temp =  (i - ylo) * (gy - h - gy) / (yhi - ylo) + gy;

      if (i == 0) {
        tft.setTextColor(acolor, bcolor);
        tft.drawString(xlabel, (int)(gx + w) , (int)temp, 2);
      }
      // draw the axis labels
      tft.setTextColor(tcolor, bcolor);
      // precision is default Arduino--this could really use some format control
      tft.drawFloat(i, dp, gx - 4, temp, 1);
    }

    // draw x scale
    for (i = xlo; i <= xhi; i += xinc) {

      // compute the transform
      temp =  (i - xlo) * ( w) / (xhi - xlo) + gx;
      if (i == 0) {
        tft.setTextColor(acolor, bcolor);
        tft.setTextDatum(BC_DATUM);
        tft.drawString(ylabel, (int)temp, (int)(gy - h - 8) , 2);
      }

      // draw the axis labels
      tft.setTextColor(tcolor, bcolor);
      tft.setTextDatum(TC_DATUM);
      // precision is default Arduino--this could really use some format control
      tft.drawFloat(i, dp, temp, gy + 7, 1);
    }

    //now draw the graph labels
    tft.setTextColor(tcolor, bcolor);
    tft.drawString(title, (int)(gx + w / 2) , (int)(gy - h - 30), 4);
  }

  // the coordinates are now drawn, plot the data
  // the entire plotting code are these few lines...
  // recall that ox and oy are initialized above
  x =  (x - xlo) * ( w) / (xhi - xlo) + gx;
  y =  (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;

  if ((x < gx) || (x > gx+w)) {update1 = true; return;}
  if ((y < gy-h) || (y > gy)) {update1 = true; return;}
    
    
  tft.drawLine(ox, oy, x, y, pcolor);
  
  //it's up to you but drawing 2 more lines to give the graph some thickness
  //tft.drawLine(ox, oy + 1, x, y + 1, pcolor);
  //tft.drawLine(ox, oy - 1, x, y - 1, pcolor);
  
  ox = x;
  oy = y;

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

void setup()
{
  Serial.begin(115200);
  Serial.println("Ready");
  pinMode(startButton, INPUT_PULLUP);
  
  sampling_period_us = round(1000000*(1.0/samplingFrequency));

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

  tft.begin();
  tft.fillScreen(BLACK);
  tft.setRotation(1);
  tft.setTextColor(GREEN , BLACK);
  tft.setTextSize(5);
}

void plotGraphs() {
  double x, y, yMagnitude;
  int xx=270, yy=130, ii, noOfOutputs=2048, yStep;
  float jj;
  char t[8], h[8];

  yMagnitude = int(yMax * 1.1);
  yStep = int(yMagnitude / 4.0);
  tft.fillScreen(BLACK);
  //tft.setRotation(1);
  tft.setTextSize(1);
  //tft.setFreeFont(FSB9); 
  //Serial.println(plotIndex);
  //Serial.print(temp[plotIndex]); Serial.print(" : ");  Serial.println(humi[plotIndex]);

  //Serial.print(temp[plotIndex]); Serial.print(" : ");  Serial.println(humi[plotIndex]);

  //Graph(tft, x, y, 1, 60, 290, 390, 260, 0, 6.5, 1, -1, 1, .25, "", "", "", display1, YELLOW);
 
  //Graph(x, y, 0, 35, 225, xx, yy, 0, 1500, 300, 0, yMagnitude, yStep, "FFT Frequency Domain", "Hz", "", display1, TFT_YELLOW);
  
  Graph(x, y, 0, 35, 225, xx, yy, 0, 1500, 300, 0, yMagnitude, yStep, "FFT Frequency Domain", "Hz", "", display1, TFT_YELLOW);
    
  // Plot Curve
  update1 = true;
  ii=0;
  for (x = 0; x < noOfOutputs; x += 1) {
    y = result[ii];
    ii++;
    //jj = x / 24;
    jj = x * 1428 / 2048;
    //Trace(tft, jj, y, 1, 60, 290, xx, yy, 0, 7.0, 1, 0, 100, 20, "Temperature & Humidity", "", "", update1, TFT_YELLOW);
    Trace(jj, y, 0, 35, 225, xx, yy, 0, 1500, 300, 0, yMagnitude, yStep, "", "", "", update1, TFT_YELLOW); 
  }
  
  tft.setTextSize(1);
  tft.setTextColor(YELLOW, BLACK);
  dtostrf(resultFrequency, 5, 1, t);
  tft.drawString(t, 185, 25, 4);
  tft.drawString("Peak :", 100,25,4);   tft.drawString("Hz", 250,25, 4);

}

void loop()
{
  Serial.println("Press Button to Start");
  while (digitalRead(startButton) == HIGH) {}
  Serial.println("Start");
  myData.itemNumber = -1;  // Start Measurement
  sendNOW();
  
  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.drawString("Measuring ...", 90,85,2);

  while (myData.itemNumber < 2048) { 
    tft.drawLine(0, 150, int(myData.itemNumber/7), 150, YELLOW);    
    //delay(1); 
  }
  resultFrequency = myData.frequency;

  Serial.println("2048 Data Received");
  Serial.print("Result Frequency : "); Serial.println(resultFrequency);
 
  tft.fillScreen(BLACK);
  tft.drawString("Measuring ...", 100,25,4);
  for (ii=0; ii<2048; ii++) {
    if (result[ii] > yMax) { yMax = result[ii]; }
  }
  plotGraphs();
  while (digitalRead(startButton) == HIGH) {}
  //for (ii = 0; ii < 4096; ii++) { vReal[ii] = 0.0; }
  tft.fillScreen(BLACK);
  tft.setRotation(1);
  tft.setTextColor(GREEN , BLACK);
  tft.setTextSize(5);
  //while(1); /* Run Once */
  // delay(2000); /* Repeat after delay */
}

