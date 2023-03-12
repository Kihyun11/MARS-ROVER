#include <Arduino.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>

#include <SPI.h>
#include <WiFi.h>
#include "esp_wpa2.h"
#include <Wire.h>

#include <HTTPClient.h>

#include <Robojax_L298N_DC_motor.h>

#define RXP1 3 // Defining UART With Vision (Pins 8 and 9 on Arduino Adaptor) 
#define TXP1 1 

#define VSPI_MISO 15 // Defining SPI with Camera on Vision (Pins 10, 11, 12 and 13 on Arduino Adaptor)
#define VSPI_MOSI 4
#define VSPI_SCLK 2
#define VSPI_SS 14

// ---------- MOTOR DRIVING ----------
// motor 1 settings
#define CHA 0
#define ENA 4 //D11 // this pin must be PWM enabled pin if Arduino board is used
#define IN1 16 //D10
#define IN2 14 //D9
// motor 2 settings
#define IN3 17 //D8
#define IN4 15 //D12
#define ENB 2 //D13 // this pin must be PWM enabled pin if Arduino board is used
#define CHB 1
const int CCW = 2; // do not change
const int CW  = 1; // do not change
#define motor1 1 // do not change
#define motor2 2 // do not change
// for two motors without debug information // Watch video https://youtu.be/2JTMqURJTwg
Robojax_L298N_DC_motor robot(IN1, IN2, ENA, CHA,  IN3, IN4, ENB, CHB);

#define PIN_SS        5   // D7
#define PIN_MISO      19  // D5
#define PIN_MOSI      23  // D2
#define PIN_SCK       18  // D6

#define PIN_MOUSECAM_RESET     35
#define PIN_MOUSECAM_CS        5

#define ADNS3080_PIXELS_X                 30
#define ADNS3080_PIXELS_Y                 30

#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_REVISION_ID           0x01
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_PIXEL_SUM             0x06
#define ADNS3080_MAXIMUM_PIXEL         0x07
#define ADNS3080_CONFIGURATION_BITS    0x0a
#define ADNS3080_EXTENDED_CONFIG       0x0b
#define ADNS3080_DATA_OUT_LOWER        0x0c
#define ADNS3080_DATA_OUT_UPPER        0x0d
#define ADNS3080_SHUTTER_LOWER         0x0e
#define ADNS3080_SHUTTER_UPPER         0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    0x10
#define ADNS3080_FRAME_PERIOD_UPPER    0x11
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_SROM_ENABLE           0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_SROM_ID               0x1f
#define ADNS3080_OBSERVATION           0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    0x3f
#define ADNS3080_PIXEL_BURST           0x40
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_SROM_LOAD             0x60

#define ADNS3080_PRODUCT_ID_VAL        0x17

// ---------- OPTICAL SENSOR ----------
int direction = 0;

float argument = 0;

int total_x = 0;
int total_y = 0;

int a=0;
int b=0;

float distance_x=0;
float distance_y=0;

volatile byte movementflag=0;
volatile int xydat[2];

float co_x=0;
float co_y=0;

typedef struct point
{
  float x;
  float y;
} point;

point position;

int convTwosComp(int b)
{
  //Convert from 2's complement
  if(b & 0x80){
    b = -1 * ((b ^ 0xff) + 1);
    }
  return b;
}

int tdistance = 0;

void mousecam_reset()
{
  digitalWrite(PIN_MOUSECAM_RESET,HIGH);
  delay(1); // reset pulse >10us
  digitalWrite(PIN_MOUSECAM_RESET,LOW);
  delay(35); // 35ms from reset to functional
}


int mousecam_init()
{
  pinMode(PIN_MOUSECAM_RESET,OUTPUT);
  pinMode(PIN_MOUSECAM_CS,OUTPUT);

  digitalWrite(PIN_MOUSECAM_CS,HIGH);

  mousecam_reset();
}

void mousecam_write_reg(int reg, int val)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(50);
}

int mousecam_read_reg(int reg)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg);
  delayMicroseconds(75);
  int ret = SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(1);
  return ret;
}

struct MD
{
 byte motion;
 char dx, dy;
 byte squal;
 word shutter;
 byte max_pix;
};


void mousecam_read_motion(struct MD *p)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(ADNS3080_MOTION_BURST);
  delayMicroseconds(75);
  p->motion =  SPI.transfer(0xff);
  p->dx =  SPI.transfer(0xff);
  p->dy =  SPI.transfer(0xff);
  p->squal =  SPI.transfer(0xff);
  p->shutter =  SPI.transfer(0xff)<<8;
  p->shutter |=  SPI.transfer(0xff);
  p->max_pix =  SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(5);
}

// pdata must point to an array of size ADNS3080_PIXELS_X x ADNS3080_PIXELS_Y
// you must call mousecam_reset() after this if you want to go back to normal operation
int mousecam_frame_capture(byte *pdata)
{
  mousecam_write_reg(ADNS3080_FRAME_CAPTURE,0x83);

  digitalWrite(PIN_MOUSECAM_CS, LOW);

  SPI.transfer(ADNS3080_PIXEL_BURST);
  delayMicroseconds(50);

  int pix;
  byte started = 0;
  int count;
  int timeout = 0;
  int ret = 0;
  for(count = 0; count < ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y; )
  {
    pix = SPI.transfer(0xff);
    delayMicroseconds(10);
    if(started==0)
    {
      if(pix&0x40)
        started = 1;
      else
      {
        timeout++;
        if(timeout==100)
        {
          ret = -1;
          break;
        }
      }
    }
    if(started==1)
    {
      pdata[count++] = (pix & 0x3f)<<2; // scale to normal grayscale byte range
    }
  }

  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(14);

  return ret;
}

// ---------- Control Subsystem ----------
WiFiClient client;
HTTPClient http;
SPIClass* vspi = NULL; //Container for VSPI connection

// Message related variables
char Command[32];   // storage for the actual command
char DriveMsg[32];  // storage for drive's message
char VisionMsg[32]; // storage for vision's message

// Autonomous Driving variables
bool autonomous = true;         // boolean representing driver's auto/manual mode

int alien_found = 0; // Number of aliens found
int building_found = 0; // Number of buildings found

// Data flow variables
bool drive_ready = true;        // checks whether drive is ready for receiving command
bool command_ready = false;     // checks whether command is ready for sending command
bool connected = false;         // checks whether the ESP32 has already connected with the server

// Internet related variables
const char* host = "http://34.226.193.83/"; // aws

const char* ssid = "lianderthalin"; // Wifi Name
const char* password = "jahad1027"; // Wifi password

int port = 12000;
int counter = 0;

void initWiFi() {
  // Initialisation
  Serial.println();
  Serial.print("Connecting to network: ");
  Serial.println(ssid);
  WiFi.disconnect(true); // disconnect from wifi to set new wifi connection
  WiFi.mode(WIFI_STA); // Init wifi mode

  // Start connecting to WiFi
  WiFi.begin(ssid, password); // connect to wifi
  Serial.print("Connecting to WiFi ...");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
    counter++;
    if (counter >= 60) { // after 30 seconds timeout - reset board (on unsucessful connection)
      ESP.restart();
    }
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address set: ");
  Serial.println(WiFi.localIP()); // print LAN IP
}


void setup() {
  Serial.begin(115200); // Debugging
  Serial1.begin(115200, SERIAL_8N1, RXP1, TXP1); // Uart with Vision
  
  // vspi = new SPIClass(VSPI); // Initialising VSPI connection
  // vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);
  // vspi->setClockDivider(SPI_CLOCK_DIV8);

  // pinMode(VSPI_SS, OUTPUT);

  pinMode(PIN_SS,OUTPUT);
  pinMode(PIN_MISO,INPUT);
  pinMode(PIN_MOSI,OUTPUT);
  pinMode(PIN_SCK,OUTPUT);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);

  if (mousecam_init()==-1) {
    Serial.println("Mouse cam failed to init");
    while(1);
  }

  //motor setup
  robot.begin();

  initWiFi();
}


char asciiart(int k)
{
  static char foo[] = "WX86*3I>!;~:,`. ";
  return foo[k>>4];
}


byte frame[ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y];


void stop () {
  robot.brake(1);
  robot.brake(2);
}


float clockwise(int speed, int argument) {
  robot.rotate(motor1, speed, CW);    // As with forward the speed is set to a given value
  robot.rotate(motor2, speed, CCW);
  
  float traveled = 0;
  float r = 12.8;  // define a variable for distance travelled
  
  int x1;
  int y1;

  int x;
  int y;

  float arclength = ((argument*M_PI/180)*r);

  while (traveled < arclength) { // stay in loop until the target distance has been reached
    delay(100);

    int val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
    MD md;
    mousecam_read_motion(&md);
    // for(int i=0; i<md.squal/4; i++)
    //   Serial.print('*');
    // Serial.print(' ');
    // Serial.print((val*100)/351);
    // Serial.print(' ');
    // Serial.print(md.shutter); Serial.print(" (");
    // Serial.print((int)md.dx); Serial.print(',');
    // Serial.print((int)md.dy); Serial.println(')');

    // Serial.println(md.max_pix);
    delay(50);

    distance_x = md.dx  ; //convTwosComp(md.dx);
    distance_y = md.dy ; //convTwosComp(md.dy);

    x1 = x1 + distance_x;
    y1 = y1 + distance_y;
    
    x = x1/157;
    y = y1/157;
    
    // Serial.print('\n');
    
    // Serial.println("Distance_x = " + String(total_x));
    
    // Serial.println("Distance_y = " + String(total_y));
    // Serial.print('\n');
    
    traveled = sqrt(y*y + x*x); //update distance travelled
    float argument = (traveled / 2*M_PI*r)*360;
  }

  stop();

  position.x = 0;
  position.y = 0;

  return argument;
}


float anticlockwise(int speed, int argument) {
  robot.rotate(motor1, speed, CCW);    // As with forward the speed is set to a given value
  robot.rotate(motor2, speed, CW);
  
  float traveled = 0;
  float r = 12.8;  //define a variable for distance travelled
  
  int x1;
  int y1;

  int x;
  int y;

  float arclength = ((argument*M_PI/180)*r);

  while (traveled < arclength) {        // stay in loop until the target distance has been reached
    delay(100);

    int val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
    MD md;
    mousecam_read_motion(&md);
    // for(int i=0; i<md.squal/4; i++)
    //   Serial.print('*');
    // Serial.print(' ');
    // Serial.print((val*100)/351);
    // Serial.print(' ');
    // Serial.print(md.shutter); Serial.print(" (");
    // Serial.print((int)md.dx); Serial.print(',');
    // Serial.print((int)md.dy); Serial.println(')');

    // Serial.println(md.max_pix);
    delay(50);

    distance_x = md.dx  ; //convTwosComp(md.dx);
    distance_y = md.dy ; //convTwosComp(md.dy);

    x1 = x1 + distance_x;
    y1 = y1 + distance_y;
    
    x = x1/157;
    y = y1/157;
    
    // Serial.print('\n');
    
    // Serial.println("Distance_x = " + String(total_x));
    
    // Serial.println("Distance_y = " + String(total_y));
    // Serial.print('\n');

    traveled = sqrt(y*y + x*x); //update distance traveled
    float argument = (traveled / 2*M_PI*r)*360;
  }

  stop();

  position.x = 0;
  position.y = 0;

  return argument;
}


void forward (int speed) {
  robot.rotate(motor1, speed, CW);  // set motors to specified speed and forward direction
  robot.rotate(motor2, speed, CW);  // the input is the total distance, thus the target is the 
                                    // current value of y + the desired distance
                                    // stay inside the loop until the target is reached
  direction = 1;
}


void backward (int speed) {
  robot.rotate(motor1, speed, CCW); // set motors to specified speed and forward direction
  robot.rotate(motor2, speed, CCW); // the input is the total distance, thus the target is the 
                                    // current value of y + the desired distance
  direction = -1;
}


point optical_sensor(float x1, float y1) {
    point coordinates;

    int val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
    MD md;
    mousecam_read_motion(&md);
    // for(int i=0; i<md.squal/4; i++)
    //   Serial.print('*');
    // Serial.print(' ');
    // Serial.print((val*100)/351);
    // Serial.print(' ');
    // Serial.print(md.shutter); Serial.print(" (");
    // Serial.print((int)md.dx); Serial.print(',');
    // Serial.print((int)md.dy); Serial.println(')');
  
    distance_x = md.dx; //convTwosComp(md.dx);
    distance_y = md.dy; //convTwosComp(md.dy);

    x1 = x1 + distance_x;
    y1 = y1 + distance_y;
    
    float x = x1/157;
    float y = y1/157;

    coordinates.x = x;
    coordinates.y = y;

    return coordinates;
}

/* 
wall checker integer meanings:
1 is wall north - LEFT
2 is wall east - LEFT
3 is wall south 
4 is wall west - RIGHT

length 21 cm
width  19 cm
*/
int wall_checker(float x, float y, int argument) { 
    if (argument > 360){
        argument = argument - 360;
    }
    else if (argument < -360){
        argument = argument + 360;
    }
    if (x<120 && ((argument > 265 && argument < 275) || (argument > -95 && argument < -85))) { // WEST
        return 2;
    } else if((x>3435)&&((argument > 85 && argument < 95) || (argument > -265 && argument < -275))) { // EAST
        return 1;
    } else if ((y<120)&&((argument < 185 && argument > 175) || (argument > -175 && argument < -185))) { // SOUTH
        return 1;
    } else if ((y>3207)&&(argument > -5 && argument < 5)) { // NORTH
        return 1;
    } else return 0;
}


void loop() {
  // Internet and webserver connection
  if (!connected) {
    if (!client.connect(host, port)) {
      Serial.println("Connection to host failed");
      delay(100);
      return;
    }
    Serial.println("Connected to server successful!");
    client.print("Hello from ESP32!");
    connected = true;
  }

  // Checks if there is any data on the UART Vision datastream
  if (Serial1.available()) {
    // Erase everything in the VisionMsg variable first
    for (int i = 0; i < 32; i++)
      VisionMsg[i] = ' ';

    // Read data on the UART Vision datastream
    for (int i = 0; i < 32; i++)
      VisionMsg[i] = Serial1.read();

    Serial.println("The message from Vision has been recorded.");
  }


  // Checks if there is any data on the http datastream
  /*
  if (client.available() && !command_ready) {
    // read the bytes incoming from the server:
    char Commandinit = client.read();

    if (Commandinit == '[') {
      Command[0] = Commandinit;
      int i = 1;

      while (client.available()) {
        char Commandchar = client.read();

        if (Commandchar != ']') {
          Command[i] = Commandchar;
          i++;
        } else {
          Command[i] = ']';
          Serial.println("The Command has been recorded.");
          command_ready = true;
          break;
        }
      }
    } else if (Commandinit == 'S') {
      Command[0] = Commandinit;

      Serial.println("The Stop signal has been recorded.");
      Serial.print("Sending Stop signal to drive: ");

      Serial.write(Command[0]);
      // Serial2.write(Command[0]);

      Command[0] = ' ';
    }
  }
  */


  int wall = wall_checker(co_x, co_y, argument); // Checks if there is a wall near the rover.

  // Decision Making based on the Vision DataStream when autonomous
  if (VisionMsg[0]==1 & autonomous) {
    // http.begin(client, host, 80);

    // Converting Area and Object Detection data
    std::string area_bin = "";
    std::string obj_bin = "";

    for (int i = 16; i < 32; i++)
      area_bin += std::to_string(VisionMsg[i]);    
    int area = std::stoi(area_bin, 0, 2);
    for (int i = 1; i < 3; i++)
      obj_bin += std::to_string(VisionMsg[i]);
    int obj = std::stoi(obj_bin, 0, 2);

    if (area > 30794 && area < 76100 && obj != 6) { // If an alien is detected
      std::string xcoord_bin = "";

      for (int i = 4; i < 15; i++) 
        xcoord_bin += std::to_string(VisionMsg[i]);
      int xcoord = std::stoi(xcoord_bin, 0, 2);
      
      switch (obj) { // TODO: send data accordingly.
        case 0: // RED
          break;
        case 1: // YELLOW
          break;
        case 2: // MAGENTA
          break;
        case 3: // TEAL
          break;
        case 4: // LIME
          break;
        case 5: // NAVY
          break;
        case 6: // building
          break;
        
        default:
          break;
      }

      // // Specify content-type header
      // http.addHeader("Content-Type", "application/x-www-form-urlencoded");
      // // Data to send with HTTP POST
      // String httpRequestData = "api_key=tPmAT5Ab3j7F9&sensor=BME280&value1=24.25&value2=49.54&value3=1005.14";           
      // // Send HTTP POST request
      // int httpResponseCode = http.POST(httpRequestData);
     
      // Serial.print("HTTP Response code: ");
      // Serial.println(httpResponseCode);
        
      // Free resources
      // http.end();
      
      if (xcoord > 320) { // If the alien is found on the right, TURN LEFT
        anticlockwise(60, 90);
      }
      else { // If the alien is found on the right, TURN RIGHT
        clockwise(60, 90);
      }
    }
    else if (area > 231360 && area < 240100 && obj==6) { // If alien building is detected, TURN LEFT
      anticlockwise(60, 90);
    }
  } else if (autonomous) { // Autonomous Driving without any detection. 
    if (wall==1) {
      anticlockwise(60, 90);
    }
    else if (wall==2) {
      clockwise(60, 90);
    }
    else {
      forward(80);
      position = optical_sensor(position.x, position.y);
      float movement = direction * (position.x*position.x + position.y*position.y);
      co_x = co_x + movement * cos(argument);
      co_y = co_y + movement * sin(argument);
    }
  }

  for (int i = 0; i < 32; i++) {
    Serial.write(DriveMsg[i]);
    // client.write(DriveMsg[i]);

    DriveMsg[i] = ' ';
  }
  Serial.println();
  // client.write('\n');
  // }
  delay(250);
}
