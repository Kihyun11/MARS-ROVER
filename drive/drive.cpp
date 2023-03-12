#include <Arduino.h>
#include "SPI.h"

// these pins may be different on different boards



//motor control 

#include <Robojax_L298N_DC_motor.h>
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
// for two motors with debug information
//Robojax_L298N_DC_motor robot(IN1, IN2, ENA, CHA, IN3, IN4, ENB, CHB, true);






































#define PIN_SS        5  //7
#define PIN_MISO      19 //5
#define PIN_MOSI      23  //2
#define PIN_SCK       18  //6

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





int argument = 0;

int total_x = 0;
int total_y = 0;


int total_x1 = 0;
int total_y1 = 0;


int x=0;
int y=0;

int a=0;
int b=0;

int distance_x=0;
int distance_y=0;

volatile byte movementflag=0;
volatile int xydat[2];

float co_x=0;
float co_y=0;


int convTwosComp(int b){
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

void setup()
{
  pinMode(PIN_SS,OUTPUT);
  pinMode(PIN_MISO,INPUT);
  pinMode(PIN_MOSI,OUTPUT);
  pinMode(PIN_SCK,OUTPUT);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);

  Serial.begin(9600);

  if(mousecam_init()==-1)
  {
    Serial.println("Mouse cam failed to init");
    while(1);
  }


  //motor setup
  robot.begin();
}

char asciiart(int k)
{
  static char foo[] = "WX86*3I>!;~:,`. ";
  return foo[k>>4];
}

byte frame[ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y];





float clockwise(int speed, int argument){
  robot.rotate(motor1, speed, CW);    //for clockwise rotation, one motor will rotate clockwise
  robot.rotate(motor2, speed, CCW);   //and one will rotate anti clockwise
  

  float traveled = 0;
  float r = 13.7;  //this is a variable for the radium of the hypothetical circle that the rover
                   //will traverse, from the midpoint of the axis from one wheel to the other
  int x1;          //to the centre of the optical sensor.
  int y1;

  int x;
  int y;

  
  float arclength = ((argument*M_PI/180)*r); //calculating the arc that the rover will traverse 
                                             //based on desired turning angle
  
  
  while(traveled < arclength){        //stay in loop until the target distance has
                                      //been reached


  delay(100);

   
  int val = mousecam_read_reg(ADNS3080_PIXEL_SUM);  //checking image quality
  MD md;
  mousecam_read_motion(&md);
  for(int i=0; i<md.squal/4; i++)
    Serial.print('*');
  Serial.print(' ');
  Serial.print((val*100)/351);
  Serial.print(' ');
  Serial.print(md.shutter); Serial.print(" (");
  Serial.print((int)md.dx); Serial.print(',');
  Serial.print((int)md.dy); Serial.println(')');

  // Serial.println(md.max_pix);
  delay(50);


    distance_x = md.dx  ; //convTwosComp(md.dx);
    distance_y = md.dy ; //convTwosComp(md.dy);

    x1 = x1 + distance_x;
    y1 = y1 + distance_y;
    
    x = x1/157;
    y = y1/157;
    
    
    Serial.print('\n');
    
    
    Serial.println("Distance_x = " + String(total_x));
    
    Serial.println("Distance_y = " + String(total_y));
    Serial.print('\n');
    
    

    traveled = sqrt(y*y + x*x); //update distance traveled
    float argument = (traveled / 2*M_PI*r)*180; //return actual distance traveled 
    
    
  }

  robot.brake(1); //stop the motors
  robot.brake(2);
  return argument;
  delay(100000);
}



float anticlockwise(int speed, int argument){
  robot.rotate(motor1, speed, CCW);    //As with forward the speed is set to a given value
  robot.rotate(motor2, speed, CW);
  

  float traveled = 0;
  float r = 13.7;  //define a variable for distance travelled
  
  int x1;
  int y1;

  int x;
  int y;

  
  float arclength = ((argument*M_PI/180)*r);

  
  
  while(traveled < arclength){        //stay in loop until the target distance has
                                      //been reached


  delay(100);


  int val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
  MD md;
  mousecam_read_motion(&md);
  for(int i=0; i<md.squal/4; i++)
    Serial.print('*');
  Serial.print(' ');
  Serial.print((val*100)/351);
  Serial.print(' ');
  Serial.print(md.shutter); Serial.print(" (");
  Serial.print((int)md.dx); Serial.print(',');
  Serial.print((int)md.dy); Serial.println(')');

  // Serial.println(md.max_pix);
  delay(50);


    distance_x = md.dx  ; //convTwosComp(md.dx);
    distance_y = md.dy ; //convTwosComp(md.dy);

    x1 = x1 + distance_x;
    y1 = y1 + distance_y;
    
    x = x1/157;
    y = y1/157;
    
    
    Serial.print('\n');
    
    
    Serial.println("Distance_x = " + String(total_x));
    
    Serial.println("Distance_y = " + String(total_y));
    Serial.print('\n');
    
    

    traveled = sqrt(y*y + x*x); //update distance traveled
    float argument = (traveled / 2*M_PI*r)*180;
    
    
  }

  robot.brake(1); //stop the motors
  robot.brake(2);
  return argument;
  delay(100000);
}






  



float forward (int speed) {
    robot.rotate(motor1, speed, CW); //set motors to specified speed and forward direction
    robot.rotate(motor2, speed, CW); // //the input is the total distance, thus the target is the 
                                     //current value of y + the desired distance
           
    int y=0;
    int x=0;
    int x1=0;
    int y1=0;
                         
    bool brake = false;
  
  while(brake == false){
   
    delay(250);

      int val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
  MD md;
  mousecam_read_motion(&md);
  for(int i=0; i<md.squal/4; i++)
    Serial.print('*');
  Serial.print(' ');
  Serial.print((val*100)/351);
  Serial.print(' ');
  Serial.print(md.shutter); Serial.print(" (");
  Serial.print((int)md.dx); Serial.print(',');
  Serial.print((int)md.dy); Serial.println(')');
  
     

  
  
      distance_x = md.dx; //convTwosComp(md.dx);
      distance_y = md.dy; //convTwosComp(md.dy);
  
      x1 = x1 + distance_x;
      y1 = y1 + distance_y;
      
      x = x1/157;
      y = y1/157;
      
      
      Serial.print('\n');
      
      
      Serial.println("Distance_x = " + String(x));
      
      Serial.println("Distance_y = " + String(y));
      Serial.print('\n');
  
      brake = checkBrake(sqrt(x*x + y*y));

    
      
   } //exit the loop 
      robot.brake(motor1);
      robot.brake(motor2);
      return sqrt(x*x + y*y);
    
}







float backward (int speed) {
    robot.rotate(motor1, speed, CCW); //set motors to specified speed and forward direction
    robot.rotate(motor2, speed, CCW); 
                                     
    int y=0;
    int x=0;
    int x1=0;
    int y1=0;
                         
    bool brake = false;
  
  while(brake == false){
   
    delay(250);

      int val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
  MD md;
  mousecam_read_motion(&md);
  for(int i=0; i<md.squal/4; i++)
    Serial.print('*');
  Serial.print(' ');
  Serial.print((val*100)/351);
  Serial.print(' ');
  Serial.print(md.shutter); Serial.print(" (");
  Serial.print((int)md.dx); Serial.print(',');
  Serial.print((int)md.dy); Serial.println(')');
  
     

  
  
      distance_x = md.dx; //convTwosComp(md.dx);
      distance_y = md.dy; //convTwosComp(md.dy);
  
      x1 = x1 + distance_x;
      y1 = y1 + distance_y;
      
      x = x1/157;
      y = y1/157;
      
      
      Serial.print('\n');
      
      
      Serial.println("Distance_x = " + String(x));
      
      Serial.println("Distance_y = " + String(y));
      Serial.print('\n');
  
      brake = checkBrake(sqrt(x*x + y*y));

    
      
   } //exit the loop 
      
    
}




bool checkBrake(int d){   //this will need to be implemented depenending
  if (d>10)               //on how you want to giv the command 
    return true ;
  
  else return false;
}









void loop()
{
 

 
  
  Serial.println("m`in loop");
  delay(500);

  
  //calculating coordinates, order types implemented in control.
  if (order.type == "forward"){
    float d = forward(order.speed)
    co_x = co_x + d*sin(argument);
    co_y = co_y + d*cos(argument);
  }

  if (order.type == "backward"){
    float d = forward(order.speed)
    co_x = co_x - d*sin(argument);
    co_y = co_y - d*cos(argument);
  }

  if (order.type == "clockwise"){
    float angleMoved = clockwise(speed, argument);
    argument = argument + angleMoved
  }


  if (order.type == String("clockwise")){
    float angleMoved = clockwise(speed, angle);
    argument = argument - angleMoved;
  }
  
}
  