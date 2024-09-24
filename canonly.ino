#include <WiFiUdp.h>
#include <WiFi.h>
#include <math.h>

#include "vive510.h"
#define SIGNALPIN1 32
#define SIGNALPIN2 33
Vive510 vive1(SIGNALPIN1);
Vive510 vive2(SIGNALPIN2);



WiFiUDP robotServer;
WiFiUDP canServer;

IPAddress ipTarget(192,168,1,255);

unsigned int PortCan=1510;
unsigned int PortRobo=2510;
const char* ssid="TP-Link_05AF";
const char* password="47543454";


float duty=0; //motor speed

int id;

int xcan=0; //can locations
int ycan=0;
int x[2];
int y[2];
int xavg;
int yavg;
double angleorient;
double anglefinal;
int wallx[2];
int wally[2];
int FS=0;
int ypointer;
int xpointer;
int xcenter=1000;
int ycenter=1000;
int ypointer2;
int marker=0;
int braker=0;
double anglesave;
int pos;
int brake_counter=0;
int brake_limit=10;

void setup() {
  // put your setup code here, to run once:
vive1.begin();
vive2.begin();
wallx[0]=1520;
wallx[1]=6000;
wally[0]=2800;
wally[1]=5500;
Serial.begin(115200);
udpinitiate();
motorinitiate();

FS=1;
 delay(5000);
}

void loop() {
handleCanMsg(); //data accumulation functions
viveread1();    //data accumulation
viveread2();    //data accumulation
 Serial.println("Vive 1: ");
  Serial.println(x[0]);
  Serial.println(y[0]);
  Serial.println("Vive 2: ");
  Serial.println(x[1]);
  Serial.println(y[1]);
if(x[1]==0||x[0]==0||xcan==0||ycan==0)
FS=0;
else if(FS==0)
FS=1;
xavg=(x[1]+x[0])/2;
yavg=(y[1]+y[0])/2;
handleRobotMsg();
angleorient=angleread(x[0]-x[1],y[0]-y[1]); //finds out orientation
Serial.println("FS:   ");
Serial.print(FS);
if(FS==0)
{
  delay(100);
}

else if(FS==1)
{
  Serial.println("orienting to can");
  anglefinal=angleread(xcan-x[0],ycan-y[0]);
  Serial.println(anglefinal); //for debugging
  Serial.println(angleorient);
  if(marker!=0) //marker checks for first read
  {
  if(abs(anglesave-angleorient)<=0.5)//checks for vive lag
  {
    brake();
    braker=1; //brakes the system to wait vive to read updated values
    Serial.println("braked");
  }
  else
  braker=0; 
  }
  else if(marker==0)//if first read
  {
    braker=0;
    marker=1;
  }
  if(braker==1)//if braked
  {
    brake_counter++; //failsafe to prevent system from being infinitely braking
    if(brake_counter>=brake_limit)
    {
      braker=0;
      brake_counter=0;
    }
  }
  if(braker==0) //if braked
  {
   if(abs(anglefinal-angleorient)<=100)//checks for exterior angle or interior angle
  {
    if((anglefinal-angleorient)>0) 
    LeftOrient(anglefinal-angleorient); 
    else(RightOrient(anglefinal-angleorient));
  }
  else
  {
    if((anglefinal-angleorient)>0)
    RightOrient(anglefinal-angleorient);
    else(LeftOrient(anglefinal-angleorient));
  }
  }
  anglesave=angleorient;
  if((abs(anglefinal-angleorient))<2) //if angle is close to final angle by 2% error
  {
  FS=2;
  marker=0;
  braker=0;
  Serial.println(FS);
  brake_counter=0;
  }
}
else if(FS==2) 
{
  Serial.println("moving straight");
  
 
 if(marker!=0)
  {
  if(abs(pos-y[0])<=0.5)//if vive doesnt update value, brake
  {
    brake();
    braker=1;
    Serial.println("braked");
  }
  else
  braker=0;
  }
  else if(marker==0) 
  {
    braker=0;
    marker=1;
  }
  if(braker==1)
  {
    brake_counter++;
    if(brake_counter>=brake_limit)
    {
      braker=0;
      brake_counter=0;
    }
  }
  if(braker==0)
  {
  straight();
  
  }
  pos=y[0];
  if((abs(y[0]-ycan))<=20)//if in front of can by roughly 20mm
  {
  brake();
  FS=1;
  pos=0;
  marker=0;
  braker=0;
  brake_counter=0;
  }

}


}





double angleread(int x, int y) //find angle of orientation, and interpolate value from 0 to 100
{
  return(atan2(x,y)*100.0f/3.14); //scales from 0 to 100
}


void viveread1()//pin 32
{
  if (vive1.status() == VIVE_LOCKEDON) {
x[0]=vive1.xCoord();
y[0]=vive1.yCoord();
}
else
vive1.sync(15);
}

void viveread2()//pin 33
{
  if (vive2.status() == VIVE_LOCKEDON) {
x[1]=vive2.xCoord();
y[1]=vive2.yCoord();
}
else
vive2.sync(15);
}





void LeftOrient(double anglediff)//left side forward, right side backward
{
  double mag=fabs(anglediff);
  
  duty=0.8;
  ledcWrite(1,duty*4096);
 digitalWrite(23,HIGH);
 digitalWrite(19,LOW);
 digitalWrite(10,HIGH);
 digitalWrite(18,LOW);
  delay(50);
  brake();
}

void RightOrient(double anglediff) //right side forward, left side backward
{
  double mag=fabs(anglediff);

  duty=0.8;
  ledcWrite(1,duty*4096);
 digitalWrite(19,HIGH);
 digitalWrite(23,LOW);
 digitalWrite(18,HIGH);
 digitalWrite(10,LOW);
  delay(50);
  brake();
  
}

void straight() //both sides forward
{
duty=1;
ledcWrite(1,duty*4096);
digitalWrite(19,LOW);
digitalWrite(23,HIGH);
digitalWrite(18,HIGH);
digitalWrite(10,LOW);
delay(50);
}

void brake()//duty cycle 0
{
duty=0;
ledcWrite(1,duty*4096); 
}

void handleCanMsg() {
const int UDP_PACKET_SIZE = 14; // can be up to 65535
uint8_t packetBuffer[UDP_PACKET_SIZE];
int cb = canServer.parsePacket(); // if there is no message cb=0
if (cb) {
packetBuffer[cb]=0; // null terminate string
canServer.read(packetBuffer, UDP_PACKET_SIZE);
xcan = atoi((char *)packetBuffer+2); // #,####,#### 2nd indexed char
ycan = atoi((char *)packetBuffer+7); // #,####,#### 7th indexed char
Serial.print("From Can ");
Serial.println((char *)packetBuffer);
Serial.println(xcan);
Serial.println(ycan);

}
}

void udpinitiate()//initiate USP
{
 pinMode(25,OUTPUT); //switch to choose ID
pinMode(26,OUTPUT);
digitalWrite(25,HIGH);
digitalWrite(26,HIGH);

IPAddress ipLocal(192,168,1,104);
if(digitalRead(25)==1&&digitalRead(26)==1)
id=1;
else if(digitalRead(25)==0&&digitalRead(26)==1)
id=2;
else if(digitalRead(25)==1&&digitalRead(26)==0)
id=3;
else if (digitalRead(25)==0&&digitalRead(26)==0)
id=4;
delay(2000);
Serial.println("Connecting to: "); Serial.println(ssid);
WiFi.config(ipLocal, IPAddress(192,168,1,1),IPAddress(255,255,255,0));
WiFi.begin(ssid,password);
canServer.begin(PortCan);
robotServer.begin(PortRobo); 
}

void motorinitiate()//initiate motor pins
{
pinMode(19,OUTPUT);//motor driver logic 1A
pinMode(23,OUTPUT);//motor driver logic 1B
pinMode(18,OUTPUT);//motor driver logic 2A
pinMode(10,OUTPUT);//motor driver logic 2B


digitalWrite(19,HIGH);//right side
digitalWrite(23,LOW);
digitalWrite(18,HIGH);//left side
digitalWrite(10,LOW); //default motor logic to avoid shorting

ledcSetup(1,1000,12);
ledcAttachPin(9,1); //pin 9 for pwm
ledcWrite(1,duty*4096);
  
}

void handleRobotMsg()//transmit robot location and id
{
  char s[13];
  sprintf(s,"%1d:%4d,%4d",id,xavg,yavg);
  robotServer.beginPacket(ipTarget,PortRobo);
  robotServer.write((uint8_t *)s,13);
  robotServer.endPacket();
}
