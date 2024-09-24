

#include "vive510.h"
#include <math.h>
#define SIGNALPIN1 32
#define SIGNALPIN2 33
Vive510 vive1(SIGNALPIN1); //program uses vive to measure robot's orientation for scanning (got approval from TA)
Vive510 vive2(SIGNALPIN1);

float duty=0.8; //motor speed
int IRstatus=0; //IR Mode
int distance=0; //dist from senso
int x[2];
int y[2];
double angleorient;
double anglefinal;
int wallx[2];
int wally[2];
int modef=1;
int xfinal; //distance to be moved to scan again
int switchx=0;
int spintrigger=0; //scanning switch
double anglesave;
int save=0;

void setup() {
  // put your setup code here, to run once:

vive1.begin();
vive2.begin();
wallx[0]=1520;
wallx[1]=6000;
wally[0]=2800;
wally[1]=5500;

pinMode(35,INPUT); //for IR sensor R
pinMode(34,INPUT); //for IR sensor L

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

void loop() {
  // put your main code here, to run repeatedly:
viveread1();    //data accumulation
viveread2();    //data accumulation
angleorient=angleread(x[0]-x[1],y[0]-y[1]); //check orientation with vive
IRstatus=IRcheck(); //check if IR is detected
if(IRstatus==1)
{
left(); //move left if left eye senses beacon
spintrigger=0;  //turn off scan
}
else if(IRstatus==2)
{
right(); //move right if right eye senses beacon
spintrigger=0; //turn off scan
}
else if(IRstatus==3)
{
  straight(); //move straight if both eyes sense beacon
  spintrigger=0; //turn off scan
  
  
}
else if(IRstatus==0)
{
  if(modef==1)
  {
    if(abs(x[1]-wallx[0])>1000) //find out which end of the field robot is on
      anglefinal=angleread(wallx[0]-x[1],0); //modef decides which side the robot will move to for next scan
    else
      modef=2;
  }
  if(modef==2)
  {
    if(abs(x[1]-wallx[0])>200)
      anglefinal=angleread(wallx[1]-x[1],0);
    else
      modef=1;
  }

  if(abs(anglefinal-angleorient)<=100) //move towards the end of the arena
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
  if(anglefinal==angleorient) 
  {
    if(switchx==0)//switchx determines if the robot is moving forward for next scan
    {
      if(modef==1)
        xfinal=x[1]-100; //determine direction the robot is moving (is x increasing or decreasing?)
      else
        xfinal=x[1]+100;
      switchx=1;
    }
    straight();
    if(modef==1)
    {
      if(x[1]<=xfinal) //move to new x location
      {
        switchx=0;
        spintrigger=1;
      }
    }
    else if(modef==2)
    {
      if(x[1]>=xfinal)
      {
        switchx=0;
        spintrigger=1;
      }
    }
  }
 if(spintrigger==1) //perform scan
 {
  if(save==0) //inital orientation
  {
    anglesave=angleorient;
    save=1;
  }
  else if(save==1) //check if 360 degree scan is complete
  {
  if(abs(anglesave-angleorient)<=5)
  {
    spintrigger=0;
    save=0;
  }
  }
 }
 if(spintrigger==1)
 left();
}
  
 }
  


void LeftOrient(double anglediff)
{
  double mag=fabs(anglediff);
  duty=(mag/100)*(1.0-0.8)+0.8;
  ledcWrite(1,duty*4096);
  digitalWrite(19,LOW);
  digitalWrite(23,HIGH);
  digitalWrite(18,HIGH);
  digitalWrite(10,LOW);
}
void RightOrient(double anglediff)
{
  double mag=fabs(anglediff);
  duty=(mag/100)*(1.0-0.8)+0.8;
  ledcWrite(1,duty*4096);
  digitalWrite(19,LOW);
  digitalWrite(23,HIGH);
  digitalWrite(18,HIGH);
  digitalWrite(10,LOW);
}




double angleread(int x, int y)
{
  return(atan2(x,y)*100.0f/3.14); //scales from 0 to 100
}


void viveread1()
{
  if (vive1.status() == VIVE_LOCKEDON) {
x[0]=vive1.xCoord();
y[0]=vive1.yCoord();
}
else
vive1.sync(15);
}

void viveread2()
{
  if (vive2.status() == VIVE_LOCKEDON) {
x[1]=vive2.xCoord();
y[1]=vive2.yCoord();
}
else
vive2.sync(15);
}
void left() //right forward, left backward
{
duty=0.8;
ledcWrite(1,duty*4096);
digitalWrite(19,LOW);
digitalWrite(23,HIGH);
digitalWrite(18,HIGH);
digitalWrite(10,LOW);
}


void right() //right backward, left forward
{
duty=0.8;
ledcWrite(1,duty*4096);
digitalWrite(19,HIGH);
digitalWrite(23,LOW);
digitalWrite(18,LOW);
digitalWrite(10,HIGH);
}

void straight() //both forward
{
duty=1;
ledcWrite(1,duty*4096);
                digitalWrite(19,LOW);
                digitalWrite(23,HIGH);
                digitalWrite(18,LOW);
                digitalWrite(10,HIGH);
}

void brake()
{
duty=0;
ledcWrite(1,duty*4096); 
}


int IRcheck()
{
  if(digitalRead(34)==0&&digitalRead(35)==0)
  return 0;
  else if(digitalRead(34)==1&&digitalRead(35)==0)
  return 1;
  else if(digitalRead(34)==0&&digitalRead(35)==1)
  return 2;
  else if(digitalRead(34)==1&&digitalRead(35)==1)
  return 3;
}
