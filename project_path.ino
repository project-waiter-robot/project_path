//#include "WiFi.h"
//#include <WebServer.h>
#include <math.h>
//SPI//////////////////////////////////////
#include<SPI.h>
volatile boolean received;
volatile byte Slavereceived,Slavesend;
///////////////////////////////////////////
int desk1_x=100;//desk1 position
int desk1_y=100;
int save_x;       //save x from the past
int save_y;
int theta_current;
int theta_target;
int break_range; //distanse that the car will break near target
int break_coefficient;//break level


void setup() 
{
  int x;//car is moving,recent position
  int y;
  
//SPI////////////////////////////////////////
  pinMode(MISO,OUTPUT);                   //Sets MISO as OUTPUT (Have to Send data to Master IN 

  SPCR |= _BV(SPE);                       //Turn on SPI in Slave Mode
  received = false;

  SPI.attachInterrupt();                  //Interuupt ON is set for SPI commnucation
///////////////////////////////////////////////////

}

//SPI/////////////////////////////////////////
ISR (SPI_STC_vect)                        //Inerrrput routine function 
{
  Slavereceived = SPDR;         // Value received from master if store in variable slavereceived
  received = true;                        //Sets received as True 
}
//////////////////////////////////////////////



void loop() 
{
  int x=20;//car is moving,recent position
  int y=30;

  calculate_now_angle(x,y,save_x,save_y);
  calculate_target_angle(x,y,desk1_x,desk1_y);
  check_if_arrive(x,y,desk1_x,desk1_y);

//SPI////////////////////////////////////////////////////////
if(received)                            //Logic to SET LED ON OR OFF depending upon the value recerived from master
{
  Slavesend=;                             
  SPDR = Slavesend;                           //Sends the x value to master via SPDR 
  delay(1000);
}
///////////////////////////////////////////////////////////////////////////





  save_x=x;
  save_y=y;
}


void calculate_now_angle(int a,int b,int c,int d)
{
  int dx=a-c;//delta x
  int dy=b-d;
  int tangent_value=dy/dx;
  theta_current=atan (tangent_value);  // arc tangent of x
}

void calculate_target_angle(int a,int b,int c,int d)
{
  int dx=c-a;//delta x
  int dy=d-b;
  int tangent_value=dy/dx;
  theta_target=atan (tangent_value);  // arc tangent of x
}

void check_if_arrive(int a,int b,int c,int d)
{ 
  int x_abs = c-a;
  int y_abs = d-b;
  x_abs = abs(x_abs);
  y_abs = abs(y_abs); //距離絕對值

  if( x_abs < break_range && y_abs < break_range ) //when distance < range
  {
    break_coeffition = 1/(x_abs + y_abs); //break harder when approaching target
  }
}

