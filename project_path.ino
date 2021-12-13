//#include "WiFi.h"
//#include <WebServer.h>
#include <math.h>
int desk1_x=100;//desk1 position
int desk1_y=100;
int save_x;
int save_y;
int theta_current;
int theta_target;
int test1111111;

int tedu;
int test;
int test1;
int test165135;
int test148949;


void setup() 
{
  int x;//car is moving,recent position
  int y;
  

}
  
void loop() 
{
  int x=20;//car is moving,recent position
  int y=30;

  calculate_now_angle(x,y,save_x,save_y);
  calculate_target_angle(x,y,desk1_x,desk1_y);



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
