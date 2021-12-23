/*
  The circuit:
 * HEDGEHOG serial data to digital pin 0 (RXD)
 * LCD RS pin to digital pin 8
 * LCD Enable pin to digital pin 9
 * LCD D4 pin to digital pin 4
 * LCD D5 pin to digital pin 5
 * LCD D6 pin to digital pin 6
 * LCD D7 pin to digital pin 7
 * LCD BL pin to digital pin 10
 *Vcc pin to  +5
 *HC-SR04 Distance Sensor Trig pin to digital pin 2
 *HC-SR04 Distance Sensor Echo pin to digital pin 3
 */

#include <stdlib.h>
#include <math.h>
#include <Wire.h>
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//  MARVELMIND HEDGEHOG RELATED PART

long hedgehog_x, hedgehog_y;// coordinates of hedgehog (X,Y), mm
long hedgehog_z;// height of hedgehog, mm
int hedgehog_pos_updated;// flag of new data from hedgehog received

bool high_resolution_mode;

///

//#define DISTANCE_SENSOR_ENABLED

#define HEDGEHOG_BUF_SIZE 40 
#define HEDGEHOG_CM_DATA_SIZE 0x10
#define HEDGEHOG_MM_DATA_SIZE 0x16
byte hedgehog_serial_buf[HEDGEHOG_BUF_SIZE];
byte hedgehog_serial_buf_ofs;

#define POSITION_DATAGRAM_ID 0x0001
#define POSITION_DATAGRAM_HIGHRES_ID 0x0011
unsigned int hedgehog_data_id;

typedef union {byte b[2]; unsigned int w;int wi;} uni_8x2_16;
typedef union {byte b[4];float f;unsigned long v32;long vi32;} uni_8x4_32;

//    Marvelmind hedgehog support initialize
void setup_hedgehog() 
{
  Serial.begin(9600); // hedgehog transmits data on 9600bps  

  hedgehog_serial_buf_ofs= 0;
  hedgehog_pos_updated= 0;
}

// Marvelmind hedgehog service loop
void loop_hedgehog()
{int incoming_byte;
 int total_received_in_loop;
 int packet_received;
 bool good_byte;
 byte packet_size;
 uni_8x2_16 un16;
 uni_8x4_32 un32;

  total_received_in_loop= 0;
  packet_received= 0;
  
  while(Serial.available() > 0)
    {
      
      if (hedgehog_serial_buf_ofs>=HEDGEHOG_BUF_SIZE) 
      {
        hedgehog_serial_buf_ofs= 0;// restart bufer fill
        break;// buffer overflow
      }
      total_received_in_loop++;
      if (total_received_in_loop>100) break;// too much data without required header
      
      incoming_byte= Serial.read();
     
      good_byte= false;
      switch(hedgehog_serial_buf_ofs)
      {
        case 0:
        {
          good_byte= (incoming_byte = 0xff);
          break;
        }
        case 1:
        {
          good_byte= (incoming_byte = 0x47);
          break;
        }
        case 2:
        {
          good_byte= true;
          break;
        }
        case 3:
        {
          hedgehog_data_id= (((unsigned int) incoming_byte)<<8) + hedgehog_serial_buf[2];
          good_byte=   (hedgehog_data_id == POSITION_DATAGRAM_ID) ||
                       (hedgehog_data_id == POSITION_DATAGRAM_HIGHRES_ID);
          break;
        }
        case 4:
        {
          switch(hedgehog_data_id)
          {
            case POSITION_DATAGRAM_ID:
            {
              good_byte= (incoming_byte == HEDGEHOG_CM_DATA_SIZE);
              break;
            }
            case POSITION_DATAGRAM_HIGHRES_ID:
            {
              good_byte= (incoming_byte == HEDGEHOG_MM_DATA_SIZE);
              break;
            }
          }
          break;
        }
        default:
        {
          good_byte= true;
          break;
        }
      }
      
      if (!good_byte)
        {
          hedgehog_serial_buf_ofs= 0;// restart bufer fill         
          continue;
        }     
      hedgehog_serial_buf[hedgehog_serial_buf_ofs++]= incoming_byte; 
      if (hedgehog_serial_buf_ofs>5)
        {
          packet_size=  7 + hedgehog_serial_buf[4];
          if (hedgehog_serial_buf_ofs == packet_size)
            {// received packet with required header
              packet_received= 1;
              hedgehog_serial_buf_ofs= 0;// restart bufer fill
              break; 
            }
        }
    }

  if (packet_received)  
    {
      hedgehog_set_crc16(&hedgehog_serial_buf[0], packet_size);// calculate CRC checksum of packet
      if ((hedgehog_serial_buf[packet_size] == 0)&&(hedgehog_serial_buf[packet_size+1] == 0))
        {// checksum success
          switch(hedgehog_data_id)
          {
            case POSITION_DATAGRAM_ID:
            {
              // coordinates of hedgehog (X,Y), cm ==> mm
              un16.b[0]= hedgehog_serial_buf[9];
              un16.b[1]= hedgehog_serial_buf[10];
              hedgehog_x= 10*long(un16.wi);
              un16.b[0]= hedgehog_serial_buf[11];
              un16.b[1]= hedgehog_serial_buf[12];
              hedgehog_y= 10*long(un16.wi);              
              // height of hedgehog, cm==>mm (FW V3.97+)
              un16.b[0]= hedgehog_serial_buf[13];
              un16.b[1]= hedgehog_serial_buf[14];
              hedgehog_z= 10*long(un16.wi);
              
              hedgehog_pos_updated= 1;// flag of new data from hedgehog received
              high_resolution_mode= false;
              break;
            }

            case POSITION_DATAGRAM_HIGHRES_ID:
            {
              // coordinates of hedgehog (X,Y), mm
              un32.b[0]= hedgehog_serial_buf[9];
              un32.b[1]= hedgehog_serial_buf[10];
              un32.b[2]= hedgehog_serial_buf[11];
              un32.b[3]= hedgehog_serial_buf[12];
              hedgehog_x= un32.vi32;

              un32.b[0]= hedgehog_serial_buf[13];
              un32.b[1]= hedgehog_serial_buf[14];
              un32.b[2]= hedgehog_serial_buf[15];
              un32.b[3]= hedgehog_serial_buf[16];
              hedgehog_y= un32.vi32;
              
              // height of hedgehog, mm 
              un32.b[0]= hedgehog_serial_buf[17];
              un32.b[1]= hedgehog_serial_buf[18];
              un32.b[2]= hedgehog_serial_buf[19];
              un32.b[3]= hedgehog_serial_buf[20];
              hedgehog_z= un32.vi32;
              
              hedgehog_pos_updated= 1;// flag of new data from hedgehog received
              high_resolution_mode= true;
              break;
            }
          }
        } 
    }
}

// Calculate CRC-16 of hedgehog packet
void hedgehog_set_crc16(byte *buf, byte size)
{uni_8x2_16 sum;
 byte shift_cnt;
 byte byte_cnt;

  sum.w=0xffffU;

  for(byte_cnt=size; byte_cnt>0; byte_cnt--)
   {
   sum.w=(unsigned int) ((sum.w/256U)*256U + ((sum.w%256U)^(buf[size-byte_cnt])));

     for(shift_cnt=0; shift_cnt<8; shift_cnt++)
       {
         if((sum.w&0x1)==1) sum.w=(unsigned int)((sum.w>>1)^0xa001U);
                       else sum.w>>=1;
       }
   }

  buf[size]=sum.b[0];
  buf[size+1]=sum.b[1];// little endian
}// hedgehog_set_crc16

//  END OF MARVELMIND HEDGEHOG RELATED PART
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//#include "WiFi.h"
//#include <WebServer.h>
#define CM 1      //Centimeter
#define INC 0     //Inch
#define TP 2      //Trig_pin
#define EP 3      //Echo_pin


//=================宣告初始變數===========================//

int desk1_x=100;//desk1 position
int desk1_y=100;
int save_x;       //save x from the past
int save_y;
int theta_current;
int theta_target;
int break_range; //distanse that the car will break near target
int break_coefficient;//break level
//=======================================================//

void setup() 
{
  Serial.begin(115200);
//======hedgehog=========================
  setup_hedgehog();//    Marvelmind hedgehog support initialize
//===============================

//======I2C=======================================
  Wire.begin(3); //I2C位址與連線設定，這裡設定為3號
  Wire.onReceive(receiveEvent); //當收到I2C訊號時，啟動的動作
//=================================================


}



void loop() 
{
  int x=20;//car is moving,recent position
  int y=30;

  calculate_now_angle(hedgehog_x,hedgehog_y,save_x,save_y);
  calculate_target_angle(hedgehog_x,hedgehog_y,desk1_x,desk1_y);
  check_if_arrive(hedgehog_x,hedgehog_y,desk1_x,desk1_y);
//======hedgehog================================
  byte lcd_coord_precision;
   char lcd_buf[12];
#ifdef DISTANCE_SENSOR_ENABLED
   long microseconds = TP_init();
   long distacne_cm = Distance(microseconds, CM);
   lcd.setCursor(10,0); 
   lcd.print("D="); 
   lcd.print(distacne_cm); 
   lcd.print("  "); 
#endif

   loop_hedgehog();// Marvelmind hedgehog service loop

   if (hedgehog_pos_updated)
     {// new data from hedgehog available
       hedgehog_pos_updated= 0;// clear new data flag 
       // output hedgehog position to LCD
       if (high_resolution_mode)
        {
          lcd_coord_precision= 3;
        }
       else
        {
          lcd_coord_precision= 2; 
        }
       
      
     }
//================================================

//======存上個xy=======================================//
  save_x=hedgehog_x;
  save_y=hedgehog_y;
//=============================================//
delay(100);
}



//======hedgehog===========================================================================
long Distance(long time, int flag)
{
  /*
  
  */
  long distacne;
  if(flag)
    distacne = time /29 / 2  ;     // Distance_CM  = ((Duration of high level)*(Sonic :340m/s))/2
                                   //              = ((Duration of high level)*(Sonic :0.034 cm/us))/2
                                   //              = ((Duration of high level)/(Sonic :29.4 cm/us))/2
  else
    distacne = time / 74 / 2;      // INC
  return distacne;
}

long TP_init()
{                     
  digitalWrite(TP, LOW);                    
  delayMicroseconds(2);
  digitalWrite(TP, HIGH);                 // pull the Trig pin to high level for more than 10us impulse 
  delayMicroseconds(10);
  digitalWrite(TP, LOW);
  long microseconds = pulseIn(EP,HIGH);   // waits for the pin to go HIGH, and returns the length of the pulse in microseconds
  return microseconds;                    // return microseconds
}
//==============================================================================================



//=================計算現在的角度=============================//
void calculate_now_angle(int a,int b,int c,int d)
{
  int dx=a-c;//delta x
  int dy=b-d;
  int tangent_value=dy/dx;
  theta_current=atan (tangent_value);  // arc tangent of x
}
//===========================================================//

//=================計算到目標的角度===========================//
void calculate_target_angle(int a,int b,int c,int d)
{
  int dx=c-a;//delta x
  int dy=d-b;
  int tangent_value=dy/dx;
  theta_target=atan (tangent_value);  // arc tangent of x
}
//==========================================================//




//======距離控制======================================================//

void check_if_arrive(int a,int b,int c,int d)
{ 
  int x_abs = c-a;
  int y_abs = d-b;
  x_abs = abs(x_abs);
  y_abs = abs(y_abs); //距離絕對值

  if( x_abs < break_range && y_abs < break_range ) //when distance < range
  {
    break_coefficient = 1/(x_abs + y_abs); //break harder when approaching target
  }
}
//================================================================//
//======I2C==========================================
void receiveEvent(int numBytes)
{
  while(Wire.available()){ //判斷Wire.available()有沒有訊號
  int c = Wire.read(); //將傳入的訊號Wire.read()指定給int C
  Serial.print(c); //透過Serial印出int C的內容
}
}
//=========================================================