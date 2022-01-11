/***********************************************************
File name:  Adeept_quadruped_robot_infrared.ino
Description:  
1. infrared control mode: control with infrared controller
2. Press different keys to realize the left and
right movement, automatic obstacle avoidance and 
self-stabilization of Quadruped Robot

Website: www.adeept.com
E-mail: support@adeept.com
Author: Felix
Date: 2019/07/01 
***********************************************************/
//add some libraries
#include<Servo.h>
#include <Adafruit_NeoPixel.h>
#include<Wire.h>
#include <SR04.h>
#include <IRremote.h>    
//define pins 
#define RECV_PIN A0
#define PIN  A1
#define led_numbers  6
#define TRIG_PIN A2
#define ECHO_PIN A3
//creat an strip object
Adafruit_NeoPixel strip = Adafruit_NeoPixel(led_numbers, PIN, NEO_GRB + NEO_KHZ800);
//creat an ultrasonic object
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
//creat an IRremote object
IRrecv irrecv(RECV_PIN);   
decode_results results;   
//creat some Servo object
  Servo s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11;
//Declaring some global variables
  char status1 =  0;
  char i1L1 = 0;
  char motorspeed = 20;
  char angle_a;
  char movespeed = 20;
  int angle_judge = 5;
  int anglerange = 5;
/***************adjust the steering gear Angle******************************/
  char angle0 = 90;   //D2
  char angle1 = 90;   //D3
  char angle2 = 90;   //D4
  char angle3 = 90;   //D5
  char angle4 = 90;   //D6
  char angle5 = 90;   //D7
  char angle6 = 90;   //D8
  char angle7 = 90;   //D9
  char angle8 = 90;   //D10
  char angle9 = 90;   //D11
  char angle10 = 90;  //D12
  char angle11 = 90;  //D13
//  char angle0 = 80;   //D2
//  char angle1 = 83;   //D3
//  char angle2 = 90;   //D4
//  char angle3 = 85;   //D5
//  char angle4 = 93;   //D6
//  char angle5 = 90;   //D7
//  char angle6 = 99;   //D8
//  char angle7 = 90;   //D9
//  char angle8 = 90;   //D10
//  char angle9 = 90;   //D11
//  char angle10 = 95;  //D12
//  char angle11 = 90;  //D13
  char angle_judge_max = 5;
  char angle_judge_min = -5;
//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll,angle_yaw;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output,angle_yaw_output;
float angle_changed;
int a;
unsigned long showtime = 0;
char serialcom;
int angle=20;
char right=0;
char count = 0;
long counter=0;
int fbl = 10;
int fbly =3;
int fblrl = 10;
int fblyrl = 1;
int battery_voltage;
double showbattery;
char infrared = 0;
void setup() {
//initialize the steering gear  
  s0.attach(2);     s1.attach(3);     s2.attach(4);     s3.attach(5);     s4.attach(6);     s5.attach(7);     s6.attach(8);     s7.attach(9);     s8.attach(10);    s9.attach(11);    s10.attach(12);     s11.attach(13);
  s0.write(angle0); s1.write(angle1); s2.write(angle2); s3.write(angle3); s4.write(angle4); s5.write(angle5); s6.write(angle6); s7.write(angle7); s8.write(angle8); s9.write(angle9); s10.write(angle10); s11.write(angle11); 
//initialize the strip
  strip.begin();
  for(int i=0;i<6;i++){
    strip.setPixelColor(i,strip.Color(255,210,67));
    strip.show();    
    delay(50);                                              
  } 
  self_balanced_setup();//initialize the MPU6050
  irrecv.enableIRIn(); 
  for(int i=0;i<6;i++){
    strip.setPixelColor(i,strip.Color(55,122,132));
    strip.show();        
    delay(50);                                          
  }
//calculate battery capacity
  battery_voltage =battery_voltage*0.92+((analogRead(A7) * 2.93) + 250)*0.08;
  showbattery = (battery_voltage-190)/2.93*5/1023*6;
}
void loop() {
  battery_voltage = battery_voltage*0.92 + ((analogRead(A7)*2.93)+250)*0.08; 
  IRremote();
}
void moveforward(){
  if(battery_voltage < 860 ){                      //If batteryvoltage is below 10.5V and higher than 8.0V 860
        for(int i=0;i<6;i++){
      strip.setPixelColor(i,strip.Color(255,0,0));
      strip.show();
    delay(1);                                                //Turn on the led if battery voltage is to low                                                    
    }                                                //Turn on the led if battery voltage is to low
  }
  /******************step 1 to step 2************************/
  // Rotate the leg1
  while (status1 <= fbl) {
    status1++;
    i1L1++;
    s0.write(angle0 + i1L1 * fbly);
    s1.write(angle1 + i1L1 * fbly);
    s2.write(angle2-30 + i1L1 * 2);

    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed); 
  }
  while (status1 <= fbl) {
    status1++;
    i1L1++;
    s0.write(angle0+30 - i1L1 * 5);
    s1.write(angle1+30 - i1L1 * 2);
    s2.write(angle2-10 + i1L1);
    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
/****************step 2 to step 3***********************/
  while(status1 <= fbl){
    status1++;
    i1L1++;
    s0.write(angle0-20 + i1L1 * 2);
    s1.write(angle1+10 - i1L1);  
    s5.write(angle5+30 - i1L1 * fbly);
    s8.write(angle8 + i1L1 * fbly);
    s9.write(angle9 - i1L1 * 2);
    s10.write(angle10 + i1L1);   
    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
/****************step 3 to step 4***********************/
  while(status1 <= fbl){
    status1++;
    i1L1++;
    s9.write(angle9-20 + i1L1 * 5);
    s10.write(angle10+10 + i1L1 * 2);
    s11.write(angle11 - i1L1 * 2);
    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
  while(status1 <= fbl){
    status1++;
    i1L1++;
    s9.write(angle9+30 - i1L1 * fbly);
    s10.write(angle10+30 - i1L1 * fbly);
    s11.write(angle11-20 - i1L1);
    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed); 
  }
  /****************step 4 to step 5***********************/
  while(status1 <= fbl){
    status1++;
    i1L1++;
    s6.write(angle6 - i1L1 * fbly);
    s7.write(angle7 - i1L1 * fbly);
    s8.write(angle8+30 - i1L1 * 2); 
    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
  while(status1 <= fbl){
    status1++;
    i1L1++;
    s6.write(angle6-30 + i1L1 * 5);
    s7.write(angle7-30 + i1L1 * 2);
    s8.write(angle8+10 - i1L1);
    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
  /****************step 5 to step 6***********************/
  while(status1 <= fbl){
    status1++;
    i1L1++;
    s2.write(angle2 - i1L1 * fbly);
    s3.write(angle3 + i1L1 * 2);
    s4.write(angle4 - i1L1);
    s6.write(angle6+20 - i1L1 * 2);
    s7.write(angle7-10 + i1L1);
    s11.write(angle11-30 + i1L1 * fbly);
    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
  /****************step 6 to step 1***********************/
  while(status1 <= fbl){
    status1++;
    i1L1++;
    s3.write(angle3+10 - i1L1 * 4);
    s4.write(angle4-10 - i1L1 * 2);
    s5.write(angle5 + i1L1 * 2);
  
    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
   delay(motorspeed);
  }
  while(status1 <= fbl){
    status1++;
    i1L1++;
    s3.write(angle3-30 + i1L1 * fbly);
    s4.write(angle4-30 + i1L1 * fbly);
    s5.write(angle5+20 + i1L1);
  
    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
   }
}
void movebackward(){
  if(battery_voltage > 860){
      for(int i=0;i<6;i++){
      strip.setPixelColor(i,strip.Color(55,122,132));
      strip.show(); 
      delay(1);                                              
     }
    }
  /****************step 6 to step 1***********************/
  while(status1 <= fbl){
    status1++;
    i1L1++;
    s3.write(angle3 - i1L1 * fbly);
    s4.write(angle4 - i1L1 * fbly);
    s5.write(angle5+30 - i1L1 * 2);
    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }

  while(status1 <= fbl){
    status1++;
    i1L1++;
    s3.write(angle3-30 + i1L1 * 5);
    s4.write(angle4-30 + i1L1 * 2);
    s5.write(angle5+10 - i1L1);
    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
  /****************step 5 to step 6***********************/
  while(status1 <= fbl){
    status1++;
    i1L1++;
    s2.write(angle2-30 + i1L1 * fbly);
    s3.write(angle3+20 - i1L1 * 2);
    s4.write(angle4-10 + i1L1);
    s6.write(angle6 + i1L1 * 2);
    s7.write(angle7 - i1L1);
    s11.write(angle11 - i1L1 * fbly);
    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }

  /****************step 4 to step 5***********************/
  while(status1 <= fbl){
    status1++;
    i1L1++;
    s6.write(angle6+20 - i1L1 * 5);
    s7.write(angle7-10 - i1L1 * 2);
    s8.write(angle8 + i1L1 * 2);
    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
  while(status1 <= fbl){
    status1++;
    i1L1++;
    s6.write(angle6-30 + i1L1 * fbly);
    s7.write(angle7-30 + i1L1 * fbly);
    s8.write(angle8+20 + i1L1);
    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
/****************step 3 to step 4***********************/
  while(status1 <= fbl){
    status1++;
    i1L1++;
    s9.write(angle9 + i1L1 * fbly);
    s10.write(angle10 + i1L1 * fbly);
    s11.write(angle11-30 + i1L1 * 2);
    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
  while(status1 <= fbl){
    status1++;
    i1L1++;
    s9.write(angle9+30 - i1L1 * 5);
    s10.write(angle10+30 - i1L1 * 2);
    s11.write(angle11-10 + i1L1);
    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
  /****************step 2 to step 3***********************/
  while(status1 <= fbl){
    status1++;
    i1L1++; 
    s0.write(angle0 - i1L1 * 2);
    s1.write(angle1 + i1L1);  
    s5.write(angle5 + i1L1 * fbly);
    s8.write(angle8+30 - i1L1 * fbly);
    s9.write(angle9-20 + i1L1 * 2);
    s10.write(angle10+10 - i1L1);
    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
    /******************step 1 to step 2************************/
  // Rotate the leg1
  while (status1 <= fbl) {
    status1++;
    i1L1++;
    s0.write(angle0-20 + i1L1 * 5);
    s1.write(angle1+10 + i1L1 * 2);
    s2.write(angle2 - i1L1 * 2);
    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
  while (status1 <= fbl) {
    status1++;
    i1L1++;
    s0.write(angle0+30 - i1L1 * fbly);
    s1.write(angle1+30 - i1L1 * fbly);
    s2.write(angle2-20 - i1L1);
    if(status1 > fbl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
}
void turnright(){
  if(battery_voltage < 860 ){                      //If batteryvoltage is below 10.5V and higher than 8.0V 860
    for(int i=0;i<6;i++){
      strip.setPixelColor(i,strip.Color(255,0,0));
      strip.show();
      delay(1);                                                //Turn on the led if battery voltage is to low                                            
    }                                               
  }
/********************step 1  first******************************/  
  while(status1 <= fblrl){
    status1++;
    i1L1++;
    s0.write(angle0 + i1L1 * 3 * fblyrl);
    s1.write(angle1 + i1L1 * 3 * fblyrl); 
    s2.write(angle2-30 + i1L1 * 2 * fblyrl);
    s3.write(angle3); 
    s4.write(angle4);
    s5.write(angle5+30 - i1L1 * fblyrl);
    s6.write(angle6);
    s7.write(angle7);
    s8.write(angle8+10 - i1L1 * fblyrl);
    s9.write(angle9);
    s10.write(angle10);
    s11.write(angle11-10 - i1L1 * fblyrl);
      if(status1 > fblrl){
        status1 = 0;
        i1L1 = 0;
        break;
      }
      delay(motorspeed);
  }
  while(status1 <= fblrl){
    status1++;
    i1L1++;
    s0.write(angle0+30 - i1L1 * 3 * fblyrl);
    s1.write(angle1+30 - i1L1 * 3 * fblyrl); 
    s2.write(angle2-10 + i1L1 * fblyrl);
    if(status1 > fblrl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
/********************step 2  third******************************/  
  while(status1 <= fblrl){
    status1++;
    i1L1++;
    s0.write(angle0);
    s1.write(angle1); 
    s2.write(angle2 - i1L1 * fblyrl);
    s3.write(angle3); 
    s4.write(angle4);
    s5.write(angle5+20 - i1L1 * fblyrl);
    s6.write(angle6 - i1L1 * 3 * fblyrl);
    s7.write(angle7 - i1L1 * 3 * fblyrl);
    s8.write(angle8 + i1L1 * 2 * fblyrl);
    s9.write(angle9);
    s10.write(angle10);
    s11.write(angle11-20 - i1L1 * fblyrl); 
    if(status1 > fblrl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }

  while(status1 <= fblrl){
    status1++;
    i1L1++;
    s6.write(angle6-30 + i1L1 * 3 * fblyrl);
    s7.write(angle7-30 + i1L1 * 3 * fblyrl);
    s8.write(angle8+20 + i1L1 * fblyrl);
      if(status1 > fblrl){
        status1 = 0;
        i1L1 = 0;
        break;
      }
      delay(motorspeed);
  }
/********************step 4  fourth******************************/  
  while(status1 <= fblrl){
    status1++;
    i1L1++;
    s0.write(angle0);
    s1.write(angle1); 
    s2.write(angle2-10 - i1L1 * fblyrl);
    s3.write(angle3); 
    s4.write(angle4);
    s5.write(angle5+10 - i1L1 * fblyrl);
    s6.write(angle6);
    s7.write(angle7);
    s8.write(angle8+30 - i1L1 * fblyrl);
    s9.write(angle9 + i1L1 * 3 * fblyrl);
    s10.write(angle10 + i1L1 * 3 * fblyrl);
    s11.write(angle11-30 + i1L1 * 2 * fblyrl); 
      if(status1 > fblrl){
        status1 = 0;
        i1L1 = 0;
        break;
      }
      delay(motorspeed);
  }
  while(status1 <= fblrl){
    status1++;
    i1L1++;
    s9.write(angle9+30 - i1L1 * 3 * fblyrl);
    s10.write(angle10+30 - i1L1 * 3 * fblyrl);
    s11.write(angle11-10 + i1L1 * fblyrl); 
      if(status1 > fblrl){
        status1 = 0;
        i1L1 = 0;
        break;
      }
      delay(motorspeed);
  }
/********************step 3  second******************************/  
  while(status1 <= fblrl){
    status1++;
    i1L1++;
    s0.write(angle0);
    s1.write(angle1); 
    s2.write(angle2-20 - i1L1 * fblyrl);
    s3.write(angle3 - i1L1 * 3 * fblyrl); 
    s4.write(angle4 - i1L1 * 3 * fblyrl);
    s5.write(angle5 + i1L1 * 2 * fblyrl);
    s6.write(angle6);
    s7.write(angle7);
    s8.write(angle8+20 - i1L1 * fblyrl);
    s9.write(angle9);
    s10.write(angle10);
    s11.write(angle11 - i1L1 * fblyrl);
        if(status1 > fblrl){
          status1 = 0;
          i1L1 = 0;
          break;
        }
        delay(motorspeed);
  }
  while(status1 <= fblrl){
    status1++;
    i1L1++;
    s3.write(angle3-30 + i1L1 * 3 * fblyrl); 
    s4.write(angle4-30 + i1L1 * 3 * fblyrl);
    s5.write(angle5+20 + i1L1 * fblyrl);
      if(status1 > fblrl){
        status1 = 0;
        i1L1 = 0;
        break;
      }
      delay(motorspeed);
  }
}
void turnleft(){
   if(battery_voltage > 860){
      for(int i=0;i<6;i++){
      strip.setPixelColor(i,strip.Color(55,122,132));
      strip.show(); 
      delay(1);                                              
     }
    }
/********************step 1  third******************************/  
  while(status1 <= fblrl){
    status1++;
    i1L1++;
    s0.write(angle0);
    s1.write(angle1); 
    s2.write(angle2-10 + i1L1 * fblyrl);
    s3.write(angle3); 
    s4.write(angle4);
    s5.write(angle5+10 + i1L1 * fblyrl);
    s6.write(angle6 - i1L1 * 3 * fblyrl);
    s7.write(angle7 - i1L1 * 3 * fblyrl);
    s8.write(angle8+30 - i1L1 * 2 * fblyrl);
    s9.write(angle9);
    s10.write(angle10);
    s11.write(angle11-30 + i1L1 * fblyrl)  ;
    if(status1 > fblrl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
  while(status1 <= fblrl){
    status1++;
    i1L1++;
    s6.write(angle6-30 + i1L1 * 3 * fblyrl);
    s7.write(angle7-30 + i1L1 * 3 * fblyrl);
    s8.write(angle8+10 - i1L1 * fblyrl);
      if(status1 > fblrl){
        status1 = 0;
        i1L1 = 0;
        break;
      }
      delay(motorspeed);
  }
/********************step 2  first******************************/  
  while(status1 <= fblrl){
    status1++;
    i1L1++;
    s0.write(angle0 + i1L1 * 3 * fblyrl);
    s1.write(angle1 + i1L1 * 3 * fblyrl); 
    s2.write(angle2 - i1L1 * 2 * fblyrl);
    s3.write(angle3); 
    s4.write(angle4);
    s5.write(angle5+20 + i1L1 * fblyrl);
    s6.write(angle6);
    s7.write(angle7);
    s8.write(angle8 + i1L1 * fblyrl);
    s9.write(angle9);
    s10.write(angle10);
    s11.write(angle11-20 + i1L1 * fblyrl);  
    if(status1 > fblrl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
  while(status1 <= fblrl){
    status1++;
    i1L1++;
    s0.write(angle0+30 - i1L1 * 3 * fblyrl);
    s1.write(angle1+30 - i1L1 * 3 * fblyrl); 
    s2.write(angle2-20 - i1L1 * fblyrl);
    if(status1 > fblrl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
/********************step 3  second******************************/  
  while(status1 <= fblrl){
    status1++;
    i1L1++;
    s0.write(angle0);
    s1.write(angle1); 
    s2.write(angle2-30 + i1L1 * fblyrl);
    s3.write(angle3 - i1L1 * 3 * fblyrl); 
    s4.write(angle4 - i1L1 * 3 * fblyrl);
    s5.write(angle5+30 - i1L1 * 2 * fblyrl);
    s6.write(angle6);
    s7.write(angle7);
    s8.write(angle8+10 + i1L1 * fblyrl);
    s9.write(angle9);
    s10.write(angle10);
    s11.write(angle11-10 + i1L1 * fblyrl); 
    if(status1 > fblrl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
  while(status1 <= fblrl){
    status1++;
    i1L1++;
    s3.write(angle3-30 + i1L1 * 3 * fblyrl); 
    s4.write(angle4-30 + i1L1 * 3 * fblyrl);
    s5.write(angle5+10 - i1L1 * fblyrl);
    if(status1 > fblrl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
/********************step 4  fourth******************************/  
  while(status1 <= fblrl){
    status1++;
    i1L1++;
    s0.write(angle0);
    s1.write(angle1); 
    s2.write(angle2-20 + i1L1 * fblyrl);
    s3.write(angle3); 
    s4.write(angle4);
    s5.write(angle5 + i1L1 * fblyrl);
    s6.write(angle6);
    s7.write(angle7);
    s8.write(angle8+20 + i1L1 * fblyrl);
    s9.write(angle9 + i1L1 * 3 * fblyrl);
    s10.write(angle10 + i1L1 * 3 * fblyrl);
    s11.write(angle11 - i1L1 * 2 * fblyrl); 
    if(status1 > fblrl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
  while(status1 <= fblrl){
    status1++;
    i1L1++;
    s9.write(angle9+30 - i1L1 * 3 * fblyrl);
    s10.write(angle10+30 - i1L1 * 3 * fblyrl);
    s11.write(angle11-20 - i1L1 * fblyrl); 
    if(status1 > fblrl){
      status1 = 0;
      i1L1 = 0;
      break;
    }
    delay(motorspeed);
  }
}
void advoid(){
  moveforward();
  advoid_judge();
}
void advoid_judge(){
  ultrasonic();
  if(a<=20){
  for(int i = 0;i<=6;i++){
    turnright();
  }
  }
}
void battery(){
  battery_voltage = battery_voltage*0.92 + ((analogRead(A7)*2.93)+250)*0.08; 
  if(battery_voltage < 860 ){                      //If batteryvoltage is below 10.5V and higher than 8.0V 860
    for(int i=0;i<6;i++){
      strip.setPixelColor(i,strip.Color(255,0,0));
      strip.show();
      delay(1);                                    //Turn on the led if battery voltage is to low                                                    
    }                                              //Turn on the led if battery voltage is to low
  }
  else{
      for(int i=0;i<6;i++){
        strip.setPixelColor(i,strip.Color(55,122,132));
        strip.show(); 
        delay(1);                                                 
      }
    }
}
void self_balanced_test(){
  s2.write(angle2);
  s5.write(angle5);
  s8.write(angle8);
  s11.write(angle11);
  if(angle_roll_output<-1 && angle_pitch_output<3 && angle_pitch_output>-3){
    s0.write(angle0 - angle_roll_output*anglerange*10/12);
    s1.write(angle1 - angle_roll_output*anglerange*10/12);
    s6.write(angle6 + angle_roll_output*anglerange*10/12);
    s7.write(angle7 + angle_roll_output*anglerange*10/12);
    s3.write(angle3 + angle_roll_output*anglerange*2/12);
    s4.write(angle4 + angle_roll_output*anglerange*2/12);
    s9.write(angle9 - angle_roll_output*anglerange*2/12);
    s10.write(angle10 - angle_roll_output*anglerange*2/12);
  }else if(angle_roll_output>3 && angle_pitch_output<3 && angle_pitch_output>-3){
      s0.write(angle0 + angle_roll_output*anglerange*2/12);
      s1.write(angle1 + angle_roll_output*anglerange*2/12);
      s6.write(angle6 - angle_roll_output*anglerange*2/12);
      s7.write(angle7 - angle_roll_output*anglerange*2/12);
      s3.write(angle3 - angle_roll_output*anglerange*10/12);
      s4.write(angle4 - angle_roll_output*anglerange*10/12);
      s9.write(angle9 + angle_roll_output*anglerange*10/12);
      s10.write(angle10 + angle_roll_output*anglerange*10/12);
  }else if(angle_pitch_output>3 && angle_roll_output<3 && angle_roll_output>-3){
      s0.write(angle0 + angle_pitch_output*anglerange*10/12);
      s1.write(angle1 + angle_pitch_output*anglerange*10/12);
      s3.write(angle3 - angle_pitch_output*anglerange*10/12);
      s4.write(angle4 - angle_pitch_output*anglerange*10/12);
      s6.write(angle6 - angle_pitch_output*anglerange*2/12);
      s7.write(angle7 - angle_pitch_output*anglerange*2/12);
      s9.write(angle9 + angle_pitch_output*anglerange*2/12);
      s10.write(angle10 + angle_pitch_output*anglerange*2/12);
  }else if(angle_pitch_output<-3 && angle_roll_output<3 && angle_roll_output>-3){
      s0.write(angle0 - angle_pitch_output*anglerange*2/12);
      s1.write(angle1 - angle_pitch_output*anglerange*2/12);
      s3.write(angle3 + angle_pitch_output*anglerange*2/12);
      s4.write(angle4 + angle_pitch_output*anglerange*2/12);
      s6.write(angle6 + angle_pitch_output*anglerange*10/12);
      s7.write(angle7 + angle_pitch_output*anglerange*10/12);
      s9.write(angle9 - angle_pitch_output*anglerange*10/12);
      s10.write(angle10 - angle_pitch_output*anglerange*10/12);
  }
}
void self_balanced_setup(){
  Wire.begin();                                                        //Start I2C as master
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  //Run this code 2000 times                            
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 1000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 1000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 1000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
  loop_timer = micros();                                               //Reset the loop timer  
}
void self_balanced(){
  read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050
  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
  //Gyro angle calculations
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll
  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();                                               //Reset the loop timer
}
void read_mpu_6050_data(){                                             
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
//  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  delay(1);
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable
}
void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}
void ultrasonic(){
  a=sr04.Distance();
}
void IRremote(){
  if (irrecv.decode(&results))
    {  
      if(results.value == 16718055){
        infrared = 1;
        irrecv.resume(); 
      }
       else if(results.value == 16730805){
        infrared = 2;
        irrecv.resume(); 
       }
       else if(results.value == 16734885){
        infrared = 3;
        irrecv.resume(); 
       }
       else if(results.value == 16716015){
        infrared = 4;
        irrecv.resume(); 
       }

       else if(results.value == 16726215){
        infrared = 5;
        irrecv.resume(); 
       }

       else if(results.value == 16724175){
        infrared = 6;
        irrecv.resume();
       }
       if(infrared ==1){
          if(results.value == 4294967295){
            moveforward();
            irrecv.resume(); 
          }
        }
        else if(infrared ==2){
          if(results.value == 4294967295){
            movebackward();
            irrecv.resume(); 
          }
        }
        else if(infrared == 3){
          if(results.value == 4294967295){
            turnright();
            irrecv.resume(); 
          }
        }
        else if(infrared ==4){
          if(results.value == 4294967295){
            turnleft();
            irrecv.resume(); 
          }
        }
        else if(infrared == 6){
          if(results.value == 4294967295){
          advoid();
          irrecv.resume(); 
          }
        }
        irrecv.resume(); 
    } 
    else if(infrared == 5){
      self_balanced();
      self_balanced_test();
    }
}
void infrared_judge(){
  if(results.value == 16718055){
    infrared = 1;
    irrecv.resume();
  }
   else if(results.value == 16730805){
    infrared = 2; 
    irrecv.resume();
   }
   else if(results.value == 16734885){
    infrared = 3;
    irrecv.resume(); 
   }
   else if(results.value == 16716015){
    infrared = 4;
    irrecv.resume(); 
   }
   else if(results.value == 16726215){
    infrared = 5;
    irrecv.resume();
   }
   else if(results.value == 4294967295){
    infrared = 6;
    irrecv.resume();
   }
}
