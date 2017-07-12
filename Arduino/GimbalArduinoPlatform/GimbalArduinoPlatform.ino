////////////////////////////////////////////////////////////////////////////
//
//  This file is part of Head Tracker
//
//  Copyright (c) 2017 Smart System Institute, National University of Singapore, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <EEPROM.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9150Lib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <EEPROM.h>

//  DEVICE_TO_USE selects whether the IMU at address 0x68 (default) or 0x69 is used
//    0 = use the device at 0x68
//    1 = use the device at ox69
#define  DEVICE_TO_USE    0
MPU9150Lib MPU;
//  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output
#define MPU_UPDATE_RATE  (20)
//  MPU_LPF_RATE is the low pas filter rate and can be between 5 and 188Hz
#define MPU_LPF_RATE   40
//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

// Serial Communication
#define SERIAL_BAUD 9600
char serial_data[100];
unsigned char serial_index = 0;


// Parameter
int pulse_Pan_Min,    pulse_Tilt_Min,    pulse_Roll_Min;
int pulse_Pan_CENTER, pulse_Tilt_CENTER, pulse_Roll_CENTER;
int pulse_Pan_Max,    pulse_Tilt_Max,    pulse_Roll_Max;
int pulse_Pan_PPM_Min,    pulse_Tilt_PPM_Min,    pulse_Roll_PPM_Min;
int pulse_Pan_PPM_Max,    pulse_Tilt_PPM_Max,    pulse_Roll_PPM_Max;
int PanCh, TiltCh, RollCh;
bool pan_reverse, tilt_reverse, roll_reverse;
bool isPPM;

// Configuration for PPM Output
#define CHANNEL_NUMBER 12  //set the number of chanels
#define CHANNEL_DEFAULT_VALUE 1500  //set the default servo value
#define FRAME_LENGTH 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 9  //set PPM signal output pin on the arduino
int ppm[CHANNEL_NUMBER];
int t_ppm[CHANNEL_NUMBER];

#define btnPin 8

void setup() {
  Wire.begin();
  MPU.selectDevice(DEVICE_TO_USE);           // only really necessary if using device 1
  MPU.init(MPU_UPDATE_RATE, MPU_LPF_RATE);   // start the MPU 
  Serial.begin(SERIAL_BAUD);
    
  //initiallize default ppm values
  for(int i=0; i<CHANNEL_NUMBER; i++){
      ppm[i]= CHANNEL_DEFAULT_VALUE;
  }
  setInitialeData();
  readParameterFromEEPROM();
  
  pinMode(btnPin, INPUT_PULLUP);
  pinMode(sigPin, OUTPUT);
  
  if(readEEPROM_I(44)!=1){
    setInitialeData();    
    saveParameterToEEPROM();    
    writeEEPROM_I(44, 1);    
  }

  
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  
  if(digitalRead(btnPin)==0){
    isPPM = !isPPM; 
  }
  if(isPPM){
    cli();
    TCCR1A = 0; // set entire TCCR1 register to 0
    TCCR1B = 0;
    OCR1A = 100;  // compare match register, change this
    TCCR1B |= (1 << WGM12);  // turn on CTC mode
    TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    sei();
  }
}



void loop() {
  // MPU data update
  MPU.selectDevice(DEVICE_TO_USE);
  if (MPU.read()) 
  {
    pan  = MPU.m_dmpEulerPose[VEC3_Z] * RAD_TO_DEGREE;
    tilt = MPU.m_dmpEulerPose[VEC3_Y] * RAD_TO_DEGREE;
    roll = MPU.m_dmpEulerPose[VEC3_X] * RAD_TO_DEGREE;
  }
  
  if(digitalRead(btnPin)==0){
    th_pan  = pan;
    th_tilt = tilt;
    th_roll = roll;
  } 
  for(int i=0; i<CHANNEL_NUMBER; i++){
    t_ppm[i] = CHANNEL_DEFAULT_VALUE;
  }
  if(pan_reverse ) f_pan  = (pan - th_pan ) * -1.0; else f_pan  = (pan - th_pan );
  if(tilt_reverse) f_tilt = (tilt- th_tilt) * -1.0; else f_tilt = (tilt- th_tilt);
  if(roll_reverse) f_roll = (roll- th_roll) * -1.0; else f_roll = (roll- th_roll);
  
  if(f_pan  < pulse_Pan_Min)  f_pan  = pulse_Pan_Min;  else if( pulse_Pan_Max  < f_pan ) f_pan  = pulse_Pan_Max;
  if(f_tilt < pulse_Tilt_Min) f_tilt = pulse_Tilt_Min; else if( pulse_Tilt_Max < f_tilt) f_tilt = pulse_Tilt_Max;
  if(f_roll < pulse_Roll_Min) f_roll = pulse_Roll_Min; else if( pulse_Roll_Max < f_roll) f_roll = pulse_Roll_Max;

  if(isPPM){
    float tmp_pan  = map(f_pan , pulse_Pan_Min, pulse_Pan_Max , pulse_Pan_PPM_Min,  pulse_Pan_PPM_Max);
    float tmp_tilt = map(f_tilt, pulse_Tilt_Min,pulse_Tilt_Max, pulse_Tilt_PPM_Min, pulse_Tilt_PPM_Max);
    float tmp_roll = map(f_roll, pulse_Roll_Min,pulse_Roll_Max, pulse_Roll_PPM_Min, pulse_Roll_PPM_Max);
    tmp_pan  = tmp_pan  - (1500 - pulse_Pan_CENTER);
    if(tmp_pan < pulse_Pan_PPM_Min) tmp_pan = pulse_Pan_PPM_Min; else if(pulse_Pan_PPM_Max < tmp_pan) tmp_pan = pulse_Pan_PPM_Max;
    tmp_tilt = tmp_tilt - (1500 - pulse_Tilt_CENTER);
    if(tmp_tilt < pulse_Tilt_PPM_Min) tmp_tilt = pulse_Tilt_PPM_Min; else if(pulse_Tilt_PPM_Max < tmp_tilt) tmp_tilt = pulse_Tilt_PPM_Max;
    tmp_roll = tmp_roll - (1500 - pulse_Roll_CENTER);
    if(tmp_roll < pulse_Roll_PPM_Min) tmp_roll = pulse_Roll_PPM_Min; else if(pulse_Roll_PPM_Max < tmp_roll) tmp_roll = pulse_Roll_PPM_Max;
    t_ppm[PanCh] = tmp_pan;
    t_ppm[TiltCh] = tmp_tilt;
    t_ppm[RollCh] = tmp_roll;
  }else{
    t_ppm[PanCh]  = map(f_pan/10 , pulse_Pan_Min/10, pulse_Pan_Max/10,0, 255);
    t_ppm[TiltCh] = map(f_tilt/10, pulse_Pan_Min/10, pulse_Pan_Max/10,0, 255);
    t_ppm[RollCh] = map(f_roll/10, pulse_Pan_Min/10, pulse_Pan_Max/10,0, 255);
  }
  for(int i=0; i<CHANNEL_NUMBER; i++){
      ppm[i]= t_ppm[i];
  }
  
  if(!isPPM){
      analogWrite(9, t_ppm[PanCh]);
      analogWrite(10,t_ppm[TiltCh]);
      analogWrite(11,t_ppm[RollCh]);
  }
        
  //For Communication against PC via Serial
  if(Serial.available()){
    serial_data[serial_index++]=Serial.read();

    // write data to eeprom
    if(serial_data[serial_index-2]=='R' &&serial_data[serial_index-1]=='A'){
      Serial.print("RA"); Serial.print("\t");
      Serial.print(f_pan); Serial.print("\t");
      Serial.print(f_tilt); Serial.print("\t");
      Serial.print(f_roll); Serial.println("\t");
      serial_index=0;
    }
    else if(serial_data[serial_index-2]=='R' &&serial_data[serial_index-1]=='P'){
      Serial.print("RP"); Serial.print("\t");
      Serial.print(pulse_Pan_Min); Serial.print("\t");
      Serial.print(pulse_Pan_CENTER); Serial.print("\t");
      Serial.print(pulse_Pan_Max); Serial.print("\t");
      Serial.print(pulse_Pan_PPM_Min); Serial.print("\t");
      Serial.print(pulse_Pan_PPM_Max); Serial.println("\t");
      serial_index=0;
    }
    else if(serial_data[serial_index-2]=='R' &&serial_data[serial_index-1]=='T'){
      Serial.print("RT"); Serial.print("\t");
      Serial.print(pulse_Tilt_Min); Serial.print("\t");
      Serial.print(pulse_Tilt_CENTER); Serial.print("\t");
      Serial.print(pulse_Tilt_Max); Serial.print("\t");
      Serial.print(pulse_Tilt_PPM_Min); Serial.print("\t");
      Serial.print(pulse_Tilt_PPM_Max); Serial.println("\t");
      serial_index=0;
    }
    else if(serial_data[serial_index-2]=='R' &&serial_data[serial_index-1]=='U'){
      Serial.print("RU"); Serial.print("\t");
      Serial.print(pulse_Roll_Min); Serial.print("\t");
      Serial.print(pulse_Roll_CENTER); Serial.print("\t");
      Serial.print(pulse_Roll_Max); Serial.print("\t");
      Serial.print(pulse_Roll_PPM_Min); Serial.print("\t");
      Serial.print(pulse_Roll_PPM_Max); Serial.println("\t");
      serial_index=0;
    }
    else if(serial_data[serial_index-2]=='R' &&serial_data[serial_index-1]=='C'){
      Serial.print("RC"); Serial.print("\t");
      if(pan_reverse ) Serial.print(1);
      else Serial.print(0);
      Serial.print("\t");
      if(tilt_reverse) Serial.print(1);
      else Serial.print(0);
      Serial.print("\t");
      if(roll_reverse) Serial.print(1);
      else Serial.print(0);
      Serial.print("\t");
      Serial.print(PanCh); Serial.print("\t");
      Serial.print(TiltCh); Serial.print("\t");
      Serial.print(RollCh); Serial.print("\t");
      if(isPPM) Serial.print(1);
      else Serial.print(0);
      Serial.println("\t");
      serial_index=0;
    }

    if(serial_data[serial_index-2]=='W' &&serial_data[serial_index-1]=='P'){
      int values[5] = {0,0,0,0,0};
      int comma_index = 0;
      bool nFlag = false;
      for (unsigned char k = 0; k < serial_index-3; k++) {
        // Looking for comma
        if(serial_data[k] == 44) {
          if(nFlag) values[comma_index] = values[comma_index] * -1;
          nFlag = false;
          comma_index++;
        }else if(serial_data[k] == 45){
          nFlag = true;
        }else{
          values[comma_index] = values[comma_index]*10+(serial_data[k]-48);
        }
      } 
      pulse_Pan_Min        = values[0];
      pulse_Pan_CENTER       = values[1];
      pulse_Pan_Max        = values[2];
      pulse_Pan_PPM_Min    = values[3];
      pulse_Pan_PPM_Max    = values[4];
      Serial.print("WP");Serial.println("\t");
      serial_index=0;
    }
    else if(serial_data[serial_index-2]=='W' &&serial_data[serial_index-1]=='T'){
      int values[5] = {0,0,0,0,0};
      int comma_index = 0;
      bool nFlag = false;
      for (unsigned char k = 0; k < serial_index-3; k++) {
        // Looking for comma
        if(serial_data[k] == 44) {
          if(nFlag) values[comma_index] = values[comma_index] * -1;
          nFlag = false;
          comma_index++;
        }else if(serial_data[k] == 45){
          nFlag = true;
        }else{
          values[comma_index] = values[comma_index]*10+(serial_data[k]-48);
        }
      } 
      pulse_Tilt_Min        = values[0];
      pulse_Tilt_CENTER       = values[1];
      pulse_Tilt_Max        = values[2];
      pulse_Tilt_PPM_Min    = values[3];
      pulse_Tilt_PPM_Max    = values[4];
      Serial.print("WT");Serial.println("\t");
      serial_index=0;
    }   
    else if(serial_data[serial_index-2]=='W' &&serial_data[serial_index-1]=='U'){
      int values[5] = {0,0,0,0,0};
      int comma_index = 0;
      bool nFlag = false;
      for (unsigned char k = 0; k < serial_index-3; k++) {
        // Looking for comma
        if(serial_data[k] == 44) {
          if(nFlag) values[comma_index] = values[comma_index] * -1;
          nFlag = false;
          comma_index++;
        }else if(serial_data[k] == 45){
          nFlag = true;
        }else{
          values[comma_index] = values[comma_index]*10+(serial_data[k]-48);
        }
      } 
      pulse_Roll_Min        = values[0];
      pulse_Roll_CENTER       = values[1];
      pulse_Roll_Max        = values[2];
      pulse_Roll_PPM_Min    = values[3];
      pulse_Roll_PPM_Max    = values[4];
      Serial.print("WU");Serial.println("\t");
      serial_index=0;
    }   
    else if(serial_data[serial_index-2]=='W' &&serial_data[serial_index-1]=='C'){
      int values[7] = {0,0,0,0,0,0,0};
      int comma_index = 0;
      for (unsigned char k = 0; k < serial_index-3; k++) {
        // Looking for comma
        if(serial_data[k] == 44) {
          comma_index++;
        }else{
          values[comma_index] = values[comma_index]*10+(serial_data[k]-48);
        }
      }
      if(values[0]==0)pan_reverse  = false; else pan_reverse  = true;
      if(values[1]==0)tilt_reverse = false; else tilt_reverse = true;
      if(values[2]==0)roll_reverse = false; else roll_reverse = true;
      PanCh        = values[3];
      TiltCh       = values[4];
      RollCh       = values[5];
      if(values[6]==0)isPPM = false; else isPPM = true;
      Serial.print("WC");Serial.println("\t");
      serial_index=0;
    }   
    else if(serial_data[serial_index-2]=='S' &&serial_data[serial_index-1]=='A'){
      saveParameterToEEPROM();
      Serial.print("SA");Serial.println("\t");
      serial_index=0;
    }

  }
}
void showAll(){
      Serial.print("RF"); Serial.print("\t");
      Serial.print(pulse_Pan_Min); Serial.print("\t");
      Serial.print(pulse_Pan_CENTER); Serial.print("\t");
      Serial.print(pulse_Pan_Max); Serial.print("\t");
      Serial.print(pulse_Tilt_Min); Serial.print("\t");
      Serial.print(pulse_Tilt_CENTER); Serial.print("\t");
      Serial.print(pulse_Tilt_Max); Serial.print("\t");
      Serial.print(pulse_Roll_Min); Serial.print("\t");
      Serial.print(pulse_Roll_CENTER); Serial.print("\t");
      Serial.print(pulse_Roll_Max); Serial.print("\t");
      if(pan_reverse ) Serial.print(1);
      else Serial.print(0);
      Serial.print("\t");
      if(tilt_reverse) Serial.print(1);
      else Serial.print(0);
      Serial.print("\t");
      if(roll_reverse) Serial.print(1);
      else Serial.print(0);
      Serial.print("\t");
      Serial.print(PanCh); Serial.print("\t");
      Serial.print(TiltCh); Serial.print("\t");
      Serial.print(RollCh); Serial.print("\t");
      if(isPPM) Serial.print(1);
      else Serial.print(0);
      Serial.println("\t");
}
void readParameterFromEEPROM(){
  pulse_Pan_Min     = readEEPROM_I(0);
  pulse_Pan_CENTER  = readEEPROM_I(2);
  pulse_Pan_Max     = readEEPROM_I(4);
  pulse_Pan_PPM_Min = readEEPROM_I(6);
  pulse_Pan_PPM_Max = readEEPROM_I(8);
      
  pulse_Tilt_Min     = readEEPROM_I(10);
  pulse_Tilt_CENTER  = readEEPROM_I(12);
  pulse_Tilt_Max     = readEEPROM_I(14);
  pulse_Tilt_PPM_Min = readEEPROM_I(16);
  pulse_Tilt_PPM_Max = readEEPROM_I(18);
      
  pulse_Roll_Min     = readEEPROM_I(20);
  pulse_Roll_CENTER  = readEEPROM_I(22);
  pulse_Roll_Max     = readEEPROM_I(24);
  pulse_Roll_PPM_Min = readEEPROM_I(26);
  pulse_Roll_PPM_Max = readEEPROM_I(28);

  PanCh  = readEEPROM_I(30);
  TiltCh = readEEPROM_I(32);
  RollCh = readEEPROM_I(34);

  if(readEEPROM_I(36)==1)pan_reverse   = true; else pan_reverse   = false;
  if(readEEPROM_I(38)==1)tilt_reverse  = true; else tilt_reverse  = false;
  if(readEEPROM_I(40)==1)roll_reverse  = true; else roll_reverse  = false;
      
  if(readEEPROM_I(42)==1)isPPM  = true; else isPPM  = false;
}

void saveParameterToEEPROM(){
  writeEEPROM_I(0,   pulse_Pan_Min    );
  writeEEPROM_I(2,   pulse_Pan_CENTER );
  writeEEPROM_I(4,   pulse_Pan_Max    );
  writeEEPROM_I(6,   pulse_Pan_PPM_Min);
  writeEEPROM_I(8,   pulse_Pan_PPM_Max);  
  
  writeEEPROM_I(10,  pulse_Tilt_Min    );
  writeEEPROM_I(12,  pulse_Tilt_CENTER );
  writeEEPROM_I(14,  pulse_Tilt_Max    );
  writeEEPROM_I(16,  pulse_Tilt_PPM_Min);
  writeEEPROM_I(18,  pulse_Tilt_PPM_Max);

  writeEEPROM_I(20,  pulse_Roll_Min    );
  writeEEPROM_I(22,  pulse_Roll_CENTER );
  writeEEPROM_I(24,  pulse_Roll_Max    );
  writeEEPROM_I(26,  pulse_Roll_PPM_Min);
  writeEEPROM_I(28,  pulse_Roll_PPM_Max);

  writeEEPROM_I(30, PanCh            );
  writeEEPROM_I(32, TiltCh           );
  writeEEPROM_I(34, RollCh           );
  int i_True = 1;
  int i_False= 0;
  if(pan_reverse )writeEEPROM_I(36, i_True);
  else            writeEEPROM_I(36, i_False);
  if(tilt_reverse)writeEEPROM_I(38, i_True);
  else            writeEEPROM_I(38, i_False);
  if(roll_reverse)writeEEPROM_I(40, i_True);
  else            writeEEPROM_I(40, i_False);
  if(isPPM) writeEEPROM_I(42, i_True      );
  else      writeEEPROM_I(42, i_False      );
}

void setInitialeData(){
  pan    = 0.0; tilt    = 0.0; roll    = 0.0;
  th_pan = 0.0; th_tilt = 0.0; th_roll = 0.0;
 
  pulse_Pan_Min  = -90;
  pulse_Tilt_Min = -90;
  pulse_Roll_Min = -90;
  
  pulse_Pan_CENTER  = CHANNEL_DEFAULT_VALUE;
  pulse_Tilt_CENTER = CHANNEL_DEFAULT_VALUE;
  pulse_Roll_CENTER = CHANNEL_DEFAULT_VALUE;

  pulse_Pan_Max  = 90;
  pulse_Tilt_Max = 90;
  pulse_Roll_Max = 90;

  pulse_Pan_PPM_Min  = 1000;
  pulse_Tilt_PPM_Min = 1000;
  pulse_Roll_PPM_Min = 1000;
  
  pulse_Pan_PPM_Max  = 2000;
  pulse_Tilt_PPM_Max = 2000;
  pulse_Roll_PPM_Max = 2000;
  
  
  PanCh = 0; TiltCh = 1; RollCh = 2;
  pan_reverse = false; tilt_reverse = false; roll_reverse = false;
  isPPM = true;
}

ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  TCNT1 = 0;
  if (state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PULSE_LENGTH * 2;
    state = false;
  } else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
    digitalWrite(sigPin, !onState);
    state = true;
    if(cur_chan_numb >= CHANNEL_NUMBER){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PULSE_LENGTH;// 
      OCR1A = (FRAME_LENGTH - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PULSE_LENGTH) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}


int readEEPROM_I(int address){
  union {
    byte b[2];
    int i;
  }tmp;
  int ref_tmp = 0;
  while(ref_tmp != tmp.i){
    for(int i=0;i<2;i++) tmp.b[i]= EEPROM.read(address+i);
    ref_tmp = tmp.i;
  }
  return tmp.i;
}

void writeEEPROM_I(int address, int a){
  byte* tmp_i;
  tmp_i = (byte *)&a;
  while(1){
    for(int i=0;i<2;i++){
      EEPROM.write(address+i, tmp_i[i]);
    }
    int b = readEEPROM_I(address);
    if(a==b)break;
  }  
}

