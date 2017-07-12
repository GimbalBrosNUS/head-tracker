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

import processing.serial.*;
import controlP5.*;
import java.util.*;

// Serial Controller
Serial serialPort;
boolean bDeviceConn;
String stDeviceList[];
int iDeviceSelectedID;
int iExecCMDID; // identifying event on GUI

//
// SensorValue Class
// This class is for storing sensor data from arduino to visualize into graph.
// 
// sensorValue:      initialize this class, and set number of storing data.
// float getValue:   return value of pointed place in stored data.
// void addValue:    insert data, and oldest value is deleted.
// void resetValues: clear all stored data.
// int getStoredNumber:    return number of strage data .

class SensorValue{
  int number;
  float values[];
  SensorValue(int maxNumber){
    number = 0;
    values = new float[maxNumber];
  } 
  float getValue(int i){
    return values[i];
  }
  void addValue(float val){
    if(number < values.length){
      values[number++]=val;
    }else{
      for(int i=0;i<values.length-1;i++){
        values[i]=values[i+1];
      }
      values[values.length-1]=val;
    }
  }
  void resetValues(){
    number = 0;
    for(int i=0;i<values.length;i++){
      values[i]=0.0;
    }
  }
  int getStoredNumber(){
    return number;
  }
}
SensorValue sv[];

//
// SensorSetting class
// hangar space for sensor parameter
// float center
// float min
// float max
// float slide_pt
// int ppm_min
// int ppm_max
// boolean reverse
// int channel
class SensorSetting{
  float center;
  float min, max;
  float slider_pt;//pointer;
  int ppm_min;
  int ppm_max;
  boolean reverse;
  int channel;
  
  SensorSetting(){
    center = 1500;
    min = -90; max = 90;
    slider_pt = center;
    ppm_min = 1000;
    ppm_max = 2000;    
    reverse = false;
  }
};
SensorSetting ssPan, ssTilt, ssRoll;

//
// Window
// 
ControlP5 cp;

// GUI tool for management of device
DropdownList DlSerialDevList;
Button BtnConnect, BtnCalibrate, BtnSave;

// Pan GUI
Textfield TxPanMin, TxPanCenter, TxPanMax;
Textfield TxPanPPMMin, TxPanPPMMax;
Slider    SlPanCenterTuning;
Button    BtnPanReverse;
ScrollableList DlPanCh;

// Tilt GUI
Textfield TxTiltMin, TxTiltCenter, TxTiltMax;
Textfield TxTiltPPMMin, TxTiltPPMMax;
Slider    SlTiltCenterTuning;;
Button    BtnTiltReverse;
ScrollableList DlTiltCh;

// Roll GUI
Textfield TxRollMin, TxRollCenter, TxRollMax;
Textfield TxRollPPMMin, TxRollPPMMax;
Slider    SlRollCenterTuning;
Button    BtnRollReverse;
ScrollableList DlRollCh;

// Other Option
Button BtnPPM, BtnPWM;
boolean bOutputTypePPM = true;
Button BtnSetDefaultValues;

// Viewer GUI
Button BtnGFClear;
Textarea consoleWindow;
Println console;
float fGLMinVal, fGLMaxVal;



void setup(){
  iExecCMDID = 0;

  // Window setup
  surface.setTitle("Terminal 0.1 [unconnected]"); // set title bar name.
  size(800,500);
  
  // GUI setup  
  cp = new ControlP5(this);
  cp.enableShortcuts();
  setupGUI();
  console = cp.addConsole(consoleWindow);
  console.setMax(20);
  
  List l = Arrays.asList("1 ch","2 ch","3 ch","4 ch","5 ch","6 ch","7 ch","8 ch","9 ch","10 ch","11 ch","12 ch");
  DlPanCh.addItems(l);
  DlTiltCh.addItems(l);
  DlRollCh.addItems(l);
  
  // listup Connected Devices
  bDeviceConn = false;
  stDeviceList = Serial.list();
  for(int i=0;i<stDeviceList.length;i++){
    DlSerialDevList.addItem(stDeviceList[i], i);
  }

  // setup hanger space of sensor information
  sv     = new SensorValue[3];
  for(int i=0;i<sv.length;i++){
    sv[i] = new SensorValue(500);
  }
  ssPan  = new SensorSetting();
  ssTilt = new SensorSetting();
  ssRoll = new SensorSetting();  
  
  InitializeParameterValues();
  ResetGraphScale();

}
void ResetGraphScale(){
  fGLMinVal =  360;
  fGLMaxVal = -360;
  if(fGLMinVal > ssPan.min)  fGLMinVal = ssPan.min;
  if(fGLMinVal > ssTilt.min) fGLMinVal = ssTilt.min;
  if(fGLMinVal > ssRoll.min) fGLMinVal = ssRoll.min;
  
  if(fGLMaxVal < ssPan.max)  fGLMaxVal = ssPan.max;
  if(fGLMaxVal < ssTilt.max) fGLMaxVal = ssTilt.max;
  if(fGLMaxVal < ssRoll.max) fGLMaxVal = ssRoll.max;
}
void draw(){
  background(128);
  if(iExecCMDID==0 && bDeviceConn){
    serialPort.write(",RA"); 
    delay(20);
  }
  drawSensorGraph(20,50, 500,200);
  
  fill(255,0,0);
  text("Pan(Yaw): Z",30,310);
  text("Pan",600,252);
  fill(0,255,0);
  text("Tilt (Pitch): Y",280,310);
  text("Tilt",650,252);
  fill(0,0,255);
  text("Roll (Roll): X",530,310);
  text("Roll",700,252);
}



void drawSensorGraph(int x, int y, int w, int h){
  fill(0x39,0x39,0x39);
  strokeWeight(0.5);
  stroke(0);
  rect(x,y,w+50+10,h+10+20);

  noFill();
  strokeWeight(0.5);
  stroke(0x9C,0x9C,0x9C);
  line(x+50, y+10, x+50, y+10+h);
  line(x+50, y+10+h, x+50+w, y+10+h);  
  stroke(0x90,0x90,0x90);
  line(x+50, y+10, x+50+w, y+10);
  line(x+50, y+10+h/2, x+50+w, y+10+h/2);
  line(x+50, y+10+h/4, x+50+w, y+10+h/4);
  line(x+50, y+10+h/4*3, x+50+w, y+10+h/4*3);
  
  fill(255);
  text(fGLMinVal               ,20,y+10+h);
  text(fGLMinVal/2             ,20,y+10+h/4*3);
  text((fGLMinVal+fGLMaxVal)/2 ,20,y+10+h/4*2);
  text(fGLMaxVal/2             ,20,y+10+h/4);
  text(fGLMaxVal               ,20,y+10);

  
  noFill();
  strokeWeight(1);

  for(int i=0;i<sv[0].getStoredNumber()-1;i++){
    stroke(255,0,0);
    line(x+50+i,-1.0*map(sv[0].getValue(i),fGLMinVal, fGLMaxVal, -h/2, h/2)+h/2+y+10,x+50+(i+1),-1.0*map(sv[0].getValue(i+1),fGLMinVal, fGLMaxVal, -h/2, h/2)+h/2+y+10);
    stroke(0,255,0);
    line(x+50+i,-1.0*map(sv[1].getValue(i),fGLMinVal, fGLMaxVal, -h/2, h/2)+h/2+y+10,x+50+(i+1),-1.0*map(sv[1].getValue(i+1),fGLMinVal, fGLMaxVal, -h/2, h/2)+h/2+y+10);
    stroke(0,0,255);
    line(x+50+i,-1.0*map(sv[2].getValue(i),fGLMinVal, fGLMaxVal, -h/2, h/2)+h/2+y+10,x+50+(i+1),-1.0*map(sv[2].getValue(i+1),fGLMinVal, fGLMaxVal, -h/2, h/2)+h/2+y+10);
  }  
  
}


void serialEvent(Serial p) {
    String src = p.readStringUntil('\n');
    if(src != null){    
      String serial_values[] = split(src,"\t");   
      println(src);
      if(serial_values[0].equals("RA")){
        float values[] = {float(serial_values[1]),float(serial_values[2]),float(serial_values[3])};
        for(int i=0;i<sv.length;i++) sv[i].addValue(values[i]);
      }    
      else if(serial_values[0].equals("RP")){
        if(iExecCMDID == 11)iExecCMDID = 0;
        int values[] = {int(serial_values[1]),int(serial_values[2]),int(serial_values[3]),int(serial_values[4]),int(serial_values[5])};
        ssPan.min       = values[0];
        ssPan.slider_pt = values[1];
        ssPan.max       = values[2];
        ssPan.ppm_min   = values[3];
        ssPan.ppm_max   = values[4];          
        TxPanMin.setValue(str(ssPan.min));
        TxPanCenter.setValue(str(ssPan.slider_pt));
        TxPanMax.setValue(str(ssPan.max));
        TxPanPPMMin.setValue(str(ssPan.ppm_min));
        TxPanPPMMax.setValue(str(ssPan.ppm_max));
        SlPanCenterTuning.setValue(ssPan.slider_pt);
      }
      else if(serial_values[0].equals("RT")){
        if(iExecCMDID == 12)iExecCMDID = 0;
        int values[] = {int(serial_values[1]),int(serial_values[2]),int(serial_values[3]),int(serial_values[4]),int(serial_values[5])};
        ssTilt.min       = values[0];
        ssTilt.slider_pt = values[1];
        ssTilt.max       = values[2];
        ssTilt.ppm_min   = values[3];
        ssTilt.ppm_max   = values[4];          
        TxTiltMin.setValue(str(ssTilt.min));
        TxTiltCenter.setValue(str(ssTilt.slider_pt));
        TxTiltMax.setValue(str(ssTilt.max));
        TxTiltPPMMin.setValue(str(ssTilt.ppm_min));
        TxTiltPPMMax.setValue(str(ssTilt.ppm_max));
        SlTiltCenterTuning.setValue(ssTilt.slider_pt);
      }
      else if(serial_values[0].equals("RU")){
        if(iExecCMDID == 13)iExecCMDID = 0;
        int values[] = {int(serial_values[1]),int(serial_values[2]),int(serial_values[3]),int(serial_values[4]),int(serial_values[5])};
        ssRoll.min       = values[0];
        ssRoll.slider_pt = values[1];
        ssRoll.max       = values[2];
        ssRoll.ppm_min   = values[3];
        ssRoll.ppm_max   = values[4];          
        TxRollMin.setValue(str(ssRoll.min));
        TxRollCenter.setValue(str(ssRoll.slider_pt));
        TxRollMax.setValue(str(ssRoll.max));
        TxRollPPMMin.setValue(str(ssRoll.ppm_min));
        TxRollPPMMax.setValue(str(ssRoll.ppm_max));
        SlRollCenterTuning.setValue(ssRoll.slider_pt);
      }
      else if(serial_values[0].equals("RC")){
        if(iExecCMDID == 14)iExecCMDID = 0;
        int values[] = {int(serial_values[1]),int(serial_values[2]),int(serial_values[3]),int(serial_values[4]),int(serial_values[5]),int(serial_values[6]),int(serial_values[7])};
        if(values[0]==0) ssPan.reverse      = false; else ssPan.reverse      = true;  
        if(values[1]==0) ssTilt.reverse     = false; else ssTilt.reverse     = true;  
        if(values[2]==0) ssRoll.reverse     = false; else ssRoll.reverse     = true;  
        ssPan.channel     = values[3];
        ssTilt.channel    = values[4];
        ssRoll.channel    = values[5];      
        DlPanCh.setValue(ssPan.channel); 
        DlTiltCh.setValue(ssTilt.channel); 
        DlRollCh.setValue(ssRoll.channel); 
        if(values[6]==0) bOutputTypePPM     = false;  else bOutputTypePPM    = true;
        if(!ssPan.reverse ) BtnPanReverse.setOff();  else BtnPanReverse.setOn();
        if(!ssTilt.reverse) BtnTiltReverse.setOff(); else BtnTiltReverse.setOn();
        if(!ssRoll.reverse) BtnRollReverse.setOff(); else BtnRollReverse.setOn();        
        if(bOutputTypePPM == true) BtnPPM.setCaptionLabel("PPM"); else BtnPPM.setCaptionLabel("PWM");
      }  
      else if(serial_values[0].equals("WP")){
        if(iExecCMDID == 21)iExecCMDID = 0;
      }
      else if(serial_values[0].equals("WT")){
        if(iExecCMDID == 22)iExecCMDID = 0;
      }
      else if(serial_values[0].equals("WU")){
        if(iExecCMDID == 23)iExecCMDID = 0;
      }
      else if(serial_values[0].equals("WC")){
        if(iExecCMDID == 24)iExecCMDID = 0;
      }
      else if(serial_values[0].equals("SA")){
        if(iExecCMDID == 31)iExecCMDID = 0;
      }
    }
}




void controlEvent(ControlEvent theEvent) {
    switch(theEvent.getController().getId()) {
      case(10):     // dropdownlist of serial ports.
        iDeviceSelectedID = int(theEvent.getController().getValue());
        println(stDeviceList[iDeviceSelectedID]);
        break;
      case(11): // Connect Button
       // button for connecting serial port which was selected id(1).    
       if(!bDeviceConn){
         try{
           serialPort = new Serial(this, stDeviceList[iDeviceSelectedID], 9600);
           theEvent.getController().setCaptionLabel("disconnect");
           bDeviceConn=true;
           surface.setTitle("Terminal 0.1 [connected to \""+stDeviceList[iDeviceSelectedID]+"\" ]");
           readParameterFromSensor();
           ResetGraphScale();
         }catch(Exception e){
           theEvent.getController().setCaptionLabel("connect");
           surface.setTitle("Terminal 0.1 [unconnected]");
           println("Cannot connect:"+stDeviceList[iDeviceSelectedID]);
           bDeviceConn=false;
           serialPort.clear();
           serialPort.stop();
         }
       }else{
         serialPort.clear();
         serialPort.stop();
         theEvent.getController().setCaptionLabel("connect");
         surface.setTitle("Terminal 0.1 [unconnected]");
         bDeviceConn=false;
       }
       break;

       case(19):    
       if(bDeviceConn){
              iExecCMDID = 21;
              while(iExecCMDID != 0){
                String cmd1 = int(ssPan.min)+","+int(ssPan.slider_pt)+","+int(ssPan.max)+","+int(ssPan.ppm_min)+","+int(ssPan.ppm_max)+",WP";
                serialPort.write(cmd1);
                println(cmd1);
                delay(600);
              }
              iExecCMDID = 22;
              while(iExecCMDID != 0){
                String cmd1 = int(ssTilt.min)+","+int(ssTilt.slider_pt)+","+int(ssTilt.max)+","+int(ssTilt.ppm_min)+","+int(ssTilt.ppm_max)+",WT";
                serialPort.write(cmd1);
                println(cmd1);
                delay(600);
              }
              iExecCMDID = 23;
              while(iExecCMDID != 0){
                String cmd1 = int(ssRoll.min)+","+int(ssRoll.slider_pt)+","+int(ssRoll.max)+","+int(ssRoll.ppm_min)+","+int(ssRoll.ppm_max)+",WU";
                serialPort.write(cmd1);
                println(cmd1);
                delay(600);
              }
              iExecCMDID = 24;
              while(iExecCMDID != 0){
                String cmd1 = "";
                if(!ssPan.reverse ) cmd1 += "1"+","; else cmd1 += "0"+",";
                if(!ssTilt.reverse) cmd1 += "1"+","; else cmd1 += "0"+",";
                if(!ssRoll.reverse) cmd1 += "1"+","; else cmd1 += "0"+",";
                cmd1 += ssPan.channel +","+ ssTilt.channel +","+ ssRoll.channel+",";
                if(bOutputTypePPM) cmd1 += "1"+","; else cmd1 += "0"+",";
                cmd1 += "WC";
                serialPort.write(cmd1);
                println(cmd1);
                delay(600);
              }
              iExecCMDID = 31;
              while(iExecCMDID != 0){
                String cmd1 = ",SA";
                serialPort.write(cmd1);
                println(cmd1);
                delay(600);
              }
       }
       break;
      
      case(20): // Pan Slider
        ssPan.slider_pt = theEvent.getController().getValue();
        TxPanCenter.setValue(str(ssPan.slider_pt));
        break;
      case(21): // Pan Minimum
        ssPan.min = float(theEvent.getController().getStringValue());
        break;
      case(22): // Pan Slider Pointer
        ssPan.slider_pt = float(nf(float(theEvent.getController().getStringValue()),3,2));
        SlPanCenterTuning.setValue(ssPan.slider_pt);
        break;
      case(23): // Pan Maximum
        ssPan.max = float(theEvent.getController().getStringValue());
        break;
      case(24): // Pan PPM Minimum Output
        ssPan.ppm_min = int(theEvent.getController().getStringValue());
        break;
      case(25): // Pan PPM Maximum Output
        ssPan.ppm_max = int(theEvent.getController().getStringValue());
        break;
      case(26): // Pan Reverse
        ssPan.reverse = !ssPan.reverse;
        break;
        
      case(30): // Tilt Slider
        ssTilt.slider_pt = theEvent.getController().getValue();
        TxTiltCenter.setValue(str(ssTilt.slider_pt));
        break;
      case(31): // Tilt Minimum
        ssTilt.min = float(theEvent.getController().getStringValue());
        break;
      case(32): // Tilt Slider Pointer
        ssTilt.slider_pt = float(nf(float(theEvent.getController().getStringValue()),3,2));
        SlTiltCenterTuning.setValue(ssTilt.slider_pt);
        break;
      case(33): // Tilt Maximum
        ssTilt.max = float(theEvent.getController().getStringValue());
        break;
      case(34): // Tilt PPM Minimum Output
        ssTilt.ppm_min = int(theEvent.getController().getStringValue());
        break;
      case(35): // Tilt PPM Maximum Output
        ssTilt.ppm_max = int(theEvent.getController().getStringValue());
        break;
      case(36): // Tiltl Reverse
        ssTilt.reverse = !ssTilt.reverse;
        break;
        
        
      case(40): // Roll Slider
        ssRoll.slider_pt = theEvent.getController().getValue();
        TxRollCenter.setValue(str(ssRoll.slider_pt));
        break;
      case(41): // Roll Minimum
        ssRoll.min = float(theEvent.getController().getStringValue());
        break;
      case(42): // Roll Slider Pointer
        ssRoll.slider_pt = float(nf(float(theEvent.getController().getStringValue()),3,2));
        SlRollCenterTuning.setValue(ssRoll.slider_pt);
        break;
      case(43): // Roll Maximum
        ssRoll.max = float(theEvent.getController().getStringValue());
        break;
      case(44): // Roll PPM Minimum Output
        ssRoll.ppm_min = int(theEvent.getController().getStringValue());
        break;
      case(45): // Roll PPM Maximum Output
        ssRoll.ppm_max = int(theEvent.getController().getStringValue());
        break;
      case(46): // Roll Reverse
        ssRoll.reverse = !ssRoll.reverse;
        break;

      case(50): // Output PPM
        if(!bOutputTypePPM){
          bOutputTypePPM = true;
          //theEvent.getController().setCaptionLabel("disconnect");
          BtnPPM.setCaptionLabel("PPM");
        }else{
          bOutputTypePPM = false;
          BtnPPM.setCaptionLabel("PWM");
        }
        break;
      case(52):     // dropdownlist of Pan channel.
        ssPan.channel = int(theEvent.getController().getValue());
        break;
      case(53):     // dropdownlist of Tilt channel.
        ssTilt.channel = int(theEvent.getController().getValue());
        break;
      case(54):     // dropdownlist of Roll channel.
        ssRoll.channel = int(theEvent.getController().getValue());
        break;
        
      case(90): // Default set
        ssPan.slider_pt = 1500;
        ssPan.min     = -90;
        ssPan.max     =  90;
        ssPan.ppm_min = 1000;
        ssPan.ppm_max = 2000;
        ssPan.reverse = false;
        ssTilt.slider_pt = 1500;
        ssTilt.min     = -90;
        ssTilt.max     =  90;
        ssTilt.ppm_min = 1000;
        ssTilt.ppm_max = 2000;
        ssTilt.reverse = false;
        ssRoll.slider_pt = 1500;
        ssRoll.min     = -90;
        ssRoll.max     =  90;
        ssRoll.ppm_min = 1000;
        ssRoll.ppm_max = 2000;
        ssRoll.reverse = false;
        bOutputTypePPM = true;
        BtnPPM.setCaptionLabel("PPM");
        InitializeParameterValues();
        break;
        
      case(91): // Plot Data Clear
        for(int i=0;i<sv.length;i++){
          sv[i].resetValues();
        }
        ResetGraphScale();
        console.clear();
        break;
    }
}

void setupGUI(){
  DlSerialDevList = cp.addDropdownList("serialList")
                      .setLabel("Serial Port")
                      .setPosition(100,15)
                      .setBackgroundColor(color(190))
                      .setItemHeight(20)
                      .setBarHeight(20)
                      .setWidth(200)
                      .setHeight(400)
                      .setId(10);
  
  BtnConnect   = cp.addButton("CONNECT")
                   .setLabel("Connect")
                   .setPosition(310,15)
                   .setHeight(20)
                   .setId(11);

  BtnCalibrate = cp.addButton("CALIBRATE")
                   .setLabel("Calibrate")
                   .setPosition(390,15)
                   .setHeight(20)
                   .setId(15);
     
  BtnSave      = cp.addButton("STORE SETTING")
                   .setLabel("Store setting")
                   .setPosition(480,15)
                   .setHeight(20)
                   .setId(19);
     
     
  // Pan Parameter Settting
  SlPanCenterTuning = cp.addSlider("SlPanCenterTuning")
                        .setLabel("Center")
                        .setPosition(30,320)
                        .setSize(200,20)
                        .setRange(0,255)
                        .setValue(255/2)
                        .setId(20);                   
  TxPanMin    = cp.addTextfield("MIN_PAN")
                  .setLabel("Min Input (deg)")
                  .setPosition(30,350)
                  .setSize(100,20)
                  .setWidth(60)
                  .setFont(createFont("arial",12))
                  .setLabelVisible(false)
                  .setAutoClear(false)
                  .setId(21);
  TxPanCenter = cp.addTextfield("Center_PAN")
                  .setLabel("Center")
                  .setPosition(100,350)
                  .setSize(100,20)
                  .setWidth(60)
                  .setFont(createFont("arial",12))
                  .setLabelVisible(false)
                  .setAutoClear(false)
                  .setId(22);
  TxPanMax    = cp.addTextfield("Max_PAN (deg)")
                  .setLabel("Max Input (deg)")
                  .setPosition(170,350)
                  .setSize(100,20)
                  .setWidth(60)
                  .setFont(createFont("arial",12))
                  .setLabelVisible(false)
                  .setAutoClear(false)
                  .setId(23);
                        
  TxPanPPMMin = cp.addTextfield("Min_OUTPUT_PAN")
                  .setLabel("Min output (um)")
                  .setPosition(30,390)
                  .setSize(100,20)
                  .setWidth(60)
                  .setFont(createFont("arial",12))
                  .setLabelVisible(false)
                  .setAutoClear(false)
                  .setId(24);
  TxPanPPMMax = cp.addTextfield("Max_OUTPUT_PAN")
                  .setLabel("Max output (um)")
                  .setPosition(170,390)
                  .setSize(100,20)
                  .setWidth(60)
                  .setFont(createFont("arial",12))
                  .setLabelVisible(false)
                  .setAutoClear(false)
                  .setId(25);
                  
  BtnPanReverse = cp.addButton("PAN: REVERSE")
                    .setLabel("Reverse")
                    .setPosition(93,460)
                    .setHeight(20)
                    .setSwitch(true)
                    .setOff()
                    .setId(26);
                    
                    
  // Tilt Parameter Settting
  SlTiltCenterTuning = cp.addSlider("SlTiltCenterTuning")
                        .setLabel("Center")
                        .setPosition(280,320)
                        .setSize(200,20)
                        .setRange(0,255)
                        .setValue(255/2)
                        .setId(30);                   
  TxTiltMin    = cp.addTextfield("MIN_TILT")
                  .setLabel("Min Input (deg)")
                  .setPosition(280,350)
                  .setSize(100,20)
                  .setWidth(60)
                  .setFont(createFont("arial",12))
                  .setLabelVisible(false)
                  .setAutoClear(false)
                  .setId(31);
  TxTiltCenter = cp.addTextfield("Center_TILT")
                  .setLabel("Center")
                  .setPosition(350,350)
                  .setSize(100,20)
                  .setWidth(60)
                  .setFont(createFont("arial",12))
                  .setLabelVisible(false)
                  .setAutoClear(false)
                  .setId(32);
  TxTiltMax    = cp.addTextfield("Max_TILT")
                  .setLabel("Max Input (deg)")
                  .setPosition(420,350)
                  .setSize(100,20)
                  .setWidth(60)
                  .setFont(createFont("arial",12))
                  .setLabelVisible(false)
                  .setAutoClear(false)
                  .setId(33);
  TxTiltPPMMin = cp.addTextfield("Min_OUTPUT_TILT")
                  .setLabel("Min output (um)")
                  .setPosition(280,390)
                  .setSize(100,20)
                  .setWidth(60)
                  .setFont(createFont("arial",12))
                  .setLabelVisible(false)
                  .setAutoClear(false)
                  .setId(34);
  TxTiltPPMMax = cp.addTextfield("Max_OUTPUT_TILT")
                  .setLabel("Max output (um)")
                  .setPosition(420,390)
                  .setSize(100,20)
                  .setWidth(60)
                  .setFont(createFont("arial",12))
                  .setLabelVisible(false)
                  .setAutoClear(false)
                  .setId(35);
                  
  BtnTiltReverse = cp.addButton("Tilt: REVERSE")
                    .setLabel("Reverse")
                    .setPosition(343,460)
                    .setHeight(20)
                    .setSwitch(true)
                    .setOff()
                    .setId(36);
                    
                    
  // Roll Parameter Settting
  SlRollCenterTuning = cp.addSlider("SlRollCenterTuning")
                        .setLabel("Center")
                        .setPosition(530,320)
                        .setSize(200,20)
                        .setRange(0,255)
                        .setValue(255/2)
                        .setId(40);                   
  TxRollMin    = cp.addTextfield("MIN_ROLL")
                  .setLabel("Min Input (deg)")
                  .setPosition(530,350)
                  .setSize(100,20)
                  .setWidth(60)
                  .setFont(createFont("arial",12))
                  .setLabelVisible(false)
                  .setAutoClear(false)
                  .setId(41);
  TxRollCenter = cp.addTextfield("Center_ROLL")
                  .setLabel("Center")
                  .setPosition(600,350)
                  .setSize(100,20)
                  .setWidth(60)
                  .setFont(createFont("arial",12))
                  .setLabelVisible(false)
                  .setAutoClear(false)
                  .setId(42);
  TxRollMax    = cp.addTextfield("Max_ROLL")
                  .setLabel("Max Input (deg)")
                  .setPosition(670,350)
                  .setSize(100,20)
                  .setWidth(60)
                  .setFont(createFont("arial",12))
                  .setLabelVisible(false)
                  .setAutoClear(false)
                  .setId(43);
  TxRollPPMMin = cp.addTextfield("Min_OUTPUT_ROLL")
                  .setLabel("Min output (um)")
                  .setPosition(530,390)
                  .setSize(100,20)
                  .setWidth(60)
                  .setFont(createFont("arial",12))
                  .setLabelVisible(false)
                  .setAutoClear(false)
                  .setId(44);
  TxRollPPMMax = cp.addTextfield("Max_OUTPUT_ROLL")
                  .setLabel("Max output (um)")
                  .setPosition(670,390)
                  .setSize(100,20)
                  .setWidth(60)
                  .setFont(createFont("arial",12))
                  .setLabelVisible(false)
                  .setAutoClear(false)
                  .setId(45);
                  
  BtnRollReverse = cp.addButton("Roll: REVERSE")
                    .setLabel("Reverse")
                    .setPosition(593,460)
                    .setHeight(20)
                    .setSwitch(true)
                    .setOff()
                    .setId(46);
                    

                    
  BtnSetDefaultValues = cp.addButton("Set Default Values")
                          .setPosition(680,15)
                          .setHeight(20)
                          .setWidth(100)
                          .setId(90);
                   
  BtnGFClear   = cp.addButton("CLEAR PLOT")
                   .setPosition(600,50)
                   .setHeight(20)
                   .setId(91);
                        
  BtnPPM       = cp.addButton("PPM")
                   .setPosition(600,217)
                   .setHeight(20)
                   .setId(50);
                   
  DlPanCh         = cp.addScrollableList("Pan Ch")
                      .setLabel("Pan")
                      .setPosition(600,255)
                      .setBackgroundColor(color(190))
                      .setItemHeight(20)
                      .setBarHeight(20)
                      .setWidth(40)
                      .setOpen(false)
                      .setId(52);
  DlTiltCh         = cp.addScrollableList("Tilt Ch")
                      .setLabel("Tilt")
                      .setPosition(650,255)
                      .setBackgroundColor(color(190))
                      .setItemHeight(20)
                      .setBarHeight(20)
                      .setWidth(40)
                      .setOpen(false)
                      .setId(53);
  DlRollCh         = cp.addScrollableList("Roll Ch")
                      .setLabel("Roll")
                      .setPosition(700,255)
                      .setBackgroundColor(color(190))
                      .setItemHeight(20)
                      .setBarHeight(20)
                      .setWidth(40)
                      .setOpen(false)
                      .setId(54);
                      
  consoleWindow = cp.addTextarea("txt")
                    .setPosition(600, 85)
                    .setSize(180, 120)
                    .setFont(createFont("", 8))
                    .setLineHeight(14)
                    .setColor(color(200))
                    .setColorBackground(color(0, 100))
                    .setColorForeground(color(255, 100));
}


void InitializeParameterValues(){
  // Pan
  SlPanCenterTuning.setValue(ssPan.slider_pt);
  SlPanCenterTuning.setRange(1000,2000);
  TxPanMin.setValue(str(ssPan.min));
  TxPanCenter.setValue(str(ssPan.slider_pt));
  TxPanMax.setValue(str(ssPan.max));
  TxPanPPMMin.setValue(str(ssPan.ppm_min));
  TxPanPPMMax.setValue(str(ssPan.ppm_max));
  if(ssPan.reverse)BtnPanReverse.setOn();
  else BtnPanReverse.setOff();
  // Tilt
  SlTiltCenterTuning.setValue(ssTilt.slider_pt);
  SlTiltCenterTuning.setRange(1000,2000);
  TxTiltCenter.setValue(str(ssTilt.slider_pt));
  TxTiltMin.setValue(str(ssTilt.min));
  TxTiltMax.setValue(str(ssTilt.max));
  TxTiltPPMMin.setValue(str(ssTilt.ppm_min));
  TxTiltPPMMax.setValue(str(ssTilt.ppm_max));
  if(ssTilt.reverse)BtnTiltReverse.setOn();
  else BtnTiltReverse.setOff();
  // Roll
  SlRollCenterTuning.setValue(ssRoll.slider_pt);
  SlRollCenterTuning.setRange(1000,2000);
  TxRollCenter.setValue(str(ssRoll.slider_pt));
  TxRollMin.setValue(str(ssRoll.min));
  TxRollMax.setValue(str(ssRoll.max));
  TxRollPPMMin.setValue(str(ssRoll.ppm_min));
  TxRollPPMMax.setValue(str(ssRoll.ppm_max));
  if(ssRoll.reverse)BtnRollReverse.setOn();
  else BtnRollReverse.setOff();
  
  DlPanCh.setType(ScrollableList.DROPDOWN)
          .setValue(7); 
  DlTiltCh.setType(ScrollableList.DROPDOWN)
          .setValue(8); 
  DlRollCh.setType(ScrollableList.DROPDOWN)
          .setValue(9);             
}

void readParameterFromSensor(){
 if(bDeviceConn){
   iExecCMDID = 11;
   while(iExecCMDID != 0){
       String cmd1 = ",RP";
       serialPort.write(cmd1);
       delay(100);
   }
   iExecCMDID = 12;
   while(iExecCMDID != 0){
       String cmd1 = ",RT";
       serialPort.write(cmd1);
       delay(100);
   }
   iExecCMDID = 13;
   while(iExecCMDID != 0){
       String cmd1 = ",RU";
       serialPort.write(cmd1);
       delay(100);
   }
   iExecCMDID = 14;
   while(iExecCMDID != 0){
       String cmd1 = ",RC";
       serialPort.write(cmd1);
       delay(100);
   }
 }
}