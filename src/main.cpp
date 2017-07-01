
#include <Arduino.h>
#include <EEPROM.h>
#include "motion.h"
#include <elapsedMillis.h>
#include <Button.h>

Motion motionControl;
Motion* mPtr;
IntervalTimer stepTimer;

elapsedMillis _adcTimer;
Button buttonUp(22);
Button buttonDown(21);
Button buttonHome(23);


#define UART_RX_SIZE  (128)
#define CMDUART Serial
static void UartHandler(void);

void loop (void)
{
  UartHandler();

  if(buttonUp.released()){
    if(motionControl.distanceToGo() > 0){
      motionControl.stop();
    }else if(motionControl.distanceToGo() == 0){
      Serial.println("UP");
      motionControl.moveTo(0, 350);
    }
  }

  if(buttonDown.released()){
    if(motionControl.distanceToGo() < 0){
      motionControl.stop();
    }else if(motionControl.distanceToGo() == 0){
      Serial.println("DOWN");
      motionControl.moveTo(motionControl.getMaxPosition(), 350);
    }
  }

  if(buttonHome.released()){
    if(!motionControl.isHomed() && !motionControl.isHoming())
      motionControl.doHoming();
  }

  if(_adcTimer > 100){
    //Serial.print("END:");
    //Serial.println(digitalRead(PIN_ENDSTOP));
    _adcTimer = 0;
  }

}

void setup(void)
{
  mPtr = &motionControl;

  buttonHome.begin();

  motionControl.begin();
  motionControl.setEndstopPolarity(LOW);
  motionControl.setHomingSpeed(100);
  motionControl.setAutoHoming(false);
  motionControl.setDirectionParameter(MOTOR_RIGHT, DIR_UP);


  stepTimer.begin([]{mPtr->process();}, 5);
  stepTimer.priority(130);

  buttonUp.begin();
  buttonDown.begin();
}

static void CommandParser(char * s, int len)
{
  char cmd[5];
  char * params;
  params = s + 5;
  int speed = 0 ;
  int steps = 0;
  if(len < 4)
    return;

  cmd[0] = toupper(s[0]);
  cmd[1] = toupper(s[1]);
  cmd[2] = toupper(s[2]);
  cmd[3] = toupper(s[3]);
  cmd[4] = 0;

  CMDUART.print("CMD:");
  CMDUART.println(cmd);

  if(strcmp("PING", cmd) == 0){
    CMDUART.println("#PING");
  }

  if(strcmp("STOP", cmd) == 0){
    motionControl.stop();
  }

  //GOTO STEPS SPEED
  if(strcmp("GOTO", cmd) == 0){
    sscanf(params, "%d %d", &steps, &speed);
    CMDUART.printf("GOTO -> STEPS:%d SPEED:%d", steps, speed);
    CMDUART.println("");
    motionControl.moveTo(steps, speed, false);
  }

  if(strcmp("RELG", cmd) == 0){
    sscanf(params, "%d %d", &steps, &speed);
    CMDUART.printf("RELG -> STEPS:%d SPEED:%d", steps, speed);
    CMDUART.println("");
    motionControl.moveTo(steps, speed, true);
  }

  if(strcmp("HOME", cmd) == 0){
    motionControl.doHoming();
  }

  if(strcmp("STAT", cmd) == 0){
    //?? thing what statuses i need here
    motionControl.showStat();
  }

  //Start max length calibration
  if(strcmp("STML", cmd) == 0){
    motionControl.startSetMaxLimit(250);
  }

  //Stop max length calibration and save value
  if(strcmp("ENML", cmd) == 0){
    motionControl.stopSetMaxLimit();
  }
}

static void UartHandler(void)
{
  static int rxIn = 0;
  static char buffer[UART_RX_SIZE];
  int c;

  while(CMDUART.available()){
    c = CMDUART.read();
    if((c == '\n') || (c == '\r'))
    {
      CommandParser(buffer, rxIn++);
      rxIn = 0;
      return;
    }

    buffer[rxIn] = (char)c;
    rxIn++;
    if(rxIn >= UART_RX_SIZE){
      rxIn = 0;
      CMDUART.println("#ERR-OVERFLOW");
    }
  }
}
