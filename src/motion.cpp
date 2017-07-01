

#include <SPI.h>
#include <AccelStepper.h>
#include <TMC2130Stepper.h>
#include <EEPROM.h>

#include "motion.h"

AccelStepper _stepperControl(1, PIN_STEP, PIN_DIR);
TMC2130Stepper _stepper(PIN_EN, PIN_DIR, PIN_STEP, PIN_CS);

Motion::Motion(void)
{

}


int Motion::begin(void)
{
  #ifdef MOTION_DEBUG
  DEBUGSERIAL.print("MOTION-CONTROL START BEGIN ...");
  #endif

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_CS, OUTPUT);

  pinMode(PIN_ENDSTOP, INPUT_PULLUP);

  _stepper.begin();
  _stepper.setCurrent(1000, 0.11, 0.5);
  _stepper.hold_current(1);
  _stepper.stealthChop(1);
  _stepper.microsteps(16);
  _stepper.interpolate(1)  ;

  _isRunning = false;
  _isHoming = false;
  _homingOk = false;
  _isMaxLimitSearch = false;

  digitalWrite(PIN_EN, LOW);

  _stepperControl.setMaxSpeed(MOTOR_MAX_SPEED*DRIVER_MICROSTEPS);
  _stepperControl.setAcceleration(MOTOR_MAX_ACCELERATION);
  _stepperControl.setCurrentPosition(0);

  _loadMaxPosition();


  #ifdef MOTION_DEBUG
  DEBUGSERIAL.println("DONE");
  #endif

  return true;
}

/**
 * process
 * -------
 * Periodicly called method to do the motor movement
 *
 */
void Motion::process(void)
{
  if(_isRunning){
    if(_stepperControl.currentPosition() % DRIVER_MICROSTEPS == 0){
      if((_stepperControl.currentPosition() / 16 >= _positionLimitMax) && (_ignoreMaxLimit == false) && (_stepperControl.targetPosition() - _stepperControl.currentPosition() > 0))
      {
        _isRunning = false;
        #ifdef MOTION_DEBUG
        DEBUGSERIAL.println("MAXIMUM POSITION REACHED");
        #endif
        return;
      }
      if(_isRunningStop){
        _isRunningStop = false;
        _isRunning = false;
        #ifdef MOTION_DEBUG
        DEBUGSERIAL.println("RUNNING STOP");
        #endif
        return;
      }
    }

    if(!_stepperControl.run()){
      _isRunning = false;
      #ifdef MOTION_DEBUG
      DEBUGSERIAL.println("TARGET POSITION REACHED");
      #endif
      return;
    }
  }else if(_isHoming){
    if(_stepperControl.currentPosition() % DRIVER_MICROSTEPS == 0){
      if(digitalRead(PIN_ENDSTOP) == _endstopPolarity){
        #ifdef MOTION_DEBUG
        DEBUGSERIAL.println("HOMING COMPLETE");
        #endif
        _isHoming = false;
        _homingOk = true;
        _stepperControl.setCurrentPosition(0);
        return;
      }
    }
    _stepperControl.runSpeed();
  }else if(_isMaxLimitSearch){
    if(_isMaxLimitSearchStop){
      if(_stepperControl.currentPosition() % DRIVER_MICROSTEPS == 0){
        _isMaxLimitSearch = false;
        _isMaxLimitSearchStop = false;
        return;
      }
    }
    _stepperControl.runSpeed();
  }
}

void Motion::stop(void)
{
    if(_isRunning){
      _stepperControl.stop();
      //_isRunningStop = true;
    }else{
      _isHoming = false;
    }

    #ifdef MOTION_DEBUG
    DEBUGSERIAL.println("MOTOR STOP!");
    #endif
}

int Motion::doHoming(void)
{
  #ifdef MOTION_DEBUG
  DEBUGSERIAL.print("START HOMING SPEED:");
  DEBUGSERIAL.println(_homingSpeed);
  #endif

  if(_isRunning){
      #ifdef MOTION_DEBUG
      DEBUGSERIAL.println("MOTOR RUNNING - ABORT HOMING");
      #endif

      return false;
  }

  if(_isMaxLimitSearch){
    #ifdef MOTION_DEBUG
    DEBUGSERIAL.println("SET MAX LIMIT RUNNING - ABORT");
    #endif
    return false;
  }

  _homingOk = false;

  _stepperControl.setCurrentPosition(0);

  _stepperControl.setMaxSpeed(_homingSpeed*DRIVER_MICROSTEPS);

  if(_homingDirection == DIR_UP){
    #ifdef MOTION_DEBUG
    DEBUGSERIAL.println("HOMING UP");
    #endif
    _stepperControl.setSpeed((_homingSpeed*DRIVER_MICROSTEPS)*(-1.0f));
  }else{
    #ifdef MOTION_DEBUG
    DEBUGSERIAL.println("HOMING DOWN");
    #endif
    _stepperControl.setSpeed((_homingSpeed*DRIVER_MICROSTEPS));
  }

  _isHoming = true;
  return true;
}

int Motion::moveTo(int32_t steps, uint32_t speed, uint8_t relative)
{
  #ifdef MOTION_DEBUG
  DEBUGSERIAL.print("MOVE-TO  STEPS:");
  DEBUGSERIAL.print(steps);
  DEBUGSERIAL.print(" SPEED:");
  DEBUGSERIAL.print(speed);
  DEBUGSERIAL.print(" RELATIVE:");
  DEBUGSERIAL.println(relative);
  #endif

  if(_isMaxLimitSearch){
    #ifdef MOTION_DEBUG
    DEBUGSERIAL.println("SET MAX LIMIT RUNNING - ABORT");
    #endif
    return false;
  }

  if(_isHoming)
  {
    #ifdef MOTION_DEBUG
    DEBUGSERIAL.println("HOMING RUNNING - ABORT moveTo");
    #endif
    return false;
  }

  if(!_homingOk)
  {
    if(!_autoHoming){
      DEBUGSERIAL.println("MOVETO ERROR - NOT HOMED! - ABORT");
      return false;
    }else{
      #ifdef MOTION_DEBUG
      DEBUGSERIAL.println("MOVETO ERROR - NOT HOMED! - START AUTO HOMING");
      doHoming();
      #endif
    }
  }

  _stepperControl.setSpeed(float(speed * DRIVER_MICROSTEPS));
  _stepperControl.setMaxSpeed(float(speed * DRIVER_MICROSTEPS));
  if(relative){
    _stepperControl.move(steps * DRIVER_MICROSTEPS);
  }else{
    _stepperControl.moveTo(steps * DRIVER_MICROSTEPS);
  }
  _isRunning = true;

  return true;
}

int Motion::startSetMaxLimit(uint32_t speed){

  int32_t searchSpeed = speed;

  if(searchSpeed > MOTOR_MAX_SPEED)
    searchSpeed = MOTOR_MAX_SPEED;

  searchSpeed *= DRIVER_MICROSTEPS;

  #ifdef MOTION_DEBUG
  DEBUGSERIAL.println("BEGIN SET MAX LIMIT");
  #endif

  if(_isMaxLimitSearch)
  {
    #ifdef MOTION_DEBUG
    DEBUGSERIAL.println("SET MAX LIMIT ALEARDY RUNNING! - ABORT!");
    #endif
    return false;
  }

  if(_isHoming)
  {
    #ifdef MOTION_DEBUG
    DEBUGSERIAL.println("HOMING RUNNING - ABORT SETMAXLIMIT");
    #endif
    return false;
  }

  if(_isRunning){
      #ifdef MOTION_DEBUG
      DEBUGSERIAL.println("MOTOR RUNNING - ABORT HOMING");
      #endif
      return false;
  }

  if(!_homingOk){
    #ifdef MOTION_DEBUG
    DEBUGSERIAL.println("SYSTEM NOT HOMED! - ABORT!");
    #endif
    return false;
  }

  _stepperControl.setMaxSpeed(searchSpeed*DRIVER_MICROSTEPS);
  if(_homingDirection == DIR_UP){
    _stepperControl.setSpeed(searchSpeed);
  }else{
    _stepperControl.setSpeed(searchSpeed);
  }

  _isMaxLimitSearch = true;

  return true;
}

int Motion::stopSetMaxLimit(void){
  #ifdef MOTION_DEBUG
  DEBUGSERIAL.println("END SET MAX LIMIT");
  #endif

  if(!_isMaxLimitSearch)
  {
    #ifdef MOTION_DEBUG
    DEBUGSERIAL.println("SET MAX LIMIT NOT RUNNING! - ABORT!");
    #endif
    return false;
  }

  //_isMaxLimitSearch = false;
  _isMaxLimitSearchStop = true;

  _savePositionAsMaxPosition();


  return true;
}

void Motion::showStat(void)
{
    DEBUGSERIAL.print("ENDSTOP:");
    DEBUGSERIAL.println(digitalRead(PIN_ENDSTOP));
    DEBUGSERIAL.print("POS:");
    DEBUGSERIAL.println(_stepperControl.currentPosition());
    DEBUGSERIAL.print("POS-MAX:");
    DEBUGSERIAL.println(_positionLimitMax);

    DEBUGSERIAL.print("STANDSTILL:");
    DEBUGSERIAL.println(_stepper.stst());

    DEBUGSERIAL.print("FULLSTEP:");
    DEBUGSERIAL.println(_stepper.fsactive());
}


long Motion::distanceToGo(void){
  if(!_isRunning)
    return 0;

  return _stepperControl.distanceToGo();
}

uint8_t Motion::isHomed(void){
  return _homingOk;
}

uint8_t Motion::isHoming(void){
  return _isHoming;
}

int32_t Motion::getMaxPosition(void){
  return _positionLimitMax;
}

void Motion::setEndstopPolarity(uint8_t polarity){
  _endstopPolarity = polarity;

  #ifdef MOTION_DEBUG
  DEBUGSERIAL.print("SET-ENDSTOP-POLARITY: ");
  if(polarity == HIGH){
    DEBUGSERIAL.println("ACTIVE-HIGH");
  }else{
    DEBUGSERIAL.println("ACTIVE-LOW");
  }
  #endif
}

void Motion::setEndstopPullup(uint8_t isEnabled){
  _endstopPullUp = isEnabled;
  if(_endstopPullUp){
    pinMode(PIN_ENDSTOP, INPUT_PULLUP);
  }else{
    pinMode(PIN_ENDSTOP, INPUT);
  }

  #ifdef MOTION_DEBUG
  DEBUGSERIAL.print("SET-ENDSTOP-PULLUP: ");
  DEBUGSERIAL.println(isEnabled);
  #endif
}

void Motion::setHomingSpeed(uint32_t homingSpeed){
  _homingSpeed = homingSpeed;
  if(_homingSpeed > MOTOR_MAX_SPEED)
    _homingSpeed = MOTOR_MAX_SPEED;

  #ifdef MOTION_DEBUG
  DEBUGSERIAL.print("SET-HOMING-SPEED: ");
  DEBUGSERIAL.println(_homingSpeed);
  #endif
}

void Motion::setDirectionParameter(uint8_t upDirection, uint8_t homingDirection){
  _upDirection = upDirection;
  _downDirection = MOTOR_LEFT;

  if(_upDirection == MOTOR_LEFT)
    _downDirection = MOTOR_RIGHT;

  _homingDirection = homingDirection;

  if(_downDirection == MOTOR_LEFT){
    _stepperControl.setPinsInverted(true);
  }else{
    _stepperControl.setPinsInverted(false);
  }

  #ifdef MOTION_DEBUG
  DEBUGSERIAL.print("SET-DIRECTIONS: UP:");
  if(_upDirection == MOTOR_LEFT){
    DEBUGSERIAL.print("LEFT");
  }else{
    DEBUGSERIAL.print("RIGHT");
  }

  DEBUGSERIAL.print("  DOWN:");
  if(_downDirection == MOTOR_LEFT){
    DEBUGSERIAL.print("LEFT");
  }else{
    DEBUGSERIAL.print("RIGHT");
  }

  DEBUGSERIAL.print("  HOMING:");
  if(_homingDirection == DIR_UP){
    DEBUGSERIAL.print("UP");
  }else{
    DEBUGSERIAL.print("DOWN");
  }
  DEBUGSERIAL.println("");

  #endif
}

void Motion::setAutoHoming(uint8_t value){
  _autoHoming = value;

  #ifdef MOTION_DEBUG
  DEBUGSERIAL.print("SET-AUTO-HOMING: ");
  DEBUGSERIAL.println(_autoHoming);
  #endif
}

/* protected */

void Motion::_savePositionAsMaxPosition(){
    int32_t pos = _stepperControl.currentPosition() / DRIVER_MICROSTEPS;
    EEPROM.update(EEPROM_MAX_POS_ADDR + 0, (uint8_t)(pos >> 24));
    EEPROM.update(EEPROM_MAX_POS_ADDR + 1, (uint8_t)(pos >> 16));
    EEPROM.update(EEPROM_MAX_POS_ADDR + 2, (uint8_t)(pos >> 8));
    EEPROM.update(EEPROM_MAX_POS_ADDR + 3, (uint8_t)(pos >> 0));

    #ifdef MOTION_DEBUG
    DEBUGSERIAL.print("SAVING MAX POSTION:");
    DEBUGSERIAL.print(pos);
    DEBUGSERIAL.print(" @ 0x");
    DEBUGSERIAL.println(EEPROM_MAX_POS_ADDR, HEX);
    #endif

    _loadMaxPosition();
}

void Motion::_loadMaxPosition(){
  int32_t pos = 0;

  pos |= (EEPROM.read(EEPROM_MAX_POS_ADDR + 0) << 24);
  pos |= (EEPROM.read(EEPROM_MAX_POS_ADDR + 1) << 16);
  pos |= (EEPROM.read(EEPROM_MAX_POS_ADDR + 2) << 8);
  pos |= (EEPROM.read(EEPROM_MAX_POS_ADDR + 3) << 0);

  _positionLimitMax = pos;

  #ifdef MOTION_DEBUG
  DEBUGSERIAL.print("LOADING MAX POSTION:");
  DEBUGSERIAL.print(pos);
  DEBUGSERIAL.print(" @ 0x");
  DEBUGSERIAL.println(EEPROM_MAX_POS_ADDR, HEX);
  #endif
}
