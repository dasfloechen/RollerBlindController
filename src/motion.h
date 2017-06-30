#ifndef _MOTION_H
#define _MOTION_H

#include <Arduino.h>

/* DEBUG STUFF */
#define MOTION_DEBUG 1
#define DEBUGSERIAL Serial

#define EEPROM_MAX_POS_ADDR   (0x0)

/* PINS */
#define PIN_STEP  (2)
#define PIN_DIR   (3)
#define PIN_EN    (4)
#define PIN_CS    (5)
#define PIN_ENDSTOP (6)

/* MOTOR DRIVER CONFIGURATION */
#define DRIVER_MICROSTEPS   (16)


/* MOTOR CONFIGURATION */
#define MOTOR_STEPS_PER_ROTATION    (200)
#define MOTOR_MAX_SPEED             (3600) //STEPS/s
#define MOTOR_MAX_ACCELERATION      (1800) //STEPS/s²



typedef enum {
    DIR_UP = 0,
    DIR_DOWN = 1
} roller_direction_t;

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1
} motor_direction_t;


class Motion {
  public:
    Motion(void);

    int begin(void);
    void process(void);

    int doHoming(void);

    int moveTo(int32_t steps, uint32_t speed, uint8_t ignoreMaxLimit=false);

    int startSetMaxLimit(uint32_t speed);
    int stopSetMaxLimit(void);

    void stop(void);

    void showStat(void);

    void setEndstopPolarity(uint8_t polarity);
    void setEndstopPullup(uint8_t isEnabled);
    void setHomingSpeed(uint32_t homingSpeed);
    void setDirectionParameter(uint8_t upDirection, uint8_t relative);
    void setAutoHoming(uint8_t value);

  protected:

    void _savePositionAsMaxPosition(void);
    void _loadMaxPosition(void);

    uint8_t _isRunning = false;
    uint8_t _isMaxLimitSearch = false;
    uint8_t _ignoreMaxLimit = false;
    uint8_t _isHoming = false;
    uint8_t _homingOk = false;

    uint8_t _endstopPolarity = LOW;
    uint8_t _endstopPullUp = true;

    uint8_t _homingDirection = DIR_UP;
    uint8_t _upDirection = MOTOR_LEFT;
    uint8_t _downDirection = MOTOR_RIGHT;

    uint8_t _currentDirection = MOTOR_LEFT;
    int32_t _positionLimitMax = INT32_MAX;

    uint32_t _homingSpeed = 1800;
    uint8_t _autoHoming = false;


};

#endif //_MOTION_H
