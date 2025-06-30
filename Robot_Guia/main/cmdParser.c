#include "../include/cmdParser.h"

//depende de que parametro recibe lo manda a la funcion de motor.h
void cmdParser(char cmd, uint16_t value){
  switch(cmd){
    case MOTOR_LEFT_FORWARD:
      motorsSetVel(MOTOR_LEFT,MOTOR_DIR_FORWARD,value);
      break;
    case MOTOR_LEFT_BACKWARD:
      motorsSetVel(MOTOR_LEFT,MOTOR_DIR_BACKWARD,value);
      break;
    case MOTOR_LEFT_STOP:
      motorStop(MOTOR_LEFT);
      break;  
    case MOTOR_RIGHT_FORWARD:
      motorsSetVel(MOTOR_RIGHT,MOTOR_DIR_FORWARD,value);
      break;
    case MOTOR_RIGHT_BACKWARD:
      motorsSetVel(MOTOR_RIGHT,MOTOR_DIR_BACKWARD,value);
      break;
    case MOTOR_RIGHT_STOP:
      motorStop(MOTOR_RIGHT);
      break;    
    case TOGLE_LED:
      gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));
      break;  
  }
}