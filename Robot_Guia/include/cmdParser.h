#ifndef CMDPARSER_H
#define CMDPARSER_H

#include "motors.h"

/*
  LIBRERIA cmdParser.h 
  RECIBE PARAMETROS DE NUESTRO DISPOSITIVO Y LAS MANDA A LAS FUNCIONES DE MOTORS.H 
*/

//Parametros que puede recibir
#define MOTOR_LEFT_FORWARD      'L'
#define MOTOR_LEFT_BACKWARD     'l'
#define MOTOR_LEFT_STOP         'k'
#define MOTOR_RIGHT_FORWARD     'R'
#define MOTOR_RIGHT_BACKWARD    'r'
#define MOTOR_RIGHT_STOP        'e'
#define TOGLE_LED               '*'
#define LED_BUILTIN             13

//funciones 
void cmdParser(char cmd, uint16_t value);

#endif