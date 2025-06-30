#ifndef MOTORS_H
#define MOTORS_H

#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include <stdbool.h>
/* 
  LIBRERIA motor.h
  INICIALIZADOR DE LOS MOTORES Y RECIBE PARAMETROS 
  DE MOTOR, DIRECCION Y PWM PARA LUEGO MOVER LOS MOTORES.
  Left/L se refiere a motor izquierdo.
  Right/R se refiere a motor derecho.
*/

//motor pin, Estas variables tienen el numero de los pines de pwm y enable, si lo usas en l298 convencional los dos IN de cada motor debes asignar y el defin USE_PWM ponerlo en 0
#define MOTOR_L_IN1_PIN 32 // PWM  
#define MOTOR_L_IN2_PIN 33 // enable
#define MOTOR_R_IN3_PIN 25 // PWM 
#define MOTOR_R_IN4_PIN 26 // enable

//esp32 channel, Estas variables tienen el numero de canal que se va a utilizar para el pwm de cada motor
#define MOTOR_L_IN1_CHANNEL 2 
#define MOTOR_L_IN2_CHANNEL 3
#define MOTOR_R_IN3_CHANNEL 4
#define MOTOR_R_IN4_CHANNEL 5

//motors, Estas variables tienen el numero del case (casos), que van a mover al motor 1(LEFT 0) y 2(RIGHT 1)
#define MOTOR_LEFT     0
#define MOTOR_RIGHT    1

//motors direction, Estas variables tienen el numero del case (casos), que van a mover los motores para adelante o atras
#define MOTOR_DIR_BACKWARD 0
#define MOTOR_DIR_FORWARD 1

//FLAG por si conectamos los motores al reves los cambiamos
#define MOTOR_LEFT_REVERSE  0
#define MOTOR_RIGHT_REVERSE 0

//Frecuencia y resolucion PWM de los motores
#define MOTOR_FREQ_PWM 5000
#define MOTOR_RESOLUTION_PWM 8
#define USE_PWM 1 //FLAG por si queremos deshabilitarlo y dejar de utilizar pwm.

#define LED_BUILTIN 13 //pin_led

//funciones 
void motorsShutDown();
void configure_channel();
void assigned_channel();
void motorsSetup();
void motorsSetVel(uint8_t motor, uint8_t dir, uint16_t vel);
void motorStop(uint8_t motor);
void ledcAttachChannel(uint8_t pin, uint32_t freq, ledc_timer_bit_t resolucion, ledc_channel_t channel);
void ledcWrite(ledc_channel_t channel, uint32_t duty);

#endif