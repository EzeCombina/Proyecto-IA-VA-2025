#include "../include/motors.h"

//Esta variable es para que todos los pines usen la misma 
//frecuencia de reloj y no se inicialice la función de reloj 
//cada vez que se usa un pin diferente
static bool ledc_timer_initialized = false;

void ledcWrite(ledc_channel_t channel, uint32_t duty) {
   if (channel > LEDC_CHANNEL_7) {
        printf("Error_1: canal LEDC inválido (%d)\n", channel);
        return;
    }
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
}

void ledcAttachChannel(uint8_t pin, uint32_t freq, ledc_timer_bit_t resolucion, ledc_channel_t channel) {

   if (channel < 0 || channel > LEDC_CHANNEL_7) {
        printf("Error_2: canal LEDC inválido (%d)\n", channel);
        return;
    }

  // Configurar el timer solo una vez
    if (!ledc_timer_initialized) {
        ledc_timer_config_t timer_conf = {
            .speed_mode       = LEDC_HIGH_SPEED_MODE,
            .timer_num        = LEDC_TIMER_0,
            .duty_resolution  = resolucion,
            .freq_hz          = freq,
            .clk_cfg          = LEDC_AUTO_CLK
        };
        ledc_timer_config(&timer_conf);
        ledc_timer_initialized = true;
    }

    // Configurar el canal PWM
    ledc_channel_config_t channel_conf = {
        .channel    = channel,
        .duty       = 0,
        .gpio_num   = pin,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    
    ledc_channel_config(&channel_conf);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
}

//Configuracion de canales para los motores
void configure_channel()
{
  ledcAttachChannel(MOTOR_L_IN1_PIN, MOTOR_FREQ_PWM, MOTOR_RESOLUTION_PWM, MOTOR_L_IN1_CHANNEL);
  ledcAttachChannel(MOTOR_L_IN2_PIN, MOTOR_FREQ_PWM, MOTOR_RESOLUTION_PWM, MOTOR_L_IN2_CHANNEL);
  ledcAttachChannel(MOTOR_R_IN3_PIN, MOTOR_FREQ_PWM, MOTOR_RESOLUTION_PWM, MOTOR_R_IN3_CHANNEL);
  ledcAttachChannel(MOTOR_R_IN4_PIN, MOTOR_FREQ_PWM, MOTOR_RESOLUTION_PWM, MOTOR_R_IN4_CHANNEL);
}

//Asignacion de canales en el pin que deseamos
void assigned_channel()
{
    // Configurar GPIO18, GPIO19 y GPIO21 como salida
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,                   // Sin interrupciones
        .mode = GPIO_MODE_OUTPUT,                         // Modo salida
        .pin_bit_mask = (1ULL << MOTOR_L_IN1_PIN) | 
                        (1ULL << MOTOR_L_IN2_PIN) |
                        (1ULL << MOTOR_R_IN3_PIN) |
                        (1ULL << MOTOR_R_IN4_PIN),        // Pines que queremos configurar
        .pull_down_en = GPIO_PULLDOWN_DISABLE,            // Sin pull-down
        .pull_up_en = GPIO_PULLUP_DISABLE                 // Sin pull-up
    };

    gpio_config(&io_conf);  // Aplica la configuración
}

//Para que los motores arrancan en 0
void motorsShutDown(){
  ledcWrite(MOTOR_L_IN1_CHANNEL, 0);
  ledcWrite(MOTOR_L_IN2_CHANNEL, 0);
  ledcWrite(MOTOR_R_IN3_CHANNEL, 0);
  ledcWrite(MOTOR_R_IN4_CHANNEL, 0);
}

//Setup motores
void motorsSetup(){
  assigned_channel();
  configure_channel();      //primero debo de configurar los canales de PWM
  motorsShutDown();         //para luego usar el ledcWrite(pin,duty)
}

//Funcion en donde llega el motor que hay que mover, la direccion que se le desea dar y su velocidad.
void motorsSetVel(uint8_t motor, uint8_t dir, uint16_t vel){
  switch(motor){
    case MOTOR_LEFT:
       switch(dir){
          case MOTOR_DIR_BACKWARD: 
            if(MOTOR_LEFT_REVERSE == 0){                     //Primera forma de conectarlo
              if(USE_PWM==1){
                ledcWrite(MOTOR_L_IN1_CHANNEL, vel);
                ledcWrite(MOTOR_L_IN2_CHANNEL, 0);
              }else{
                ledcWrite(MOTOR_L_IN1_CHANNEL, 255);
                ledcWrite(MOTOR_L_IN2_CHANNEL, 0);
              }
            }else{
              if(USE_PWM==1){                                //Segunda forma de conectarlo
                ledcWrite(MOTOR_L_IN1_CHANNEL, 0);
                ledcWrite(MOTOR_L_IN2_CHANNEL, vel);
              }else{
                ledcWrite(MOTOR_L_IN1_CHANNEL, 0);
                ledcWrite(MOTOR_L_IN2_CHANNEL, 255);
              }
            }
            break;
          case MOTOR_DIR_FORWARD:
            if(MOTOR_LEFT_REVERSE == 0){
              if(USE_PWM==1){
                ledcWrite(MOTOR_L_IN1_CHANNEL, 0);                   //adelante, 32 = 0, 33 = 1 
                ledcWrite(MOTOR_L_IN2_CHANNEL, vel);
              }else{
                ledcWrite(MOTOR_L_IN1_CHANNEL, 0);                   //adelante, 32 = 0, 33 = 1 
                ledcWrite(MOTOR_L_IN2_CHANNEL, 255);
              }
            }else{
              if(USE_PWM==1){
                ledcWrite(MOTOR_L_IN1_CHANNEL, vel);
                ledcWrite(MOTOR_L_IN2_CHANNEL, 0);
              }else{
                ledcWrite(MOTOR_L_IN1_CHANNEL, 255);
                ledcWrite(MOTOR_L_IN2_CHANNEL, 0);
              }
            }
            break;        
       }
      break;
    case MOTOR_RIGHT:
      switch(dir){
          case MOTOR_DIR_BACKWARD:
            if(MOTOR_RIGHT_REVERSE == 0){
              if(USE_PWM==1){
                ledcWrite(MOTOR_R_IN3_CHANNEL, 0);                          //atrás, 25 = 0, 26 = 1 
                ledcWrite(MOTOR_R_IN4_CHANNEL, vel); 
              }else{
                ledcWrite(MOTOR_R_IN3_CHANNEL, 0);                          //atrás, 25 = 0, 26 = 1 
                ledcWrite(MOTOR_R_IN4_CHANNEL, 255);
              }
              
            }else{
              if(USE_PWM==1){
                ledcWrite(MOTOR_R_IN3_CHANNEL, vel);              //atrás, 25 = 1, 26 = 0 
                ledcWrite(MOTOR_R_IN4_CHANNEL, 0);
              }else{
                ledcWrite(MOTOR_R_IN3_CHANNEL, 255);              //atrás, 25 = 1, 26 = 0 
                ledcWrite(MOTOR_R_IN4_CHANNEL, 0);
              }
              
            }
            break;
          case MOTOR_DIR_FORWARD:
            if(MOTOR_RIGHT_REVERSE == 0){
              if(USE_PWM==1){
                ledcWrite(MOTOR_R_IN3_CHANNEL, vel);                
                ledcWrite(MOTOR_R_IN4_CHANNEL, 0);
              }else{
                ledcWrite(MOTOR_R_IN3_CHANNEL, 255);                 
                ledcWrite(MOTOR_R_IN4_CHANNEL, 0);
              }    
            }else{
              if(USE_PWM==1){
                ledcWrite(MOTOR_R_IN3_CHANNEL, 0);                          
                ledcWrite(MOTOR_R_IN4_CHANNEL, vel);
              }else{
                ledcWrite(MOTOR_R_IN3_CHANNEL, 0);                        
                ledcWrite(MOTOR_R_IN4_CHANNEL, 255);
              }
            }
            break;
           default:
            gpio_set_level(LED_BUILTIN,0);
            break;
       }
      break;  
  }
}

//Frenar motores
void motorStop(uint8_t motor){
  switch(motor){
    case MOTOR_LEFT:
      if(USE_PWM==1){
        ledcWrite(MOTOR_L_IN1_CHANNEL,0);    
        ledcWrite(MOTOR_L_IN2_CHANNEL,0);
      }else{
        ledcWrite(MOTOR_L_IN1_CHANNEL,0);    
        ledcWrite(MOTOR_L_IN2_CHANNEL,0);
      }
      break;
    case MOTOR_RIGHT:
      if(USE_PWM==1){
        ledcWrite(MOTOR_R_IN3_CHANNEL,0);    
        ledcWrite(MOTOR_R_IN4_CHANNEL,0);
      }else{
        ledcWrite(MOTOR_R_IN3_CHANNEL,0);    
        ledcWrite(MOTOR_R_IN4_CHANNEL,0);
      }
      break;
  }
}