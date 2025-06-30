/*==================[Inclusiones]======================*/
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include "../include/motors.h"
#include "../include/cmdParser.h"

/*==================[Definiciones]======================*/
#define PORT                3333
#define PROCESADORA         0
#define PROCESADORB         1

#define MAX_SEQ             5           // Máximo número de secuencias almacenadas
#define MAX_PASOS           100         // Máximo pasos por secuencia
#define MAX_COMANDO_LEN     16
#define VEL_RIGHT           65
#define VEL_LEFT            65
#define ANGULO_CORRECCION   0.5f        // Escala de corrección de velocidad
#define MAX_DELTA_YAW       90.0f         // Límite para delta_yaw

/*==================[Variables globales]======================*/
static const char *TAG = "TCP_SERVER";

int client_sock = -1;

QueueHandle_t comando_queue;         // Cola para comandos para motores
QueueHandle_t posicion_queue;        // Cola para mensajes POS

SemaphoreHandle_t secuencia_mutex;   // Mutex para proteger acceso a secuencias

// Estructura para guardar secuencia de comandos
typedef struct {
    char comandos[MAX_PASOS][MAX_COMANDO_LEN];
    int pasos;
} Secuencia;

Secuencia secuencias[MAX_SEQ];
int num_secuencias = 0;

// Variables control de ejecución
int ruta_seleccionada = -1;          // índice de secuencia seleccionada
int paso_actual = 0;
//int esperando_posicion = 0;
char pos_actual[16] = "0,0";

char meta_actual[16] = "";
char secuencia_q_learning[MAX_PASOS][MAX_COMANDO_LEN];
int pasos_q_learning = 0;
int i = 0;

typedef enum{
    NORTE,
    SUR,
    ESTE,
    OESTE
}Direccion_matriz;

int fila_siguiente;
int col_siguiente;

Direccion_matriz direccion_actual_matriz = NORTE;

//int fila_actual = 0;
//int columna_actual = 0;

typedef enum{
    INICIO,
    GIRAR_180, 
    GIRAR_IZQUIERDA, 
    GIRAR_DERECHA, 
    AVANZAR
}Direccion;

Direccion direccion_actual = INICIO;

float delta_yaw_recibido = 0.0f;
float delta_yaw_real = 0.0f;
int comp_yaw = 0;
int vel_left, vel_right; 
float vel_base = 65.0; 
float multiplicador = 0.2;
int ajuste = 0; 
float yaw;
SemaphoreHandle_t yaw_mutex;

/*==================[Funciones]======================*/

void wifi_init_softap(void)
{
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32_AP",
            .ssid_len = strlen("ESP32_AP"),
            .channel = 1,
            .password = "12345678",
            .max_connection = 1,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen("12345678") == 0) wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "Access Point creado. SSID: ESP32_AP, PASS: 12345678");
}

void limpiar_secuencias() {
    secuencia_mutex = xSemaphoreCreateMutex();
    xSemaphoreTake(secuencia_mutex, portMAX_DELAY);
    num_secuencias = 0;
    for (int i=0; i<MAX_SEQ; i++) {
        secuencias[i].pasos = 0;
        for (int j=0; j<MAX_PASOS; j++) {
            secuencias[i].comandos[j][0] = 0;
        }
    }
    xSemaphoreGive(secuencia_mutex);
}

// Función para parsear el JSON simple con las rutas recibidas
// Ejemplo JSON: {"(4,4)":["UP","UP","RIGHT"],"(0,4)":["RIGHT","RIGHT","UP"]}
#include "cJSON.h"  // Asumo que tienes cJSON en el proyecto (si no, avísame)

int parsear_y_guardar_secuencias(const char* json_str)
{
    xSemaphoreTake(secuencia_mutex, portMAX_DELAY);

    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        ESP_LOGE(TAG, "Error parseando JSON");
        xSemaphoreGive(secuencia_mutex);
        return -1;
    }

    cJSON *accion = cJSON_GetObjectItem(root, "accion");
    if (!accion || !cJSON_IsNumber(accion) || accion->valueint != 1) {
        ESP_LOGE(TAG, "Acción no válida o diferente de 1");
        cJSON_Delete(root);
        xSemaphoreGive(secuencia_mutex);
        return -1;
    }

    // Obtener meta
    cJSON *meta = cJSON_GetObjectItem(root, "meta");
    if (meta && cJSON_IsArray(meta)) {
        snprintf(meta_actual, sizeof(meta_actual), "(%d,%d)", 
                 cJSON_GetArrayItem(meta, 0)->valueint, 
                 cJSON_GetArrayItem(meta, 1)->valueint);
    } else {
        ESP_LOGE(TAG, "Meta no encontrada o inválida");
    }

    // Obtener secuencia_q_learning
    cJSON *seq_q = cJSON_GetObjectItem(root, "secuencia_q_learning");
    pasos_q_learning = 0;
    if (seq_q && cJSON_IsArray(seq_q)) {
        for (int i = 0; i < cJSON_GetArraySize(seq_q) && i < MAX_PASOS; i++) {
            cJSON *item = cJSON_GetArrayItem(seq_q, i);
            if (cJSON_IsString(item)) {
                strncpy(secuencia_q_learning[i], item->valuestring, MAX_COMANDO_LEN-1);
                secuencia_q_learning[i][MAX_COMANDO_LEN-1] = '\0';
                pasos_q_learning++;
            }
        }
    }

    // Obtener secuencia_real
    /*
    cJSON *seq_real = cJSON_GetObjectItem(root, "secuencia_real");
    if (seq_real && cJSON_IsArray(seq_real)) {
        secuencias[num_secuencias].pasos = 0;
        for (int i = 0; i < cJSON_GetArraySize(seq_real) && i < MAX_PASOS; i++) {
            cJSON *item = cJSON_GetArrayItem(seq_real, i);
            if (cJSON_IsString(item)) {
                strncpy(secuencias[num_secuencias].comandos[i], item->valuestring, MAX_COMANDO_LEN-1);
                secuencias[num_secuencias].comandos[i][MAX_COMANDO_LEN-1] = '\0';
                secuencias[num_secuencias].pasos++;
            }
        }
        ESP_LOGI(TAG, "Secuencia real %d cargada con %d pasos", num_secuencias, secuencias[num_secuencias].pasos);
        num_secuencias++;
    }
    */

    cJSON *seq_real = cJSON_GetObjectItem(root, "secuencia_real");
    if (seq_real && cJSON_IsArray(seq_real)) {
        ESP_LOGI(TAG, "Parseando secuencia_real...");
        int pasos = 0;
        for (int i = 0; i < cJSON_GetArraySize(seq_real) && i < MAX_PASOS; i++) {
            cJSON *item = cJSON_GetArrayItem(seq_real, i);
            if (cJSON_IsString(item)) {
                ESP_LOGI(TAG, "Paso %d: %s", i, item->valuestring);
                strncpy(secuencias[num_secuencias].comandos[i], item->valuestring, MAX_COMANDO_LEN-1);
                secuencias[num_secuencias].comandos[i][MAX_COMANDO_LEN-1] = '\0';
                pasos++;
            } else {
                ESP_LOGW(TAG, "Elemento en secuencia_real no es string");
            }
        }
        if (pasos > 0) {
            secuencias[num_secuencias].pasos = pasos;
            ESP_LOGI(TAG, "Secuencia real %d cargada con %d pasos", num_secuencias, pasos);
            num_secuencias++;
        } else {
            ESP_LOGW(TAG, "Secuencia real vacía");
        }
    } else {
        ESP_LOGE(TAG, "secuencia_real no encontrada o no es array");
    }


    // Imprimir meta
    ESP_LOGI(TAG, "Meta recibida: %s", meta_actual);

    // Imprimir secuencia de Q-Learning
    ESP_LOGI(TAG, "Secuencia Q-Learning (%d pasos):", pasos_q_learning);
    for (int i = 0; i < pasos_q_learning; i++) {
        ESP_LOGI(TAG, "Q[%d]: %s", i, secuencia_q_learning[i]);
    }

    // Imprimir secuencia real
    if (num_secuencias > 0) {
        ESP_LOGI(TAG, "Secuencia real (%d pasos):", secuencias[num_secuencias-1].pasos);
        for (int i = 0; i < secuencias[num_secuencias-1].pasos; i++) {
            ESP_LOGI(TAG, "R[%d]: %s", i, secuencias[num_secuencias-1].comandos[i]);
        }
    }

    ruta_seleccionada = 0;
    paso_actual = 1;
    //esperando_posicion = 0;
    ESP_LOGI(TAG, "Todo OK");

    cJSON_Delete(root);
    xSemaphoreGive(secuencia_mutex);
    return 0;
}


// Función para enviar mensaje al cliente TCP
void enviar_respuesta(const char *msg)
{
    if(client_sock >= 0) {
        send(client_sock, msg, strlen(msg), 0);
    }
}

void ejecutar_giro_y_avanzar(Direccion nueva_dir_real) {
    //if (direccion_actual == nueva_dir) return;

    //switch(direccion_actual)
    switch(nueva_dir_real)
    {
        case GIRAR_DERECHA:
            cmdParser(MOTOR_LEFT_BACKWARD, VEL_LEFT);
            cmdParser(MOTOR_RIGHT_FORWARD, VEL_RIGHT);
            vTaskDelay(1100 / portTICK_PERIOD_MS);
            xSemaphoreTake(yaw_mutex, portMAX_DELAY);
            comp_yaw -= 90;
            //delta_yaw_real -= 90.0f;
            xSemaphoreGive(yaw_mutex);
            cmdParser(MOTOR_LEFT_STOP, 0);
            cmdParser(MOTOR_RIGHT_STOP, 0);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            break;
        case GIRAR_IZQUIERDA:
            cmdParser(MOTOR_LEFT_FORWARD, VEL_LEFT);
            cmdParser(MOTOR_RIGHT_BACKWARD, VEL_RIGHT);
            vTaskDelay(1100 / portTICK_PERIOD_MS);
            xSemaphoreTake(yaw_mutex, portMAX_DELAY);
            comp_yaw += 90;
            //delta_yaw_real += 90.0f;
            xSemaphoreGive(yaw_mutex);
            cmdParser(MOTOR_LEFT_STOP, 0);
            cmdParser(MOTOR_RIGHT_STOP, 0);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            break;
        case GIRAR_180:
            cmdParser(MOTOR_LEFT_BACKWARD, VEL_LEFT);
            cmdParser(MOTOR_RIGHT_FORWARD, VEL_RIGHT);
            vTaskDelay(2100 / portTICK_PERIOD_MS);
            break;
        case AVANZAR:
            //cmdParser(MOTOR_LEFT_FORWARD, 300);
            //cmdParser(MOTOR_RIGHT_FORWARD, 300);          

            fila_siguiente = pos_actual[0] - '0';
            col_siguiente = pos_actual[2] - '0';

            Direccion_matriz nueva_dir_matriz = direccion_actual_matriz;
            if(strcmp(secuencia_q_learning[i], "UP") == 0) nueva_dir_matriz = NORTE;
            else if(strcmp(secuencia_q_learning[i], "DOWN") == 0) nueva_dir_matriz = SUR;
            else if(strcmp(secuencia_q_learning[i], "LEFT") == 0) nueva_dir_matriz = ESTE;
            else if(strcmp(secuencia_q_learning[i], "RIGHT") == 0) nueva_dir_matriz = OESTE;

            /*
            if(nueva_dir_matriz == NORTE) fila_siguiente--;
            if(nueva_dir_matriz == SUR) fila_siguiente++;
            if(nueva_dir_matriz == ESTE) col_siguiente++;
            if(nueva_dir_matriz == OESTE) col_siguiente--;

            //ESP_LOGI(TAG, "Pos: %s", pos_actual[2]);

            while(fila_siguiente != (pos_actual[0] - '0') && col_siguiente != (pos_actual[2] - '0'))
            {
                ESP_LOGI(TAG, "Avanzando");
                cmdParser(MOTOR_LEFT_FORWARD, 300);
                cmdParser(MOTOR_RIGHT_FORWARD, 300);
                vTaskDelay(1000/portTICK_PERIOD_MS);
            }
            */

            switch(nueva_dir_matriz)
            {
                case NORTE:
                    fila_siguiente--;
                    while(fila_siguiente != (pos_actual[0] - '0')) {

                        xSemaphoreTake(yaw_mutex, portMAX_DELAY);
                        yaw = delta_yaw_real;
                        xSemaphoreGive(yaw_mutex);

                        // Ajuste de velocidades
                        if((yaw + comp_yaw) <= 35)
                        {
                            ajuste = (yaw + comp_yaw) * multiplicador;  // o alguna función proporcional
                        }
                        else if((yaw + comp_yaw) > 35 && (yaw + comp_yaw) <= 65)
                        {
                            ajuste = (yaw + comp_yaw) * (multiplicador + 0.15);
                        }
                        else
                        {
                            ajuste = (yaw + comp_yaw) * (multiplicador + 0.3);
                        }
                        //int ajuste = (yaw + comp_yaw) * multiplicador;  // o alguna función proporcional

                        vel_left = vel_base + ajuste;
                        vel_right = (vel_base + 4.0) - ajuste;  

                        cmdParser(MOTOR_LEFT_FORWARD, vel_left);
                        cmdParser(MOTOR_RIGHT_FORWARD, vel_right);
                        vTaskDelay(10/portTICK_PERIOD_MS);
                    }
                    break;
                case SUR:
                    fila_siguiente++;
                    while(fila_siguiente != (pos_actual[0] - '0')) {

                        xSemaphoreTake(yaw_mutex, portMAX_DELAY);
                        yaw = delta_yaw_real;
                        xSemaphoreGive(yaw_mutex);

                        // Ajuste de velocidades
                        if((yaw + comp_yaw) <= 35)
                        {
                            ajuste = (yaw + comp_yaw) * multiplicador;  // o alguna función proporcional
                        }
                        else if((yaw + comp_yaw) > 35 && (yaw + comp_yaw) <= 65)
                        {
                            ajuste = (yaw + comp_yaw) * (multiplicador + 0.15);
                        }
                        else
                        {
                            ajuste = (yaw + comp_yaw) * (multiplicador + 0.3);
                        }
                        //int ajuste = (yaw + comp_yaw) * multiplicador;  // o alguna función proporcional

                        vel_left = vel_base + ajuste;
                        vel_right = (vel_base + 4.0) - ajuste;  

                        cmdParser(MOTOR_LEFT_FORWARD, vel_left);
                        cmdParser(MOTOR_RIGHT_FORWARD, vel_right);
                        vTaskDelay(10/portTICK_PERIOD_MS);
                    }
                    break;
                case OESTE:
                    col_siguiente++;
                    while(col_siguiente != (pos_actual[2] - '0')) {

                        xSemaphoreTake(yaw_mutex, portMAX_DELAY);
                        yaw = delta_yaw_real;
                        xSemaphoreGive(yaw_mutex);

                        // Ajuste de velocidades
                        if((yaw + comp_yaw) <= 35)
                        {
                            ajuste = (yaw + comp_yaw) * multiplicador;  // o alguna función proporcional
                        }
                        else if((yaw + comp_yaw) > 35 && (yaw + comp_yaw) <= 65)
                        {
                            ajuste = (yaw + comp_yaw) * (multiplicador + 0.15);
                        }
                        else
                        {
                            ajuste = (yaw + comp_yaw) * (multiplicador + 0.3);
                        }
                        //int ajuste = (yaw + comp_yaw) * multiplicador;  // o alguna función proporcional

                        vel_left = vel_base + ajuste;
                        vel_right = (vel_base + 4.0) - ajuste;  

                        cmdParser(MOTOR_LEFT_FORWARD, vel_left);
                        cmdParser(MOTOR_RIGHT_FORWARD, vel_right);
                        vTaskDelay(10/portTICK_PERIOD_MS);
                    }
                    break;
                case ESTE:
                    col_siguiente--;
                    while(col_siguiente != (pos_actual[2] - '0')) {

                        xSemaphoreTake(yaw_mutex, portMAX_DELAY);
                        yaw = delta_yaw_real;
                        xSemaphoreGive(yaw_mutex);

                        // Ajuste de velocidades
                        if((yaw + comp_yaw) <= 35)
                        {
                            ajuste = (yaw + comp_yaw) * multiplicador;  // o alguna función proporcional
                        }
                        else if((yaw + comp_yaw) > 35 && (yaw + comp_yaw) <= 65)
                        {
                            ajuste = (yaw + comp_yaw) * (multiplicador + 0.15);
                        }
                        else
                        {
                            ajuste = (yaw + comp_yaw) * (multiplicador + 0.3);
                        }
                        //int ajuste = (yaw + comp_yaw) * multiplicador;  // o alguna función proporcional

                        vel_left = vel_base + ajuste;
                        vel_right = (vel_base + 4.0) - ajuste;  

                        cmdParser(MOTOR_LEFT_FORWARD, vel_left);
                        cmdParser(MOTOR_RIGHT_FORWARD, vel_right);
                        vTaskDelay(10/portTICK_PERIOD_MS);
                    }
                    break;
            }

            i++;
            direccion_actual_matriz = nueva_dir_matriz;
            break;
        case INICIO:
            // No hacer nada, es el estado inicial
            break;
    }
    cmdParser(MOTOR_LEFT_STOP, 0);
    cmdParser(MOTOR_RIGHT_STOP, 0);  // Detener motores después de girar
    //direccion_actual = nueva_dir;
}

void avanzar() {
    cmdParser(MOTOR_LEFT_FORWARD, VEL_LEFT);
    cmdParser(MOTOR_RIGHT_FORWARD, VEL_RIGHT);
}

void detener() {
    cmdParser(MOTOR_LEFT_STOP, 0);
    cmdParser(MOTOR_RIGHT_STOP, 0);
}

// Thread para manejar recepción TCP y comandos
void tcp_server_task(void *pvParameters)
{
    char rx_buffer[1024];
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    listen(listen_sock, 1);

    ESP_LOGI(TAG, "Esperando conexión TCP en puerto %d...", PORT);

    while (1)
    {
        struct sockaddr_in6 source_addr;
        uint addr_len = sizeof(source_addr);
        client_sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);

        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str)-1);
        ESP_LOGI(TAG, "Cliente conectado desde %s", addr_str);

        while (1)
        {
            int len = recv(client_sock, rx_buffer, sizeof(rx_buffer)-1, 0);
            if (len < 0)
            {
                ESP_LOGE(TAG, "Error en recv");
                break;
            }
            else if (len == 0)
            {
                ESP_LOGI(TAG, "Cliente desconectado");
                break;
            }
            else
            {
                rx_buffer[len] = 0;
                ESP_LOGI(TAG, "Mensaje recibido: %s", rx_buffer);

                // Procesar mensajes:

                // Mensaje de selección de ruta: "accion n"
                /*
                if(strncmp(rx_buffer, "accion ", 7) == 0) {
                    int sel = atoi(&rx_buffer[7]);
                    xSemaphoreTake(secuencia_mutex, portMAX_DELAY);
                    if(sel >= 0 && sel < num_secuencias){
                        ruta_seleccionada = sel;
                        paso_actual = 0;
                        esperando_posicion = 0;
                        enviar_respuesta("OK: Ruta seleccionada\n");
                        ESP_LOGI(TAG, "Ruta seleccionada: %d", sel);
                    }
                    else 
                    {
                        enviar_respuesta("ERROR: Ruta inválida\n");
                    }
                    xSemaphoreGive(secuencia_mutex);
                }
                */
                
                // Mensaje de confirmación de posición POS f,c
                if(strncmp(rx_buffer, "POS ", 4) == 0) {
                    // Guardar posición y liberar espera
                    strncpy(pos_actual, &rx_buffer[4], sizeof(pos_actual)-1);
                    pos_actual[sizeof(pos_actual)-1] = 0;
                    //esperando_posicion = 0;  // Liberar espera para avanzar paso
                    //ESP_LOGI(TAG, "Posición recibida: %s", pos_actual);
                    //ESP_LOGI(TAG, "Posición recibida: %d", pos_actual[0] - '0');
                }
                // Supongamos que todo lo demás es JSON con las rutas
                else if(strncmp(rx_buffer, "ANG ", 4) == 0) 
                {
                    float nuevo_yaw = atof(&rx_buffer[4]);

                    xSemaphoreTake(yaw_mutex, portMAX_DELAY);
                    delta_yaw_recibido = nuevo_yaw;
                    delta_yaw_real = nuevo_yaw;  // Lo copiamos solo si estamos en AVANZAR
                    xSemaphoreGive(yaw_mutex);

                    //ESP_LOGI(TAG, "Ángulo recibido: %.2f", nuevo_yaw);
                    //ESP_LOGI(TAG, "Posición recibida: %s", pos_actual);
                    //ESP_LOGI(TAG, "ruta_seleccionada = %d", ruta_seleccionada);
                    //ESP_LOGI(TAG, "paso_actual = %d", paso_actual);
                }
                else 
                {
                    int res = parsear_y_guardar_secuencias(rx_buffer);
                    if (res == 0) {
                        enviar_respuesta("OK: Secuencias cargadas\n");
                    } else {
                        enviar_respuesta("ERROR: JSON inválido\n");
                    }
                }
            }
        }
        close(client_sock);
    }
}

// Tarea para ejecutar motores paso a paso según secuencia y posición
void motores_task(void *pvParameters) {
    while(1)
    {
        if(ruta_seleccionada >= 0)
        {
            xSemaphoreTake(secuencia_mutex, portMAX_DELAY);
            Secuencia *seq = &secuencias[ruta_seleccionada];
            ESP_LOGI(TAG, "Pasos: %d", seq->pasos);
            // Retardo solo para el comienzo
            if(paso_actual == 1)
            {
                vTaskDelay(5000/portTICK_PERIOD_MS);
            }
            
            if(paso_actual < seq->pasos)
            {
                char *cmd = seq->comandos[paso_actual];
                ESP_LOGI(TAG, "Ejecutando paso %d: %s", paso_actual, cmd);

                //direccion_actual = cmd;
                Direccion nueva_dir_real = direccion_actual;
                if(strcmp(cmd, "GIRAR_180") == 0) nueva_dir_real = GIRAR_180;
                else if(strcmp(cmd, "GIRAR_DERECHA") == 0) nueva_dir_real = GIRAR_DERECHA;
                else if(strcmp(cmd, "GIRAR_IZQUIERDA") == 0) nueva_dir_real = GIRAR_IZQUIERDA;
                else if(strcmp(cmd, "AVANZAR") == 0) nueva_dir_real = AVANZAR;

                ejecutar_giro_y_avanzar(nueva_dir_real);

                //avanzar();
                
                direccion_actual = nueva_dir_real;
                paso_actual++;
            }
            else
            {
                ESP_LOGI(TAG, "Secuencia finalizada");

                // Reseteo de variables
                num_secuencias = 0;
                pasos_q_learning = 0;
                //pos_actual[16] = "0,0";
                pos_actual[0] = 0;
                pos_actual[2] = 0;  
                ESP_LOGI(TAG, "pos_actual: %s", pos_actual);
                //paso_actual = 0;
                ruta_seleccionada = -1;
                i = 0;

                direccion_actual = INICIO;
                direccion_actual_matriz = NORTE;
            }
            xSemaphoreGive(secuencia_mutex);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/*==================[Main]======================*/
void app_main(void)
{
    motorsSetup();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_softap();

    comando_queue = xQueueCreate(5, sizeof(char) * 128);
    posicion_queue = xQueueCreate(5, sizeof(char) * 64);

    limpiar_secuencias();

    secuencia_mutex = xSemaphoreCreateMutex();
    yaw_mutex = xSemaphoreCreateMutex();

    BaseType_t errA = xTaskCreatePinnedToCore(
        tcp_server_task,
        "tcp_server",
        configMINIMAL_STACK_SIZE * 8,
        NULL,
        tskIDLE_PRIORITY + 3,
        NULL,
        PROCESADORA
    );
    if (errA == pdFAIL)
    {
        ESP_LOGE(TAG, "Error creando tcp_server_task");
        while(1);
    }

    BaseType_t errB = xTaskCreatePinnedToCore(
        motores_task,
        "motores_task",
        configMINIMAL_STACK_SIZE * 8,
        NULL,
        tskIDLE_PRIORITY + 5,
        NULL,
        PROCESADORB
    );
    if (errB == pdFAIL)
    {
        ESP_LOGE(TAG, "Error creando motores_task");
        while(1);
    }
}
