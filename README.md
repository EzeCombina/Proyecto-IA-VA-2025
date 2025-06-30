# Proyecto-IA-VA

## **Mapeo y Q-Learning**

### **lanzador.py**     
### ↳ Programa con el que se ejecutan todos los .py a la vez. 

### **mapeo.py**        
### ↳ Programa con el que se hace el mapeo del dominio, creación de la grilla y posterior envio de la posición del robot al ESP32.

### **mapeo_guia.py**   
### ↳ Programa con el que se hace el mapeo del dominio, creación de la grilla y posterior envio de la posición del robot al ESP32. En este se incluyen los vectores para el cálculo del sistema de control del movimiento del robot. 

### **q-learning**      
### ↳ Programa que lee los datos del tamaño de la grilla, el punto de inicio, el punto final y los obáculos guardados en el .txt y realiza el aprendizaje para obtener el camino mas óptimo. 

### **web.py**          
### ↳ script que genera el servidor web en donde se ingresan los datos necesarios. 

## **Robot** 

### → Recibe la secuencia generada por el q-learning. 
### → Realiza los movimientos.
### → Compara la posición del robot con la de el siguiente movimiento. 

## **Robot_Guia**

### → Recibe la secuencia generada por el q-learning. 
### → Realiza los movimientos.
### → Compara la posición del robot con la de el siguiente movimiento. 
### → Modifica el valor de las velocidades de los motores según la diferencia entre el valor del vector dirección del robot y el vector del ArUco fijo. 