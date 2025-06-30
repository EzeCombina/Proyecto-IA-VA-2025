# === Lanzador modificado ===
# lanzar_todo.py
import subprocess
import time

# Lanzar servidor
#servidor = subprocess.Popen(["python", "servidor_q-learning.py"])
#time.sleep(1)

# Lanzar Pag Web
web = subprocess.Popen(["python", "web.py"])
time.sleep(1)

# Lanzar cliente Q-Learning
q_learning = subprocess.Popen(["python", "q-learning.py"])
time.sleep(5)
#q_learning.wait()

# Lanzar ArUco tracking (en paralelo)
#aruco = subprocess.Popen(["python", "mapeo.py"])
aruco = subprocess.Popen(["python", "mapeo_guia.py"])
#aruco = subprocess.Popen(["python", "Server_V2.py"])
time.sleep(1)

# Esperar finalización de Q-Learning
aruco.wait()
#print("[LAUNCHER] ArUco terminado.")

# Esperar cierre del servidor y ArUco
#servidor.wait()
aruco.terminate()
print("[LAUNCHER] q_learning Finalizado.")
