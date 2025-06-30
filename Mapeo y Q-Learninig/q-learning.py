import socket
import json
import numpy as np
import random
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque
import time
import os
import keyboard

# ---------- MAIN ----------
"""
def conectar_esp32(ip, puerto):
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((ip, puerto))
            print("[CLIENTE] Conectado al ESP32.")
            return s
        except Exception:
            print("[CLIENTE] ESP32 ocupado o no disponible. Esperando para reconectar...")
            time.sleep(2)

def main():
    TCP_IP = "192.168.4.1"
    TCP_PORT = 3333

    while True:
        print("[CLIENTE] Presioná 'a' para conectar al ESP32...")
        keyboard.wait('a')

        # Intentar conectar al ESP32
        s = conectar_esp32(TCP_IP, TCP_PORT)

        print("[CLIENTE] Esperando tecla 'e' para iniciar el entrenamiento...")
        keyboard.wait('e')

        try:
            # Leer y ejecutar el archivo datos.txt
            print("[CLIENTE] Leyendo archivo 'datos.txt'...")
            with open("datos.txt", "r") as f:
                data = f.read()
            local = {}
            exec(data, {}, local)
            config = local["config"]

            # Preparar entorno
            GRID_SIZE = tuple(config["GRID_SIZE"])
            OBSTACLES = [tuple(ob) for ob in config["OBSTACLES"]]
            START = tuple(config["START"])
            GOAL = tuple(config["GOAL"])
            ACTIONS = config["ACTIONS"]

            env = GridWorldEnv(GRID_SIZE, START, GOAL, OBSTACLES)
            state_size = GRID_SIZE[0] * GRID_SIZE[1]
            action_size = len(ACTIONS)

            model = train(env, state_size, action_size)
            action_sequence = run_simulation(model, env, GRID_SIZE, ACTIONS)

            payload = {
                "accion": "SECUENCIA",
                "meta": GOAL,
                "secuencia": action_sequence
            }

            print("[CLIENTE] Enviando secuencia al ESP32...")
            s.send(json.dumps(payload).encode())
            print("[CLIENTE] Secuencia enviada correctamente.")
        except Exception as e:
            print(f"[CLIENTE] Error durante el proceso: {e}")
        finally:
            s.close()
            print("[CLIENTE] Conexión cerrada. Volvé a presionar 'a' para reiniciar el ciclo.")
"""

def main():
    TCP_IP = "192.168.4.1"  # IP del ESP32
    TCP_PORT = 3333         # Puerto del ESP32

    print("[CLIENTE] Conectando al ESP32...")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((TCP_IP, TCP_PORT))
            print("[CLIENTE] Conexión establecida con el ESP32.")

            print("[CLIENTE] Esperando tecla 'e' para comenzar el entrenamiento...")
            keyboard.wait('e')

            print("[CLIENTE] Leyendo archivo de configuración 'datos.txt'...")
            with open("config.txt", "r") as f:
                data = f.read()
            local = {}
            exec(data, {}, local)
            config = local["config"]

            GRID_SIZE = tuple(config["GRID_SIZE"])
            OBSTACLES = [tuple(ob) for ob in config["OBSTACLES"]]
            START = tuple(config["START"])
            GOAL = tuple(config["GOAL"])
            ACTIONS = config["ACTIONS"]

            env = GridWorldEnv(GRID_SIZE, START, GOAL, OBSTACLES)
            state_size = GRID_SIZE[0] * GRID_SIZE[1]
            action_size = len(ACTIONS)

            model = train(env, state_size, action_size)
            (action_sequence, comandos_robot) = run_simulation(model, env, GRID_SIZE, ACTIONS)

            payload = {
                "accion": 1,
                "meta": GOAL,
                "secuencia_q_learning": action_sequence,
                "secuencia_real": comandos_robot
            }

            print("[CLIENTE] Enviando secuencia al ESP32...")
            s.send(json.dumps(payload).encode())
            print("[CLIENTE] Secuencia enviada correctamente.")
    except Exception as e:
        print(f"[CLIENTE] Error en la conexión con el ESP32: {e}")


# ---------- CLASES Y FUNCIONES ----------
class GridWorldEnv:
    def __init__(self, grid_size, start, goal, obstacles):
        self.grid_size = grid_size
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.reset()

    def reset(self):
        self.agent_pos = list(self.start)
        return self._get_state()

    def _get_state(self):
        return self.agent_pos[0] * self.grid_size[1] + self.agent_pos[1]

    def step(self, action):
        old_pos = self.agent_pos.copy()
        if action == 0:
            self.agent_pos[0] = max(self.agent_pos[0] - 1, 0)
        elif action == 1:
            self.agent_pos[0] = min(self.agent_pos[0] + 1, self.grid_size[0] - 1)
        elif action == 2:
            self.agent_pos[1] = max(self.agent_pos[1] - 1, 0)
        elif action == 3:
            self.agent_pos[1] = min(self.agent_pos[1] + 1, self.grid_size[1] - 1)

        if tuple(self.agent_pos) in self.obstacles:
            self.agent_pos = old_pos
            reward = -1
            done = False
        elif tuple(self.agent_pos) == self.goal:
            reward = 10
            done = True
        else:
            reward = -0.1
            done = False

        return self._get_state(), reward, done


class DQN(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(DQN, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(),
            nn.Linear(128, output_dim)
        )

    def forward(self, x):
        return self.fc(x)
    
def convertir_a_comandos_robot(secuencia_absoluta):
    comandos = []
    orientacion = 'UP'  # Asumimos que el robot inicia mirando hacia arriba

    # Mapeo de rotaciones necesarias desde orientacion actual hacia acción deseada
    rotaciones = {
        'UP':    {'UP': [], 'DOWN': ['GIRAR_180'], 'LEFT': ['GIRAR_IZQUIERDA'], 'RIGHT': ['GIRAR_DERECHA']},
        'DOWN':  {'UP': ['GIRAR_180'], 'DOWN': [], 'LEFT': ['GIRAR_DERECHA'], 'RIGHT': ['GIRAR_IZQUIERDA']},
        'LEFT':  {'UP': ['GIRAR_DERECHA'], 'DOWN': ['GIRAR_IZQUIERDA'], 'LEFT': [], 'RIGHT': ['GIRAR_180']},
        'RIGHT': {'UP': ['GIRAR_IZQUIERDA'], 'DOWN': ['GIRAR_DERECHA'], 'LEFT': ['GIRAR_180'], 'RIGHT': []}
    }

    for accion in secuencia_absoluta:
        # Obtener las rotaciones necesarias
        pasos_rotacion = rotaciones[orientacion][accion]
        comandos.extend(pasos_rotacion)
        comandos.append('AVANZAR')  # Siempre avanza tras girar

        # Actualizar orientación del robot
        orientacion = accion  # Ahora el robot "mira" hacia la nueva dirección

    return comandos


def train(env, state_size, action_size):
    model = DQN(state_size, action_size)
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    criterion = nn.MSELoss()
    memory = deque(maxlen=2000)

    episodes = 500
    gamma = 0.95
    epsilon = 1.0
    epsilon_min = 0.01
    epsilon_decay = 0.995
    batch_size = 64

    for e in range(episodes):
        state = env.reset()
        total_reward = 0
        for _ in range(100):
            state_tensor = torch.zeros(state_size)
            state_tensor[state] = 1.0
            if random.random() < epsilon:
                action = random.randint(0, action_size - 1)
            else:
                with torch.no_grad():
                    q_values = model(state_tensor)
                    action = torch.argmax(q_values).item()

            next_state, reward, done = env.step(action)
            next_state_tensor = torch.zeros(state_size)
            next_state_tensor[next_state] = 1.0

            memory.append((state_tensor, action, reward, next_state_tensor, done))
            state = next_state
            total_reward += reward

            if done:
                break

        if len(memory) >= batch_size:
            minibatch = random.sample(memory, batch_size)
            for s, a, r, s_next, d in minibatch:
                target = r
                if not d:
                    with torch.no_grad():
                        target += gamma * torch.max(model(s_next)).item()
                target_f = model(s).clone()
                target_f[a] = target
                optimizer.zero_grad()
                loss = criterion(model(s), target_f)
                loss.backward()
                optimizer.step()

        if epsilon > epsilon_min:
            epsilon *= epsilon_decay

    return model


def render_env(env, agent_pos):
    for i in range(env.grid_size[0]):
        row = ""
        for j in range(env.grid_size[1]):
            pos = (i, j)
            if pos in env.obstacles:
                row += " X "
            elif pos == env.goal:
                row += " G "
            elif pos == tuple(agent_pos):
                row += " R "
            else:
                row += " . "
        print(row)
    print("\n")
    time.sleep(0.2)


def run_simulation(model, env, GRID_SIZE, ACTIONS):
    state = env.reset()
    total_reward = 0
    action_sequence = []

    for _ in range(50):
        state_tensor = torch.zeros(GRID_SIZE[0] * GRID_SIZE[1])
        state_tensor[state] = 1.0

        with torch.no_grad():
            q_values = model(state_tensor)
            action = torch.argmax(q_values).item()

        action_sequence.append(ACTIONS[action])
        render_env(env, env.agent_pos)
        next_state, reward, done = env.step(action)
        state = next_state
        total_reward += reward

        if done:
            render_env(env, env.agent_pos)
            print("\n[SIMULACIÓN] Meta alcanzada!")
            break
    else:
        print("[SIMULACIÓN] No llegó a la meta.")

    print(f"\nRecompensa total: {total_reward:.2f}")
    print("Secuencia de movimientos:")
    print(action_sequence)

    comandos_robot = convertir_a_comandos_robot(action_sequence)
    print(comandos_robot)

    return (action_sequence, comandos_robot)

if __name__ == "__main__":
    main()

"""

# ---------- ENTORNO GRIDWORLD ----------
class GridWorldEnv:
    def __init__(self, grid_size, start, goals, obstacles):
        self.grid_size = grid_size
        self.start = start
        self.goals = goals  # Lista de metas
        self.obstacles = obstacles
        self.reset()

    def reset(self, goal=None):
        self.agent_pos = list(self.start)
        self.goal = goal if goal else random.choice(self.goals)
        return self._get_state()

    def _get_state(self):
        state = torch.zeros(self.grid_size[0] * self.grid_size[1] * 2)
        agent_index = self.agent_pos[0] * self.grid_size[1] + self.agent_pos[1]
        goal_index = self.goal[0] * self.grid_size[1] + self.goal[1]
        state[agent_index] = 1.0
        state[self.grid_size[0] * self.grid_size[1] + goal_index] = 1.0
        return state

    def step(self, action):
        old_pos = self.agent_pos.copy()
        if action == 0:  # UP
            self.agent_pos[0] = max(self.agent_pos[0] - 1, 0)
        elif action == 1:  # DOWN
            self.agent_pos[0] = min(self.agent_pos[0] + 1, self.grid_size[0] - 1)
        elif action == 2:  # LEFT
            self.agent_pos[1] = max(self.agent_pos[1] - 1, 0)
        elif action == 3:  # RIGHT
            self.agent_pos[1] = min(self.agent_pos[1] + 1, self.grid_size[1] - 1)

        if tuple(self.agent_pos) in self.obstacles:
            self.agent_pos = old_pos
            reward = -1
            done = False
        elif tuple(self.agent_pos) == self.goal:
            reward = 10
            done = True
        else:
            reward = -0.1
            done = False

        return self._get_state(), reward, done

# ---------- MODELO DE RED NEURONAL ----------
class DQN(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(DQN, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(),
            nn.Linear(128, output_dim)
        )

    def forward(self, x):
        return self.fc(x)

# ---------- ENTRENAMIENTO ----------
def train(env, state_size, action_size):
    model = DQN(state_size, action_size)
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    criterion = nn.MSELoss()
    memory = deque(maxlen=2000)

    episodes = 200
    gamma = 0.95
    epsilon = 1.0
    epsilon_min = 0.01
    epsilon_decay = 0.995
    batch_size = 64

    for e in range(episodes):
        goal = random.choice(env.goals)
        state = env.reset(goal=goal)
        total_reward = 0

        for _ in range(100):
            state_tensor = state
            if random.random() < epsilon:
                action = random.randint(0, action_size - 1)
            else:
                with torch.no_grad():
                    q_values = model(state_tensor)
                    action = torch.argmax(q_values).item()

            next_state, reward, done = env.step(action)
            memory.append((state_tensor, action, reward, next_state, done))
            state = next_state
            total_reward += reward

            if done:
                break

        if len(memory) >= batch_size:
            minibatch = random.sample(memory, batch_size)
            for s, a, r, s_next, d in minibatch:
                target = r
                if not d:
                    with torch.no_grad():
                        target += gamma * torch.max(model(s_next)).item()
                target_f = model(s).clone()
                target_f[a] = target
                optimizer.zero_grad()
                loss = criterion(model(s), target_f)
                loss.backward()
                optimizer.step()

        if epsilon > epsilon_min:
            epsilon *= epsilon_decay

        if (e + 1) % 100 == 0:
            print(f"[Entrenamiento] Episodio {e + 1}, Recompensa total: {total_reward:.2f}, Epsilon: {epsilon:.3f}")

    return model

# ---------- SIMULACIÓN ----------
def run_simulation(model, env, actions_str):
    goal_paths = {}
    for goal in env.goals:
        state = env.reset(goal=goal)
        path = []
        total_reward = 0
        for _ in range(50):
            with torch.no_grad():
                q_values = model(state)
                action = torch.argmax(q_values).item()

            path.append(actions_str[action])
            next_state, reward, done = env.step(action)
            state = next_state
            total_reward += reward
            if done:
                break
        goal_paths[str(goal)] = {
            "path": path,
            "reward": total_reward
        }
    return goal_paths

# ---------- SOCKET CLIENTE ----------
def main():
    HOST = '192.168.4.1'
    PORT = 3333
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((HOST, PORT))

    try:
        while True:
            datos = client_socket.recv(4096)
            if not datos:
                print("[CLIENTE] Servidor desconectado.")
                break

            config = json.loads(datos.decode())
            GRID_SIZE = tuple(config["GRID_SIZE"])
            OBSTACLES = [tuple(ob) for ob in config["OBSTACLES"]]
            START = tuple(config["START"])
            GOALS = [tuple(g) for g in config["GOALS"]]
            ACTIONS = config["ACTIONS"]

            env = GridWorldEnv(GRID_SIZE, START, GOALS, OBSTACLES)
            state_size = GRID_SIZE[0] * GRID_SIZE[1] * 2
            action_size = len(ACTIONS)

            model = train(env, state_size, action_size)
            resultados = run_simulation(model, env, ACTIONS)

            for goal, info in resultados.items():
                print(f"Meta {goal}:")
                print(f"  Secuencia: {info['path']}")
                print(f"  Recompensa total: {info['reward']:.2f}\n")

            # Mostrar la grilla
            print("\n--- Estado del entorno ---")
            for i in range(GRID_SIZE[0]):
                fila = ""
                for j in range(GRID_SIZE[1]):
                    pos = (i, j)
                    if pos == START:
                        fila += " R "
                    elif pos in OBSTACLES:
                        fila += " x "
                    elif pos in GOALS:
                        fila += " G "
                    else:
                        fila += " . "
                print(fila)

            # Enviar los resultados como JSON al servidor
            client_socket.send(json.dumps(resultados).encode())

    except KeyboardInterrupt:
        print("[CLIENTE] Finalizado por el usuario.")
    finally:
        client_socket.close()

# ---------- EJECUCIÓN ----------
if __name__ == "__main__":
    main()

"""