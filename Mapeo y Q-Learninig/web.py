from flask import Flask, render_template, request, jsonify
import json
import os

app = Flask(__name__)

DATA_FILE = 'puntos.json' # Archivo para guardar los puntos del agente
CONFIG_FILE = 'config.txt' # Archivo de configuración para el agente

# Cargar puntos guardados si existen
if os.path.exists(DATA_FILE):
    with open(DATA_FILE, 'r') as f:
        puntos_guardados = json.load(f)
else:
    puntos_guardados = {}

@app.route('/')
def index():
    return render_template('index1.html')

@app.route('/datos')
def obtener_datos():
    return jsonify(puntos_guardados)

@app.route('/guardar_puntos', methods=['POST'])
def guardar_puntos():
    global puntos_guardados
    puntos_guardados = request.json
    with open(DATA_FILE, 'w') as f:
        json.dump(puntos_guardados, f)
    return '', 204

@app.route('/enviar_config', methods=['POST'])
def enviar_config():
    try:
        global puntos_guardados
        puntos = puntos_guardados.copy()

        grid_size_dict = puntos.get("grid_config", {"filas": 15, "columnas": 20})
        grid_size = [grid_size_dict["filas"], grid_size_dict["columnas"]]

        # Obtener puntos finales
        #finales = [k for k in puntos if k.startswith("F")]
        #goals = [(puntos[k]["fila"], puntos[k]["columna"]) for k in sorted(finales)]

        finales = [k for k in puntos if k.startswith("F")]
        if finales:
            # Tomar el primero en orden
            k = sorted(finales)[0]
            goal = (puntos[k]["fila"], puntos[k]["columna"])
        else:
            goal = (0, 0)

        # Crear configuración
        config = {
            "GRID_SIZE": grid_size,
            "OBSTACLES": [(o["fila"], o["columna"]) for o in puntos.get("obstaculos", [])],
            "START": (puntos["I"]["fila"], puntos["I"]["columna"]) if "I" in puntos else (0, 0),
            "GOAL": goal,
            "ACTIONS": ["UP", "DOWN", "LEFT", "RIGHT"]
        }

        # Guardar como archivo de configuración
        with open(CONFIG_FILE, 'w') as f:
            f.write("config = {\n")
            for clave, valor in config.items():
                f.write(f'    "{clave}": {valor},\n' if clave != "ACTIONS" else f'    "{clave}": {valor}\n')
            f.write("}")

        return jsonify({"status": "ok", "mensaje": "Configuración guardada en config.txt"}), 200

    except Exception as e:
        return jsonify({"status": "error", "mensaje": str(e)}), 500


    except Exception as e:
        return jsonify({"status": "error", "mensaje": str(e)}), 500

if __name__ == '__main__':
    app.run(debug=True)

