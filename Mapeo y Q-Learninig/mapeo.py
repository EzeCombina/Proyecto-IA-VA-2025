import cv2 as cv
import numpy as np
import time
import datetime
import socket

# === CONFIGURACIÓN DE TCP ===
TCP_IP = "192.168.4.1"   # IP del ESP32 en modo AP
TCP_PORT = 3333

cap = cv.VideoCapture(1)  # O la URL de la cámara IP

def esperar_conexion_esp32():
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((TCP_IP, TCP_PORT))
            print("[MAPEO] Conectado al ESP32.")
            return s
        except (ConnectionRefusedError, OSError):
            print("[MAPEO] ESP32 ocupado. Reintentando en 1 segundos...")
            time.sleep(1)

# Al comienzo del script de mapeo:
sock = esperar_conexion_esp32()

# === Configuración ArUco y cámara ===
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
parameters = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, parameters)

aruco_positions = {
    25: ('TL', 2),
    33: ('TR', 3),
    30: ('BR', 0),
    23: ('BL', 1)
}
mobile_id = 99

grid_width = 400
grid_height = 400
step = 100

dst_pts = np.array([
    [0, 0],
    [grid_width, 0],
    [grid_width, grid_height],
    [0, grid_height]
], dtype=np.float32)

GRID_COLOR = (255, 0, 0)
MARKER_COLOR = (0, 255, 255)

#cap = cv.VideoCapture(1)  # O la URL de la cámara IP

last_sent_cell = None
last_sent_time = 0
send_interval = 1.0  # segundos

def guardar_snapshot_con_grilla(frame_con_grilla, src_pts):
    """
    Recorta y guarda la zona delimitada por los ArUcos (con la grilla ya dibujada encima).
    """
    global dst_pts

    src_pts = np.array(src_pts, dtype=np.float32)

    # Matriz de homografía para recorte corregido
    h_crop = cv.getPerspectiveTransform(src_pts, dst_pts)

    # Aplicamos la transformación al frame con grilla ya superpuesta
    snapshot = cv.warpPerspective(frame_con_grilla, h_crop, (grid_width, grid_height))

    # Generamos un nombre con timestamp
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"snapshot_{timestamp}.png"
    cv.imwrite(filename, snapshot)
    print(f"[INFO] Imagen guardada como {filename}")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None:
        ids = ids.flatten()
        detected = {int(id): corner for id, corner in zip(ids, corners)}

        if all(id in detected for id in aruco_positions):
            src_pts = []
            for id in [25, 33, 30, 23]:
                _, idx = aruco_positions[id]
                pt = detected[id][0][idx]
                src_pts.append(pt)
            src_pts = np.array(src_pts, dtype=np.float32)

            h_matrix = cv.getPerspectiveTransform(dst_pts, src_pts)
            inv_h_matrix = cv.getPerspectiveTransform(src_pts, dst_pts)

            grid_img = np.zeros((grid_height, grid_width, 4), dtype=np.uint8)
            line_color = (255, 0, 0, 255)

            for x in range(0, grid_width, step):
                cv.line(grid_img, (x, 0), (x, grid_height), line_color, 1)
            for y in range(0, grid_height, step):
                cv.line(grid_img, (0, y), (grid_width, y), line_color, 1)

            warped = cv.warpPerspective(grid_img, h_matrix, (frame.shape[1], frame.shape[0]), flags=cv.INTER_LINEAR, borderMode=cv.BORDER_TRANSPARENT)
            warped_bgr = warped[..., :3]
            alpha_mask = warped[..., 3]
            alpha_mask_3 = cv.merge([alpha_mask]*3)
            alpha = alpha_mask_3.astype(float) / 255.0
            frame_float = frame.astype(float)
            warped_float = warped_bgr.astype(float)
            frame = cv.convertScaleAbs(warped_float * alpha + frame_float * (1 - alpha))
            frame_con_grilla = frame.copy()

            if mobile_id in detected:
                mobile_corners = detected[mobile_id][0]
                pts_int = mobile_corners.astype(int)
                cv.polylines(frame, [pts_int], isClosed=True, color=MARKER_COLOR, thickness=2)

                mobile_corners = mobile_corners.reshape(-1, 1, 2)
                transformed_corners = cv.perspectiveTransform(mobile_corners, inv_h_matrix)
                xs = transformed_corners[:, 0, 0]
                ys = transformed_corners[:, 0, 1]

                cols = xs // step
                rows = ys // step

                if np.all(cols == cols[0]) and np.all(rows == rows[0]):
                    col = int(cols[0])
                    row = int(rows[0])

                    if 0 <= col < (grid_width // step) and 0 <= row < (grid_height // step):
                        x_center = np.mean(xs)
                        y_center = np.mean(ys)

                        cell_x_min = col * step
                        cell_x_max = (col + 1) * step
                        cell_y_min = row * step
                        cell_y_max = (row + 1) * step

                        margin_ratio = 0.2
                        margin_x = step * margin_ratio
                        margin_y = step * margin_ratio

                        central_x_min = cell_x_min + margin_x
                        central_x_max = cell_x_max - margin_x
                        central_y_min = cell_y_min + margin_y
                        central_y_max = cell_y_max - margin_y

                        if central_x_min <= x_center <= central_x_max and central_y_min <= y_center <= central_y_max:
                            celda = f"Fila {row}, Columna {col}"
                            cv.putText(frame, celda, (50, 80), cv.FONT_HERSHEY_SIMPLEX, 0.8, MARKER_COLOR, 2)

                            current_time = time.time()
                            if (last_sent_cell != (row, col)) or (current_time - last_sent_time > send_interval):
                                mensaje = f"POS {row},{col}\n"
                                print(f"[TCP] Enviando: {mensaje.strip()}")
                                try:
                                    if sock:
                                        sock.send(mensaje.encode())
                                except Exception as e:
                                    print(f"[TCP] Error al enviar: {e}")
                                last_sent_cell = (row, col)
                                last_sent_time = current_time

    cv.imshow("Tracking ArUco", frame)
    key = cv.waitKey(1) & 0xFF
    if key == 27:  # ESC
        break
    elif key == ord('s'):
        if 'src_pts' in locals() and 'frame_con_grilla' in locals():
            guardar_snapshot_con_grilla(frame_con_grilla, src_pts)
        else:
            print("[ADVERTENCIA] No se puede guardar la imagen. Verificá que los ArUcos estén detectados.")

cap.release()
cv.destroyAllWindows()
if sock:
    sock.close()

