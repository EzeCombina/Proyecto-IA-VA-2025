import cv2 as cv
import numpy as np
import time
import datetime
import socket

# === CONFIGURACIÓN DE TCP ===
TCP_IP = "192.168.4.1"
TCP_PORT = 3333

cap = cv.VideoCapture(1)

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

sock = esperar_conexion_esp32()

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
#states = 4  # IR VARIANDO   
step = 100

dst_pts = np.array([
    [0, 0],
    [grid_width, 0],
    [grid_width, grid_height],
    [0, grid_height]
], dtype=np.float32)

camera_matrix = np.array([
    [800, 0, 320],
    [0, 800, 240],
    [0, 0, 1]
], dtype=np.float32)
dist_coeffs = np.zeros((5, 1))

marker_length = 0.05

GRID_COLOR = (255, 0, 0)
MARKER_COLOR = (0, 255, 255)

last_sent_cell = None
last_sent_time = 0
send_interval = 1.0
last_sent_angle = None
angle_send_threshold = 2.0

def guardar_snapshot_con_grilla(frame_con_grilla, src_pts):
    src_pts = np.array(src_pts, dtype=np.float32)
    h_crop = cv.getPerspectiveTransform(src_pts, dst_pts)
    snapshot = cv.warpPerspective(frame_con_grilla, h_crop, (grid_width, grid_height))
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"snapshot_{timestamp}.png"
    cv.imwrite(filename, snapshot)
    print(f"[INFO] Imagen guardada como {filename}")

def estimate_pose_from_corners(corners, marker_length):
    obj_points = np.array([
        [-marker_length/2,  marker_length/2, 0],
        [ marker_length/2,  marker_length/2, 0],
        [ marker_length/2, -marker_length/2, 0],
        [-marker_length/2, -marker_length/2, 0]
    ], dtype=np.float32)

    img_points = corners.reshape(4, 2).astype(np.float32)
    success, rvec, tvec = cv.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)
    if not success:
        raise ValueError("solvePnP falló")
    return rvec, tvec

while True:
    ret, frame = cap.read()
    if not ret:
        break

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None:
        ids = ids.flatten()
        detected = {int(id): corner for id, corner in zip(ids, corners)}

        if all(id in detected for id in aruco_positions):
            src_pts = [detected[id][0][aruco_positions[id][1]] for id in [25, 33, 30, 23]]
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
            alpha = warped[..., 3].astype(float) / 255.0
            frame_float = frame.astype(float)
            warped_float = warped[..., :3].astype(float)
            frame = cv.convertScaleAbs(warped_float * alpha[..., None] + frame_float * (1 - alpha[..., None]))
            frame_con_grilla = frame.copy()

            # === POSICION ===
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

                if np.std(cols) < 0.2 and np.std(rows) < 0.2:
                    col = int(cols[0])
                    row = int(rows[0])

                    if 0 <= col < (grid_width // step) and 0 <= row < (grid_height // step):
                        x_center = np.mean(xs)
                        y_center = np.mean(ys)

                        margin_x = step * 0.05 # 0.2
                        margin_y = step * 0.05 # 0.2

                        cxmin = col * step + margin_x
                        cxmax = (col + 1) * step - margin_x
                        cymin = row * step + margin_y
                        cymax = (row + 1) * step - margin_y

                        if cxmin <= x_center <= cxmax and cymin <= y_center <= cymax:
                            celda = f"Fila {row}, Columna {col}"
                            cv.putText(frame, celda, (50, 80), cv.FONT_HERSHEY_SIMPLEX, 0.8, MARKER_COLOR, 2)

                            current_time = time.time()
                            if (last_sent_cell != (row, col)) or (current_time - last_sent_time > send_interval):
                                mensaje_pos = f"POS {row},{col}\n"
                                #print(f"[TCP] Enviando: {mensaje_pos.strip()}")
                                for _ in range(5):
                                    if sock:
                                        print(f"[TCP] Enviando: {mensaje_pos.strip()}")
                                        sock.send(mensaje_pos.encode())
                                        time.sleep(10/1000)
                                last_sent_cell = (row, col)
                                last_sent_time = current_time

            # === ORIENTACIÓN ===
            if mobile_id in detected and 25 in detected:
                rvec_99, tvec_99 = estimate_pose_from_corners(detected[mobile_id][0], marker_length)
                rvec_25, tvec_25 = estimate_pose_from_corners(detected[25][0], marker_length)

                cv.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec_99, tvec_99, 0.03)
                cv.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec_25, tvec_25, 0.03)

                def get_yaw_from_rvec(rvec):
                    R, _ = cv.Rodrigues(rvec)
                    yaw = np.arctan2(R[1, 0], R[0, 0])
                    return np.degrees(yaw)

                yaw_mobile = get_yaw_from_rvec(rvec_99)
                yaw_ref = get_yaw_from_rvec(rvec_25)
                delta_yaw = (yaw_mobile - yaw_ref + 180) % 360 - 180

                cv.putText(frame, f"Delta yaw: {delta_yaw:.1f} deg", (50, 110), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                current_time = time.time()
                if (last_sent_angle is None or abs(delta_yaw - last_sent_angle) > angle_send_threshold or (current_time - last_sent_time > send_interval)):
                    mensaje_yaw = f"ANG {delta_yaw:.2f}\n"
                    print(f"[TCP] Enviando: {mensaje_yaw.strip()}")
                    if sock:
                        sock.send(mensaje_yaw.encode())
                    last_sent_angle = delta_yaw
                    last_sent_time = current_time

    cv.imshow("Tracking ArUco", frame)
    key = cv.waitKey(1) & 0xFF
    if key == 27:
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
