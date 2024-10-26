import cv2
import time
import numpy as np
import colorama as col
from djitellopy import Tello
from FlyLib3.vision.apriltag import ApriltagDetector, ApriltagDetectionResult
from FlyLib3.math.pid import PID
import threading

drone = Tello()
apriltag_detector = ApriltagDetector()
last_detected_apriltag = None
align_pid = PID(0.1, 0, 0)
alignment_error_px = 40

def on_apriltag_detected(apriltag: ApriltagDetectionResult, frame: np.ndarray):
    match apriltag.tag_id:
        case 1:
            print("Apriltag 1 detected")
            drone.move_forward(80)
        case 2:
            print("Apriltag 2 detected")
            drone.move_left(80)
        case 3:
            print("Apriltag 3 detected")
            drone.move_right(80)
        case 4:
            print("Apriltag 4 detected")
            drone.land()

def stream_video():
    global last_detected_apriltag

    while True:
        # Obtener el frame del dron
        frame = drone.get_frame_read().frame

        # Verificar si el frame es válido
        if frame is None or frame.size == 0:
            continue

        # Convertir BGR a RGB para corregir los colores
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Detectar Apriltags en el frame
        apriltag_detections = apriltag_detector.detect(frame_rgb)

        # Si se detecta un Apriltag
        if apriltag_detections:
            current_detected_apriltag = apriltag_detections[0]  # Asumimos solo uno, puedes ajustar

            # Evitar procesar el mismo Apriltag repetidamente, pero permitir nuevas detecciones tras 2 seg.
            if current_detected_apriltag.tag_id != last_detected_apriltag:
                on_apriltag_detected(current_detected_apriltag, frame_rgb)
                last_detected_apriltag = current_detected_apriltag.tag_id

                # Restablecer `last_detected_apriltag` después de 2 segundos para permitir nuevas detecciones
                threading.Timer(2.0, lambda: reset_last_tag()).start()

        # Mostrar el frame
        cv2.imshow("Tello Video Stream", frame_rgb)

        # Presionar 'q' para salir del stream
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def reset_last_tag():
    global last_detected_apriltag
    last_detected_apriltag = None  # Restablecer para permitir nuevas detecciones

def main():
    # Configurar el dron
    print(col.Style.BRIGHT + "Conectando al dron..." + col.Style.RESET_ALL)
    while not drone.get_current_state():
        try:
            drone.connect()
            print(col.Fore.GREEN + "Dron conectado!" + col.Style.RESET_ALL)
            break
        except Exception as e:
            print(col.Fore.YELLOW + "No se pudo conectar al dron, reintentando..." + col.Style.RESET_ALL)
            print(e)
            continue
    
    drone.streamon()
    drone.set_speed(25)
    drone.takeoff()

    print(col.Fore.GREEN + str(drone.get_battery()) + col.Style.RESET_ALL)

    # Iniciar un hilo separado para el streaming de video
    video_thread = threading.Thread(target=stream_video)
    video_thread.start()

    # Dejar el dron en vuelo durante 200 segundos
    time.sleep(200)

    # Aterrizar el dron después de 200 segundos
    print("Aterrizando después de 200 segundos...")
    drone.land()

    # Asegurarse de que el hilo de video se cierre correctamente y detener el stream
    video_thread.join()
    drone.streamoff()

    # Liberar los recursos de OpenCV
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    