# --- 1. IMPORTAZIONI ---
import cv2
import numpy as np
import serial
import time
import logging
import threading
import argparse
import sys

# Importazione condizionale delle librerie TFLite
try:
    from tflite_support.task import core
    from tflite_support.task import processor
    from tflite_support.task import vision
    TFLITE_SUPPORT_AVAILABLE = True
except ImportError:
    TFLITE_SUPPORT_AVAILABLE = False

# --- 2. CONFIGURAZIONE GLOBALE E COSTANTI ---
# Configurazione del logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Impostazioni Arduino e Robot
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
CMD_FORWARD = "avanti\n"
CMD_STOP = "stop\n"
CMD_LEFT = "sinistra\n"
CMD_RIGHT = "destra\n"
CMD_BACKWARD = "indietro\n"
DISTANCE_CLOSE_MSG = "OBJ_VICINO"

# Impostazioni Video
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Configurazione per Modalità 'color'
LOWER_YELLOW = np.array([15, 100, 100])
UPPER_YELLOW = np.array([35, 255, 255])
MIN_RADIUS_COLOR = 10

# Configurazione per Modalità 'ml'
MODEL_ML = 'efficientdet_lite0.tflite'
TARGET_LABEL_ML = 'person'
NUM_THREADS_ML = 4

# Logica di Controllo Comune
TARGET_RADIUS = 80
RADIUS_TOLERANCE = 20
CENTER_ZONE_PERCENT = 0.35

# --- 3. VARIABILI GLOBALI DI STATO ---
# Oggetto per la connessione seriale, inizializzato in main()
arduino = None
# Eventi per la comunicazione tra thread
object_close_flag = threading.Event()
stop_thread = threading.Event()

# --- 4. FUNZIONI HELPER ---

# --- Funzioni di Comunicazione con Arduino ---
def send_command(cmd):
    """Funzione globale per inviare comandi all'Arduino."""
    if arduino and arduino.is_open:
        try:
            arduino.write(cmd.encode('utf-8'))
            logging.info(f"Comando inviato: {cmd.strip()}")
        except Exception as e:
            logging.error(f"Errore durante l'invio del comando: {e}")

def serial_reader_thread():
    """Thread che legge costantemente dalla seriale per messaggi di sicurezza (es. ostacolo)."""
    global arduino
    while not stop_thread.is_set():
        try:
            if arduino and arduino.in_waiting > 0:
                line = arduino.readline().decode('utf-8').strip()
                if line == DISTANCE_CLOSE_MSG:
                    object_close_flag.set()
                else:
                    object_close_flag.clear()
        except serial.SerialException as e:
            logging.error(f"Errore di lettura seriale: {e}")
            break  # Esci dal thread in caso di errore critico seriale
        except Exception as e:
            logging.error(f"Errore generico nel thread seriale: {e}")
            break
        time.sleep(0.05)

# --- Funzione di Visualizzazione ---
def visualize(image, detection_result, mode, color_target_info=None, ml_target_label=None):
    """
    Disegna i risultati del rilevamento sull'immagine per la visualizzazione a schermo.
    """
    if mode == 'color' and color_target_info:
        center = color_target_info['center']
        radius = color_target_info['radius']
        cv2.circle(image, center, radius, (0, 255, 0), 2)
        cv2.circle(image, center, 5, (0, 0, 255), -1)
        cv2.putText(image, f"Raggio: {radius}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    elif mode == 'ml' and detection_result:
        for detection in detection_result.detections:
            bbox = detection.bounding_box
            category = detection.categories[0]

            # Disegna il rettangolo di contorno
            start_point = (bbox.origin_x, bbox.origin_y)
            end_point = (bbox.origin_x + bbox.width, bbox.origin_y + bbox.height)
            color = (255, 0, 0)  # Blu
            thickness = 2
            cv2.rectangle(image, start_point, end_point, color, thickness)

            # Scrivi l'etichetta e il punteggio di confidenza
            label = f"{category.category_name} ({category.score:.2f})"
            cv2.putText(image, label, (bbox.origin_x, bbox.origin_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    cv2.imshow('Robot Vision', image)


# --- 5. CLASSE PRINCIPALE DI CONTROLLO ---
class HybridRobotTracker:
    """
    Classe che gestisce la logica di inseguimento e controllo del robot.
    """
    def __init__(self, mode, model_path=None, target_label=None):
        self.mode = mode
        self.target_label = target_label
        self.motors_current_command = CMD_STOP  # Inizializza il comando corrente

        if self.mode == 'ml':
            if not TFLITE_SUPPORT_AVAILABLE:
                raise ImportError("Libreria tflite-support non trovata. Assicurati che sia installata.")
            
            base_options = core.BaseOptions(file_name=model_path, num_threads=NUM_THREADS_ML)
            detection_options = processor.DetectionOptions(max_results=5, score_threshold=0.3)
            options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
            self.detector = vision.ObjectDetector.create_from_options(options)

    def find_target(self, frame):
        """Delega alla funzione di ricerca corretta in base alla modalità."""
        if self.mode == 'color':
            return self._find_target_by_color(frame)
        elif self.mode == 'ml':
            return self._find_target_by_ml(frame)
        return None, None

    def _find_target_by_color(self, frame):
        """Implementa la pipeline di visione classica per il colore."""
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        target_info = None
        if contours:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius > MIN_RADIUS_COLOR:
                target_info = {'center': (int(x), int(y)), 'radius': int(radius)}
        
        visualize(frame, [], self.mode, target_info)
        return target_info is not None, target_info

    def _find_target_by_ml(self, frame):
        """Implementa la pipeline di machine learning."""
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_tensor = vision.TensorImage.create_from_array(rgb_image)
        detection_result = self.detector.detect(input_tensor)
        
        target_info = None
        target_detected = False
        
        # Cerca il target principale tra i rilevamenti
        for detection in detection_result.detections:
            category = detection.categories[0]
            if category.category_name == self.target_label:
                target_detected = True
                bbox = detection.bounding_box
                center_x = bbox.origin_x + bbox.width / 2
                center_y = bbox.origin_y + bbox.height / 2
                radius = (bbox.width + bbox.height) / 4  # Raggio approssimato
                target_info = {'center': (int(center_x), int(center_y)), 'radius': int(radius)}
                break
        
        visualize(frame, detection_result, self.mode, None, self.target_label)
        return target_detected, target_info

    def control_robot(self, target_detected, target_info):
        """Logica decisionale per il movimento del robot."""
        command = CMD_STOP
        if object_close_flag.is_set():
            logging.warning("Ostacolo rilevato, robot fermo per sicurezza.")
            command = CMD_STOP
        elif target_detected and target_info:
            center_x = target_info['center'][0]
            radius = target_info['radius']
            
            center_zone_width = FRAME_WIDTH * CENTER_ZONE_PERCENT
            left_bound = (FRAME_WIDTH - center_zone_width) / 2
            right_bound = left_bound + center_zone_width
            
            # La logica dà priorità alla sterzata
            if center_x < left_bound:
                command = CMD_LEFT
            elif center_x > right_bound:
                command = CMD_RIGHT
            # Se centrato, regola la distanza
            elif radius < TARGET_RADIUS - RADIUS_TOLERANCE:
                command = CMD_FORWARD
            elif radius > TARGET_RADIUS + RADIUS_TOLERANCE:
                command = CMD_BACKWARD
            else:  # Target centrato e a distanza ottimale
                command = CMD_STOP
        else:  # Target non rilevato
            command = CMD_STOP
            
        # Invia comando solo se c'è un cambio di stato per evitare comandi ridondanti
        if command != self.motors_current_command:
            send_command(command)
            self.motors_current_command = command

# --- 6. FUNZIONE PRINCIPALE DI AVVIO ---
def main():
    """
    Funzione principale che inizializza l'hardware, il tracker e avvia il loop di elaborazione.
    """
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--mode', type=str, required=True, choices=['color', 'ml'], help='Modalità di tracking: "color" o "ml".')
    parser.add_argument('--model', type=str, default=MODEL_ML, help='Percorso del modello TFLite (solo per modalita "ml").')
    parser.add_argument('--targetLabel', type=str, default=TARGET_LABEL_ML, help="Etichetta dell'oggetto da tracciare (solo per modalita 'ml').")
    args = parser.parse_args()

    # Inizializzazione Hardware
    global arduino
    try:
        arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Attesa per la stabilizzazione della connessione seriale
        threading.Thread(target=serial_reader_thread, daemon=True).start()
        logging.info(f"Connessione Arduino stabilita su {SERIAL_PORT}")
    except serial.SerialException as e:
        logging.critical(f"ERRORE: Connessione Arduino fallita su {SERIAL_PORT}. Errore: {e}")
        sys.exit("Impossibile connettersi all'Arduino.")
        
    # Inizializzazione Tracker
    try:
        tracker = HybridRobotTracker(mode=args.mode, model_path=args.model, target_label=args.targetLabel)
        logging.info(f"Tracker inizializzato in modalità: {args.mode}")
    except Exception as e:
        logging.critical(f"ERRORE: Inizializzazione tracker fallita. Errore: {e}")
        if arduino and arduino.is_open:
            arduino.close()
        sys.exit("Errore critico durante l'inizializzazione del tracker.")

    # Inizializzazione Camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        logging.critical("ERRORE: Impossibile aprire la webcam.")
        stop_thread.set()
        if arduino and arduino.is_open:
            arduino.close()
        sys.exit("Errore: Webcam non accessibile.")
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    logging.info(f"Webcam inizializzata a {FRAME_WIDTH}x{FRAME_HEIGHT}")

    # Loop Principale
    start_time = time.time()
    frame_count = 0
    logging.info("Avvio del loop principale del robot...")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                logging.error("Impossibile leggere il frame dalla webcam. Riprovo...")
                time.sleep(0.1)
                continue

            target_detected, target_info = tracker.find_target(frame)
            tracker.control_robot(target_detected, target_info)

            # Calcolo FPS
            frame_count += 1
            if (time.time() - start_time) >= 1.0:
                fps = frame_count / (time.time() - start_time)
                logging.info(f"FPS: {fps:.2f}")
                frame_count = 0
                start_time = time.time()

            # Interruzione con tasto 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                logging.info("Tasto 'q' premuto. Terminazione applicazione.")
                break
    finally:
        # Pulizia risorse
        logging.info("Inizio pulizia risorse...")
        cap.release()
        cv2.destroyAllWindows()
        stop_thread.set()  # Segnala al thread seriale di terminare
        send_command(CMD_STOP) # Invia un comando di stop finale
        if arduino and arduino.is_open:
            arduino.close()
            logging.info("Connessione seriale chiusa.")
        logging.info("Applicazione terminata.")

# --- 7. BLOCCO DI ESECUZIONE ---
if __name__ == '__main__':
    main()