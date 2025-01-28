#!/usr/bin/env python3

import serial
import time

# Paramètres du LiDAR
LIDAR_MODULE_NUMBER = 16
LIDAR_FRAME_SIZE = 114
INVALID_DISTANCE = 20000  # Valeur correspondant à une mesure invalide (20 000 mm)

# Finite State Machine
class LidarVL53L1XParsingStatus:
    BEGIN = 0
    INFO = 1
    DISTANCE_MES = 2

class LidarVL53L1X:
    def __init__(self):
        self.parsing_status = LidarVL53L1XParsingStatus.BEGIN
        self.active_sensor = 0
        self.ROI_number = 0
        self.measure_number = 0
        self.rx_storage = bytearray(LIDAR_FRAME_SIZE * 2)
        self.measure = [[0, 0] for _ in range(LIDAR_MODULE_NUMBER)]  # Historique des mesures

def get_lidar_data(lidar: LidarVL53L1X) -> LidarVL53L1X:
    """
    Computing and storing the lidar distance data
    """
    reading_head = 0
    wait_for_head_cmp = 0
    wait_for_fill = 0

    while reading_head < len(lidar.rx_storage):
        reading_head_limit = len(lidar.rx_storage) if reading_head < LIDAR_FRAME_SIZE else LIDAR_FRAME_SIZE

        while reading_head < reading_head_limit:
            if lidar.parsing_status == LidarVL53L1XParsingStatus.BEGIN:
                if lidar.rx_storage[reading_head] == 0xFF:
                    wait_for_head_cmp += 1
                else:
                    wait_for_head_cmp = 0

                if wait_for_head_cmp > 5:
                    wait_for_head_cmp = 0
                    lidar.parsing_status = LidarVL53L1XParsingStatus.INFO

            elif lidar.parsing_status == LidarVL53L1XParsingStatus.INFO:
                if wait_for_head_cmp == 0:
                    lidar.active_sensor = lidar.rx_storage[reading_head]
                    wait_for_head_cmp += 1
                elif wait_for_head_cmp == 1:
                    lidar.ROI_number = lidar.rx_storage[reading_head]
                    wait_for_head_cmp += 1
                elif wait_for_head_cmp > 1:
                    lidar.measure_number = lidar.rx_storage[reading_head]
                    wait_for_head_cmp = 0
                    lidar.parsing_status = LidarVL53L1XParsingStatus.DISTANCE_MES

            elif lidar.parsing_status == LidarVL53L1XParsingStatus.DISTANCE_MES:
                if wait_for_fill % 3 == 0 and wait_for_head_cmp < lidar.measure_number:
                    lidar.measure[wait_for_head_cmp][0] = lidar.rx_storage[reading_head]
                    wait_for_fill += 1
                elif wait_for_fill % 3 == 1 and wait_for_head_cmp < lidar.measure_number:
                    lidar.measure[wait_for_head_cmp][1] = lidar.rx_storage[reading_head]
                    lidar.measure[wait_for_head_cmp][1] <<= 8
                    wait_for_fill += 1
                elif wait_for_fill % 3 == 2 and wait_for_head_cmp < lidar.measure_number:
                    lidar.measure[wait_for_head_cmp][1] += lidar.rx_storage[reading_head]
                    wait_for_fill += 1
                    wait_for_head_cmp += 1

                if wait_for_head_cmp >= lidar.measure_number:
                    wait_for_fill = 0
                    wait_for_head_cmp = 0
                    lidar.parsing_status = LidarVL53L1XParsingStatus.BEGIN

            reading_head += 1
        if reading_head >= LIDAR_FRAME_SIZE:
            reading_head_limit = LIDAR_FRAME_SIZE

    return lidar

# Fonction principale pour lire les données LiDAR à partir du port série
def read_lidar_data(serial_port):
    try:
        # Ouvrir le port série
        ser = serial.Serial(serial_port, baudrate=115200, timeout=1)
        lidar = LidarVL53L1X()

        print(f"Ouverture du port série {serial_port} réussie.")
        print("Lecture des trames LiDAR...")

        while True:
            # Lire une trame du LiDAR
            data = ser.read(LIDAR_FRAME_SIZE)
            if len(data) == LIDAR_FRAME_SIZE:
                lidar.rx_storage = data
                lidar = get_lidar_data(lidar)

                # Affichage des mesures avec ajustement des valeurs
                print("Mesures LiDAR :")
                for i in range(LIDAR_MODULE_NUMBER):
                    # Calculer la distance
                    distance = (lidar.measure[i][1] << 8) + lidar.measure[i][0]

                    # Retrancher 20000 si la distance est trop grande
                    if distance > INVALID_DISTANCE:
                        distance -= INVALID_DISTANCE

                    print(f"Module {i+1}: {distance} mm")

            else:
                print("Erreur de lecture de la trame LiDAR.")

            time.sleep(0.5)  # Attente avant de lire la prochaine trame

    except serial.SerialException as e:
        print(f"Erreur de communication série: {e}")

# Remplacer '/dev/serial0' par le port série correct pour votre configuration
serial_port = '/dev/serial0'

# Lancer la lecture des données LiDAR
read_lidar_data(serial_port)
