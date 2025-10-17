import serial
import time
import numpy as np
import math
import csv
import re

# ========================
# CONFIGURACIÓN SERIAL
# ========================
def init_serial(port='/dev/ttyACM0', baudrate=115200):
    try:
        arduino = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(2)
        return arduino
    except serial.SerialException:
        print("[ERROR] No se pudo abrir el puerto serial.")
        exit()

# ========================
# PARÁMETROS DEL ROBOT
# ========================
DW = 0.42
DIAMETRO_RUEDA = 0.1524
RADIO_RUEDA = DIAMETRO_RUEDA / 2
TICS_POR_VUELTA = 36
M_POR_TIC = (2 * np.pi * RADIO_RUEDA) / TICS_POR_VUELTA

# ========================
# FUNCIONES AUXILIARES
# ========================
def delta_encoder(current, previous, max_count=65535):
    delta = current - previous
    if delta > max_count / 2:
        delta -= (max_count + 1)
    elif delta < -max_count / 2:
        delta += (max_count + 1)
    return delta

def normalizar_theta(theta):
    while theta > math.pi:
        theta -= 2 * math.pi
    while theta < -math.pi:
        theta += 2 * math.pi
    return theta

def send_command(arduino, left_speed, right_speed):
    cmd = f"L{left_speed} R{right_speed}\n"
    arduino.write(cmd.encode())
    print(f">>> {cmd.strip()}")

def leer_encoders(arduino):
    """Lee y parsea la línea de encoders desde Arduino."""
    while True:
        line = arduino.readline().decode('utf-8').strip()
        if not line:
            time.sleep(0.01)
            continue
        match = re.search(r"Encoder derecho:(\d+),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),Encoder izquierdo:(\d+)", line)
        if match:
            encR = int(match.group(1))
            encL = int(match.group(4))
            corIz = float(match.group(2))
            corDer = float(match.group(3))
            return encR, encL, corIz, corDer
        else:
            print(f"[ALERTA] Línea inválida: {line}")

# ========================
# CLASE DE ODOMETRÍA CON VELOCIDADES Y CORRIENTES SUAVIZADAS
# ========================
class Odometry:
    def __init__(self, encR0, encL0, alpha=0.3, alpha_corr=0.2):
        self.encR0 = encR0
        self.encL0 = encL0
        self.encR_prev = 0
        self.encL_prev = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.t_prev = time.perf_counter()
        self.alpha = alpha  # factor de suavizado velocidades
        self.alpha_corr = alpha_corr  # factor de suavizado corrientes

        # Velocidades suavizadas
        self.omega_R_smooth = 0.0
        self.omega_L_smooth = 0.0
        self.v_linear_smooth = 0.0
        self.v_angular_smooth = 0.0

        # Corrientes suavizadas
        self.corrienteD_smooth = 0.0
        self.corrienteI_smooth = 0.0

    def actualizar(self, encR_actual, encL_actual):
        t_now = time.perf_counter()
        dt = t_now - self.t_prev
        self.t_prev = t_now

        # Deltas de encoders
        deltaR = delta_encoder(encR_actual, self.encR_prev)
        deltaL = delta_encoder(encL_actual, self.encL_prev)

        # Distancias recorridas
        distR = deltaR * M_POR_TIC
        distL = deltaL * M_POR_TIC
        d = (distL + distR) / 2
        dtheta = (distL - distR) / DW

        # Velocidades instantáneas
        v_linear = d / dt if dt > 0 else 0.0
        v_angular = dtheta / dt if dt > 0 else 0.0
        omega_R = (deltaR * 2 * np.pi) / (TICS_POR_VUELTA * dt) if dt > 0 else 0.0
        omega_L = (deltaL * 2 * np.pi) / (TICS_POR_VUELTA * dt) if dt > 0 else 0.0

        # Suavizado exponencial
        self.v_linear_smooth = self.alpha * v_linear + (1 - self.alpha) * self.v_linear_smooth
        self.v_angular_smooth = self.alpha * v_angular + (1 - self.alpha) * self.v_angular_smooth
        self.omega_R_smooth = self.alpha * omega_R + (1 - self.alpha) * self.omega_R_smooth
        self.omega_L_smooth = self.alpha * omega_L + (1 - self.alpha) * self.omega_L_smooth

        # Actualizar pose
        self.theta = normalizar_theta(self.theta + dtheta)
        self.x += d * np.cos(self.theta + dtheta / 2)
        self.y += d * np.sin(self.theta + dtheta / 2)

        # Actualizar encoders previos
        self.encR_prev = encR_actual
        self.encL_prev = encL_actual

        return (deltaR, deltaL, self.x, self.y, self.theta,
                self.v_linear_smooth, self.v_angular_smooth,
                self.omega_R_smooth, self.omega_L_smooth)

    def suavizar_corrientes(self, corrienteD, corrienteI):
        self.corrienteD_smooth = (self.alpha_corr * corrienteD +
                                  (1 - self.alpha_corr) * self.corrienteD_smooth)
        self.corrienteI_smooth = (self.alpha_corr * corrienteI +
                                  (1 - self.alpha_corr) * self.corrienteI_smooth)
        return self.corrienteD_smooth, self.corrienteI_smooth

# ========================
# GUARDAR DATOS
# ========================
def guardar_trayectoria(trayectoria, archivo="trayectoria_robot.csv"):
    with open(archivo, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['EncR_actual','EncR_prev','DeltaR','EncL_actual','EncL_prev','DeltaL',
                         'EncR_raw','EncL_raw','X','Y','Theta','v_linear','v_angular',
                         'omega_R','omega_L','CorrienteD','CorrienteI'])
        writer.writerows(trayectoria)
    print(f"✅ Datos guardados en {archivo}")

# ========================
# BUCLE PRINCIPAL
# ========================
def main():
    arduino = init_serial()
    trayectoria = []

    print("Esperando lectura inicial de encoders...")
    arduino.reset_input_buffer()
    encR0, encL0, corIz0, corDer0 = leer_encoders(arduino)

    odometria = Odometry(encR0, encL0, alpha=0.02, alpha_corr=0.02)

    try:
        print("Presiona Ctrl+C para detener el script.")
        while True:
            send_command(arduino, 60, 50)

            encR_raw, encL_raw, corIz, corDer = leer_encoders(arduino)
            encR_actual = encR_raw - encR0
            encL_actual = encL_raw - encL0

            deltaR, deltaL, x, y, theta, v_lin, v_ang, omega_R, omega_L = odometria.actualizar(encR_actual, encL_actual)

            # Suavizar corrientes
            corrD_smooth, corrI_smooth = odometria.suavizar_corrientes(corDer, corIz)

            trayectoria.append([encR_actual, odometria.encR_prev, deltaR,
                                encL_actual, odometria.encL_prev, deltaL,
                                encR_raw, encL_raw, x, y, theta,
                                v_lin, v_ang, omega_R, omega_L,
                                corrD_smooth, corrI_smooth])

            print(f"X={x:.3f}, Y={y:.3f}, θ={theta:.2f}, v={v_lin:.3f} m/s, ω={v_ang:.3f} rad/s, "
                  f"ω_R={omega_R:.3f} rad/s, ω_L={omega_L:.3f} rad/s, "
                  f"CorrD={corrD_smooth:.2f}, CorrI={corrI_smooth:.2f}")

    except KeyboardInterrupt:
        print("\nDeteniendo motores y cerrando conexión...")
        send_command(arduino, 0, 0)
        arduino.close()
        guardar_trayectoria(trayectoria)

if __name__ == "__main__":
    main()
