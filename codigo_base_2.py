import serial
import time
import re  # para buscar con expresiones regulares
import numpy as np
import matplotlib.pyplot as plt
import math

# Configura el puerto del Arduino
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)  # timeout corto
time.sleep(2)  # esperar reinicio Arduino

# Variables globales para encoders
encR = 0
encL = 0

# Parámetros del robot
dw = 0.21   # distancia entre ruedas [m]
diametro_rueda = 0.1524  # 6 pulgadas en metros
radio_rueda = diametro_rueda / 2
tics_por_vuelta = 36
m_por_tic = (2 * np.pi * radio_rueda) / tics_por_vuelta

# Pose inicial (x, y, theta)
x, y, theta = 0.0, 0.0, 0.0

def normalizar_theta(theta):
    while theta > math.pi:
        theta -= 2*math.pi
    while theta < -math.pi:
        theta += 2*math.pi
    return theta

def send_command(left_speed, right_speed):
    """Envía comando de velocidad a los motores y lee respuesta con encoders."""
    global encR, encL, corIz, coDer

    cmd = f"L{left_speed} R{right_speed}\n"
    arduino.write(cmd.encode())
    print(f">>> {cmd.strip()}")

    # Leer todas las líneas disponibles


    # while arduino.in_waiting:
    #     line = arduino.readline().decode(errors='ignore').strip()
    #     if line:
    #         print("<<<", line)

    #         # Buscar si la línea contiene lecturas de encoders
    #         match = re.search(r"Encoder derecho:(\d+)\s+CoIzqu:(-?\d+(?:\.\d+)?)\s+CoDer:(-?\d+(?:\.\d+)?)\s+Encoder izquierdo:(\d+)", line)
    #         if match:
    #             encR = int(match.group(1))
    #             encL = int(match.group(4))
    #             corIz=float(match.group(2))
    #             corDer=float(match.group(3))
    #             print(f"[PY] EncR={encR},CorIz={corIz},CorDer={corDer}, EncL={encL}")

try:
    print("Presiona Ctrl+C para detener el script en cualquier momento.")

    while True:

        print("hola")

        if arduino.in_waiting > 0:
            line = arduino.readline().decode('utf-8').strip()
            try:
                encR, corIz, corDer, encL = map(float, line.split(','))
                print(f"[JETSON] EncR={encR}, CorIz={corIz}, CorDer={corDer}, EncL={encL}")
            except ValueError:
                print("Línea inválida:", line)  


        # if arduino.in_waiting > 0:
        #     line = arduino.readline().decode('utf-8').strip()
        #     try:
        #         # datos=line.split(',')
        #         encR, corIz, corDer, encL = map(float, line.split(','))
        #         # dato1=int(datos[0])
        #         # dato2=float(datos[1])
        #         # dato3=float(datos[2])
        #         # dato4=int(datos[3])
        #         print("hola")
        #         print("hola2")
        #         print(f"[JETSON] EncR={encR}, CorIz={corIz}, CorDer={corDer}, EncL={encL}")
        #         # aquí puedes guardarlos en un archivo o procesarlos
        #     except ValueError:
        #         print("Línea inválida:", line)        
        


        # arduino.write(b"L100 R100\n")
        # print(arduino.readline().decode().strip())
        # Ejemplo: avanzar
        send_command(50, 50)
        time.sleep(0.5)

        #arduino.write(b"GET_CURR\n")


        # Leer las respuestas (puedes leer varias líneas)
        # for i in range(4):  # Arduino manda varias líneas
        #     line = arduino.readline().decode('utf-8', errors='ignore').strip()
        #     if line:
        #         print("Arduino dice:", line)


        # # Detener motores
        # send_command(0, 0)
        # time.sleep(2)

except KeyboardInterrupt:
    print("\nDeteniendo motores y cerrando conexión...")
    send_command(0, 0)
    arduino.close()
    print("Listo. Script finalizado.")
