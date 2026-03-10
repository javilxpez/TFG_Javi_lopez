import minimalmodbus
import serial
import time
import numpy as np
from collections import deque

# --- CONFIGURACIÓN ---
PUERTO_SERVO = 'COM8'
PUERTO_SENSOR = 'COM7'
BAUDIOS_SERVO = 38400
WINDOW_SIZE = 20 

lecturas_recientes = deque(maxlen=WINDOW_SIZE)

servo = minimalmodbus.Instrument(PUERTO_SERVO, 1)
servo.serial.baudrate = BAUDIOS_SERVO
servo.serial.timeout = 0.05
servo.clear_buffers_before_each_transaction = True

try:
    esp32 = serial.Serial(PUERTO_SENSOR, 115200, timeout=0.01)
    time.sleep(2)
    print(f"✅ Sensor conectado. Modo TORQUE (Límites C03.47/48 activos).")
except Exception as e:
    print(f"❌ Error sensor: {e}")
    exit()

def filtrar_por_iqr(datos_lista):
    if len(datos_lista) < 5: return datos_lista
    q1, q3 = np.percentile(datos_lista, [25, 75])
    iqr = q3 - q1
    limite_inferior = q1 - (1.5 * iqr)
    limite_superior = q3 + (1.5 * iqr)
    filtrados = [x for x in datos_lista if limite_inferior <= x <= limite_superior]
    return filtrados if filtrados else datos_lista

def procesar_valor_filtrado(nuevo_valor):
    lecturas_recientes.append(nuevo_valor)
    if len(lecturas_recientes) < 5: return nuevo_valor
    datos_limpios = filtrar_por_iqr(list(lecturas_recientes))
    return sum(datos_limpios) / len(datos_limpios)

def extraer_adc(linea):
    if "ADC:" in linea:
        try:
            return float(linea.split(",")[0].split(":")[1])
        except: return None
    return None

def main():
    try:
        print("Configurando motor y límites de velocidad...")
        
        # 1. Modo Torque (C00.00 = 2)
        servo.write_register(0, 2, functioncode=6) 
        
        # 2. Selección de Referencia de Torque (C03.40 = 0)
        servo.write_register(832, 0, functioncode=6) 
        
        # --- LIMITACIÓN DE VELOCIDAD EN MODO TORQUE ---
        # 3. C03.47: Speed limit in torque control (Forward) -> Registro 839
        # 4. C03.48: Speed limit in torque control (Reverse) -> Registro 840
        LIMITE_RPM = 1500  # Ajusta este valor según necesites
        servo.write_register(839, LIMITE_RPM, functioncode=6)
        servo.write_register(840, LIMITE_RPM, functioncode=6)

        # 5. Habilitar Motor (Servo ON)
        servo.write_register(1041, 1, functioncode=6)
        time.sleep(0.5)

        print(f"\n--- CONTROL ACTIVO (Límite: {LIMITE_RPM} RPM) ---")

        while True:
            if esp32.in_waiting > 150:
                esp32.reset_input_buffer()

            linea_cruda = esp32.readline().decode('utf-8', errors='ignore').strip()
            val_adc_crudo = extraer_adc(linea_cruda)
            
            if val_adc_crudo is not None:
                val_adc_filtrado = procesar_valor_filtrado(val_adc_crudo)

                # Escalado: 1000 = 100% Torque
                torque_set = int(abs(val_adc_filtrado/10))
                torque_set = min(torque_set, 1000) 

                try:
                    # Aplicar Torque (C03.41 -> Registro 833)
                    servo.write_register(833, torque_set, functioncode=6)
                    
                    vel_real = servo.read_register(16385, signed=True)
                    trq_real = servo.read_register(16387, signed=True)
                    
                    print(f"ADC: {val_adc_filtrado:7.1f} | T_Set: {torque_set/10.0:4.1f}% | Real: {trq_real/10.0:4.1f}% | Vel: {vel_real:4} RPM", end="\r")
                except:
                    pass

            time.sleep(0.005) 

    except KeyboardInterrupt:
        print("\n🛑 Deteniendo...")
        servo.write_register(1041, 0, functioncode=6)
    finally:
        esp32.close()

if __name__ == "__main__":
    main()