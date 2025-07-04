import serial
import time
import csv

# --- Configuración ---
# Asegúrate de que este sea el puerto COM correcto para tu ESP32
PORT = 'COM5'
BAUD_RATE = 115200
# Timeout para la lectura del puerto serie en segundos
SERIAL_TIMEOUT = 2

def run_datalogger():
    """
    Se conecta al dispositivo serie, ejecuta la captura de datos y los guarda en un archivo CSV.
    """
    # Generar un nombre de archivo único con la fecha y hora actual
    timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"plant_data_{timestamp}.csv"
    
    print(f"Iniciando el registro de datos. Se guardará en: {filename}")

    try:
        # Abrir puerto serie. El bloque 'with' asegura que se cierre automáticamente.
        with serial.Serial(PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT) as ser:
            # Abrir archivo CSV para escribir.
            with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
                # Crear un objeto writer de CSV
                csv_writer = csv.writer(csvfile)
                
                # Escribir la fila de encabezado para MATLAB (iddata)
                csv_writer.writerow(['tiempo', 'u (entrada)', 'v (salida)'])
                print("Archivo CSV creado con encabezados. Esperando al dispositivo...")

                # Limpiar buffers de entrada/salida para empezar desde cero
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                time.sleep(2) # Dar tiempo al ESP32 para que se prepare

                # Enviar comando START para iniciar el movimiento y la medición en el ESP32
                print("Enviando comando 'START'...")
                ser.write(b'START\n')

                start_time = None
                
                # Bucle principal para leer los datos del ESP32
                while True:
                    try:
                        # Leer una línea desde el puerto serie
                        line = ser.readline().decode('utf-8', errors='ignore').strip()

                        # Si la línea está vacía, continuar a la siguiente iteración
                        if not line:
                            continue

                        # Detectar el mensaje 'STARTED' para iniciar el cronómetro
                        if 'STARTED' in line:
                            print("¡Proceso iniciado en el ESP32! Comenzando a registrar...")
                            start_time = time.time()
                            continue
                        
                        # Detectar el mensaje 'END' para terminar el bucle
                        if 'END' in line:
                            print("Proceso finalizado por el ESP32. Cerrando el registro.")
                            break
                        
                        # Si el proceso ha comenzado, procesar la línea de datos
                        if start_time:
                            # Parsear los datos: ángulo (entrada u) y distancia (salida v)
                            parts = line.split(',')
                            if len(parts) == 2:
                                # Calcular el tiempo transcurrido desde el inicio
                                elapsed_time = time.time() - start_time
                                
                                # Asignar entrada y salida
                                u_input_angle = int(parts[0])
                                v_output_distance = float(parts[1])
                                
                                # Escribir los datos en el archivo CSV
                                csv_writer.writerow([f"{elapsed_time:.4f}", u_input_angle, v_output_distance])
                                
                                # Imprimir en consola para feedback en tiempo real
                                print(f"Tiempo: {elapsed_time:.2f}s, Ángulo (u): {u_input_angle}°, Distancia (v): {v_output_distance:.2f} mm")

                    except ValueError:
                        print(f"Advertencia: No se pudo convertir la línea a número: '{line}'")
                        continue
                    except KeyboardInterrupt:
                        print("\nRegistro interrumpido por el usuario.")
                        ser.write(b'END\n') # Intentar detener el ESP32
                        break
                        
    except serial.SerialException as e:
        print(f"Error: No se pudo abrir el puerto serie {PORT}. ¿Está conectado?")
        print(f"Detalle del error: {e}")
    except Exception as e:
        print(f"Ocurrió un error inesperado: {e}")

    finally:
        print("Script finalizado.")


if __name__ == '__main__':
    run_datalogger()
