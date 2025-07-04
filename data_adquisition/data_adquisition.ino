#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// --- Configuración del servo ---
const int servoPin = 4;
const long serialBaud = 115200;
Servo myServo;

// --- Parámetros de la onda senoidal ---
float sineFreq = 0.48;    // Frecuencia en Hz
float minAngle = 69.0;    // Ángulo mínimo en grados
float maxAngle = 80.0;    // Ángulo máximo en grados
const int restingAngle = 74; // Ángulo de reposo

// --- Sensor VL53L0X ---
#define I2C_SDA 8
#define I2C_SCL 9
Adafruit_VL53L0X lox;

// --- Variables del filtro de Kalman ---
float q = 0.125; // Covarianza del ruido del proceso
float r = 8.0;   // Covarianza del ruido de la medición
float x_est = 0; // Distancia estimada
float p_est = 1; // Covarianza del error de estimación

// --- Control de estado ---
bool running = false;
unsigned long startMillis;
const unsigned long duration = 20000; // Duración en milisegundos (20 s)

// Función para calcular el ángulo con una onda senoidal
double sineAngle(float freq, float minA, float maxA) {
  double amp = (maxA - minA) / 2.0;
  double offset = (maxA + minA) / 2.0;
  double t = (millis() - startMillis) / 1000.0; // Usar tiempo relativo al inicio
  return offset + amp * sin(2.0 * PI * freq * t);
}

void setup() {
  Serial.begin(serialBaud);
  while (!Serial)
    ;

  // --- Inicializar y configurar el servo ---
  myServo.setPeriodHertz(50);
  // Acoplar el servo en el pin correspondiente
  myServo.attach(servoPin, 500, 2400);
  // **CAMBIO:** Mover el servo a la posición de reposo (74°) al iniciar.
  // El servo mantendrá esta posición hasta que se le den nuevas instrucciones.
  myServo.write(restingAngle);

  // --- Inicializar I2C y el sensor ---
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!lox.begin()) {
    Serial.println("Fallo al iniciar el sensor VL53L0X");
    while (1)
      delay(100);
  }

  Serial.println("Listo. Envía 'START' para comenzar.");
}

void loop() {
  // --- Manejo de comandos desde el puerto serie ---
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase("START")) {
      if (!running) {
        running = true;
        startMillis = millis(); // Reiniciar el contador de tiempo
        Serial.println("STARTED");
      }
    } else if (cmd.equalsIgnoreCase("END")) {
      if (running) {
        running = false;
        myServo.write(restingAngle); // **CAMBIO:** Mover a 74°
        Serial.println("END - Comando recibido");
        // **CAMBIO IMPORTANTE:** No desacoplar (detach) el servo para que mantenga la posición.
      }
    }
  }

  // --- Lógica de movimiento y medición ---
  if (running) {
    // Comprobar si se ha agotado el tiempo de ejecución
    if (millis() - startMillis >= duration) {
      running = false;
      myServo.write(restingAngle); // **CAMBIO:** Mover a 74° al finalizar
      Serial.println("END - Tiempo finalizado");
      // **CAMBIO IMPORTANTE:** No desacoplar el servo.
    } else {
      // --- Operación normal mientras está en ejecución ---
      // Calcular y escribir el ángulo del servo
      int angle = constrain(int(sineAngle(sineFreq, minAngle, maxAngle) + 0.5), 0, 180);
      myServo.write(angle);

      // Medir la distancia con el sensor
      VL53L0X_RangingMeasurementData_t m;
      lox.rangingTest(&m, false);
      float z = (m.RangeStatus != 4) ? m.RangeMilliMeter : 0;

      // Aplicar filtro de Kalman (predicción)
      float x_pred = x_est;
      float p_pred = p_est + q;
      // Aplicar filtro de Kalman (actualización)
      float k_gain = p_pred / (p_pred + r);
      x_est = x_pred + k_gain * (z - x_pred);
      p_est = (1 - k_gain) * p_pred;

      // Enviar datos por el puerto serie: ángulo, distancia filtrada
      Serial.printf("%d,%.2f\n", angle, x_est);
    }
  }

  // Pequeña pausa para estabilizar el bucle
  delay(20);
}

