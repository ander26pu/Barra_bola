#include <Arduino.h>
#include <ESP32Servo.h>

// --- Configuración del servo ---
const int servoPin = 4;           // Pin de señal del servo
const long serialBaud = 115200;   // Velocidad del puerto serie
Servo myServo;

// --- Parámetros de la onda senoidal ---
float sineFreq   = 0.48;   // Frecuencia en Hz
float minAngle   = 69.0;  // Ángulo mínimo en grados
float maxAngle   = 80.0;  // Ángulo máximo en grados

// Función que devuelve el ángulo en grados basado en una onda senoidal
float sineAngle(float frequency, float minA, float maxA) {
  float amplitude = (maxA - minA) / 2.0;            // Semiamplitud
  float offset    = (maxA + minA) / 2.0;            // Desplazamiento medio
  float t         = millis() / 1000.0;              // Tiempo en segundos
  return offset + amplitude * sin(2.0 * PI * frequency * t);
}

void setup() {
  Serial.begin(serialBaud);
  while (!Serial) { }  // Espera a que el Serial esté listo (opcional)

  // Configura el PWM del servo (50 Hz) y el rango de pulso típico (0.5–2.4 ms)
  myServo.setPeriodHertz(50);
  myServo.attach(servoPin, 500 /*µs pulso mínimo*/, 2400 /*µs pulso máximo*/);

  Serial.println("Moviendo servo con función seno...");
  Serial.print("Frecuencia: "); Serial.print(sineFreq); Serial.println(" Hz");
  Serial.print("Rango: "); Serial.print(minAngle); Serial.print("° – ");
  Serial.print(maxAngle); Serial.println("°");
}

void loop() {
  // Calcula el ángulo actual según la función seno
  float ang = sineAngle(sineFreq, minAngle, maxAngle);
  int angle = constrain(int(ang + 0.5), 0, 180);  // Redondeo y límite [0,180]

  myServo.write(angle);

  // Opcional: imprimir por serial cada cierto tiempo
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {  // cada 200 ms
    Serial.print("Ángulo: ");
    Serial.print(angle);
    Serial.println("°");
    lastPrint = millis();
  }
}
