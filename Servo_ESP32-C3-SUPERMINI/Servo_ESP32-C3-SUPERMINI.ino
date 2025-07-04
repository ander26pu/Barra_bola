#include <Arduino.h>
#include <ESP32Servo.h>

// Pin al que está conectado el servo (ajústalo según tu conexión)
const int servoPin = 4;

// Velocidad del puerto serie
const long serialBaud = 115200;

// Objeto servo
Servo myServo;

void setup() {
  // Iniciar serial
  Serial.begin(serialBaud);
  while (!Serial) { }  // Espera a que el serial esté listo (opcional)

  // Configurar el servo: señal de 50 Hz (20 ms) y rango de pulso 500–2400 μs
  myServo.setPeriodHertz(50);
  myServo.attach(servoPin, 500 /*micros pulso mínimo*/, 2400 /*micros pulso máximo*/);

  Serial.println("Control de servo listo. Envía un ángulo (0–180) seguido de ENTER:");
}

void loop() {
  // Si hay datos en el serial...
  if (Serial.available() > 0) {
    // Leer hasta salto de línea
    String line = Serial.readStringUntil('\n');
    line.trim();  // Quitar espacios y retornos

    if (line.length() > 0) {
      // Convertir a entero
      int angle = line.toInt();
      // Limitar el ángulo a [0,180]
      angle = constrain(angle, 0, 180);

      // Mover el servo
      myServo.write(angle);

      // Confirmación por serial
      Serial.print("Servo movido a: ");
      Serial.print(angle);
      Serial.println("°");
    }
  }
}
