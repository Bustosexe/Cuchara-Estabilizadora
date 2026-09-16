#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>  // ¡Librería específica para el ESP32!

MPU6050 mpu;
Servo servo;

float anguloServo = 180;
float suavizado = 0.9;
float ganancia = 1.5;
float umbralMovimiento = 1.0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  
  // En el ESP32 conectamos el servo de prueba al pin 18
  servo.attach(18);

  if (mpu.testConnection()) {
    Serial.println("MPU6050 conectado correctamente");
  } else {
    Serial.println("Error al conectar el MPU6050");
  }

  servo.write(anguloServo);
}

void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  float angleX = atan2(ax, az) * 180.0 / PI;
  float nuevaPos = 90 - (angleX * ganancia);
  nuevaPos = constrain(nuevaPos, 0, 180);

  // Aplicar suavizado
  float anguloCalculado = (suavizado * anguloServo) + ((1 - suavizado) * nuevaPos);

  if (abs(anguloCalculado - anguloServo) > umbralMovimiento) {
    anguloServo = anguloCalculado;
    servo.write(anguloServo);
  }
  
  Serial.print("Ángulo X: ");
  Serial.print(angleX);
  Serial.print("  Servo: ");
  Serial.println(anguloServo);

  delay(20);
}