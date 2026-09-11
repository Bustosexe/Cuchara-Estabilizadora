//TAREA --> Eliminar Delay (by Profe)

#include <Wire.h>
#include <MPU6050.h>

#include <Servo.h>

MPU6050 mpu;
Servo servo;

float anguloServo = 180;
float suavizado = 0.9;
float ganancia = 1.5;
float umbralMovimiento = 1.0;  // Umbral mínimo para mover el servo (en grados)

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  servo.attach(11);

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

  // Solo mover el servo si el cambio es mayor al umbral
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
