/*#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo servo;

int anguloServo = 90;  // Posición neutral (nivelado)

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  servo.attach(11); // Conectá el servo al pin D11

  if (mpu.testConnection()) {
    Serial.println("MPU6050 conectado correctamente");
  } else {
    Serial.println("Error al conectar el MPU6050");
  }

  servo.write(anguloServo);  // Inicialmente nivelado
}

void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Convertir aceleración en X a ángulo aproximado
  float angleX = atan2(ax, az) * 180.0 / PI;

  Serial.print("Ángulo X aproximado: ");
  Serial.println(angleX);

  // Ajustar servomotor en dirección opuesta al ángulo
  // Ganancia de compensación (multiplicador)
  float ganancia = 1.5;

  int ajuste = angleX * ganancia;

  // Invertimos el ajuste para que sea en dirección opuesta
  int nuevaPos = 90 - ajuste;

  // Limitar a rango válido del servo
  nuevaPos = constrain(nuevaPos, 0, 180);

  servo.write(nuevaPos);

  delay(100);
}*/



/*
#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo servo;

float anguloServo = 90;  // Posición neutral
float suavizado = 0.9;   // Valor entre 0 (nada suave) y 1 (muy suave)
float ganancia = 1.5;    // Ajuste de sensibilidad

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  servo.attach(11); // Pin del servo

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

  // Cálculo del ángulo en X
  float angleX = atan2(ax, az) * 180.0 / PI;

  // Aplicar ganancia e invertir
  float nuevaPos = 90 - (angleX * ganancia);

  // Suavizado exponencial
  anguloServo = (suavizado * anguloServo) + ((1 - suavizado) * nuevaPos);

  // Limitar a 0-180
  anguloServo = constrain(anguloServo, 0, 180);

  // Mover el servo
  servo.write(anguloServo);

  // Mostrar por consola
  Serial.print("Ángulo X: ");
  Serial.print(angleX);
  Serial.print("  Servo: ");
  Serial.println(anguloServo);

  delay(20); // Refresco rápido
}*/


/*
#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo servo;

float anguloServo = 90;
float suavizado = 0.9;
float ganancia = 1.5;

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
  anguloServo = (suavizado * anguloServo) + ((1 - suavizado) * nuevaPos);
  anguloServo = constrain(anguloServo, 0, 180);
  servo.write(anguloServo);

  Serial.print("Ángulo X: ");
  Serial.print(angleX);
  Serial.print("  Servo: ");
  Serial.println(anguloServo);

  delay(20);
}
*/


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

