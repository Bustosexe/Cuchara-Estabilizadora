/*
Tareas y sus responsables: (Realizar las modificaciones correspondientes cada uno con la 
BUSS --> Despejar la función loop()
BUSTOS --> Crear función TxSerie() que sea la única encargada de enviar datos por el puerto serie.
NEVEU --> Crear función ReadSensors() que contenga todo lo relacionado a la lecturta de los sensores y actualice variables para utilizar en el resto del sistema.
BUSS --> Crear función CtrlEstabilizado() que se ocupe de actualizar el valor del ángulo que debería tener el servo.
BUSTOS --> Crear función CtrlServo() que sea la que se encargue de mover el servo según corresponda.
NEVEU --> Incorporar LedTest()
BUSS --> Eliminar los retardos de todo el sistema (delay)
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
