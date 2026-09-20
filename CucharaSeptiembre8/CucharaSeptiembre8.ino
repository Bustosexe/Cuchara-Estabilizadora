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

// --- TAREAS BUSTOS: Abstracción de hardware ---
#define PIN_SERVO 11
#define MOVER_SERVO(pos) servo.write(pos)
#define IMPRIMIR(x) Serial.print(x)
#define IMPRIMIR_LN(x) Serial.println(x)

MPU6050 mpu;
Servo servo;

float anguloServo = 180;
float suavizado = 0.9;
float ganancia = 1.5;
float umbralMovimiento = 1.0;  // Umbral mínimo para mover el servo (en grados)

// --- TAREAS BUSS: control de tiempo sin bloquear (reemplaza delay) ---
const unsigned long INTERVALO_MS = 20;
unsigned long tUltimaActualizacion = 0;

// Variables compartidas actualizadas por CtrlEstabilizado()
// (NEVEU: cuando esté lista ReadSensors(), ax/ay/az deberían venir de ahí)
float anguloXActual = 0;
float anguloCalculadoActual = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  servo.attach(PIN_SERVO);

  if (mpu.testConnection()) {
    Serial.println("MPU6050 conectado correctamente");
  } else {
    Serial.println("Error al conectar el MPU6050");
  }

  servo.write(anguloServo);
}

// --- TAREA BUSS: loop() despejado, sin delay, sin lógica de sensores/servo ---
void loop() {
  unsigned long ahora = millis();

  if (ahora - tUltimaActualizacion >= INTERVALO_MS) {
    tUltimaActualizacion = ahora;

    CtrlEstabilizado();
    CtrlServo(anguloCalculadoActual);
    TxSerie(anguloXActual, anguloServo);
  }
}

// ==========================================
// FUNCIONES BUSS
// ==========================================

// Se ocupa ÚNICAMENTE de calcular el ángulo que debería tener el servo.
// TODO(NEVEU): cuando exista ReadSensors(), reemplazar la lectura directa
// del MPU acá por las variables globales que esa función actualice.
void CtrlEstabilizado() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  float angleX = atan2(ax, az) * 180.0 / PI;
  float nuevaPos = 90 - (angleX * ganancia);
  nuevaPos = constrain(nuevaPos, 0, 180);

  // Aplicar suavizado
  float anguloCalculado = (suavizado * anguloServo) + ((1 - suavizado) * nuevaPos);

  // Guardamos en variables globales para que loop() se las pase a CtrlServo/TxSerie
  anguloXActual = angleX;
  anguloCalculadoActual = anguloCalculado;
}

// ==========================================
// FUNCIONES BUSTOS
// ==========================================

// Función encargada ÚNICAMENTE de mover el servo según corresponda
void CtrlServo(float anguloDeseado) {
  if (abs(anguloDeseado - anguloServo) > umbralMovimiento) anguloServo = anguloDeseado, MOVER_SERVO(anguloServo);
}

// Función encargada ÚNICAMENTE de enviar datos por el puerto serie
void TxSerie(float angleX, float anguloAct) {
  IMPRIMIR("Ángulo X: ");
  IMPRIMIR(angleX);
  IMPRIMIR("  Servo: ");
  IMPRIMIR_LN(anguloAct);
}
