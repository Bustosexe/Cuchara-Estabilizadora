#include "arduino_secrets.h"

#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

// --- 1. Abstraerse de las funciones del hardware ---
#define PIN_SERVO 11
#define VELOCIDAD_SERIAL 9600
#define INICIAR_SERIAL Serial.begin(VELOCIDAD_SERIAL)
#define INICIAR_I2C Wire.begin()
#define MOVER_SERVO(pos) servo.write(pos)
#define IMPRIMIR(x) Serial.print(x)
#define IMPRIMIR_LN(x) Serial.println(x)

// ParÃ¡metros de configuracion (Reemplazan variables globales - Reglas 6 y 8)
#define SUAVIZADO 0.9
#define GANANCIA 1.5
#define UMBRAL_MOVIMIENTO 1.0
#define INTERVALO_MUESTREO 20 // Reemplaza al delay de 20ms

// Instancias de hardware (Unicas globales permitidas por necesidad de la libreria)
MPU6050 mpu;
Servo servo;

void setup() {
  INICIAR_SERIAL;
  INICIAR_I2C;
  mpu.initialize();
  servo.attach(PIN_SERVO);

  // --- 9. Ternario ?: y 10. Sin llaves ---
  mpu.testConnection() ? IMPRIMIR_LN("MPU6050 conectado correctamente") : IMPRIMIR_LN("Error al conectar el MPU6050");

  MOVER_SERVO(180);
}

void loop() {
  // --- 6 y 8. Sin variables globales (usamos static locales) ---
  static float angleX_actual = 0;
  static float anguloServo_actual = 180;
  static unsigned long ultimoTiempo = 0;

  // --- 3. No bloqueante y 9. Early return (Eliminamos el delay de 20ms) ---
  if (millis() - ultimoTiempo < INTERVALO_MUESTREO) return;
  ultimoTiempo = millis();

  // --- 2. Bucle principal lo mas limpio posible ---
  // Pasamos las variables por referencia (&) para que CtrlServo las pueda modificar
  CtrlServo(&angleX_actual, &anguloServo_actual);
  
  // Pasamos los valores por copia para que TxSerie solo los lea
  TxSerie(angleX_actual, anguloServo_actual);
}

// --- TAREA 2: Funcion encargada UNICAMENTE de calcular y mover el servo ---
void CtrlServo(float *angleX, float *anguloServo) {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  *angleX = atan2(ax, az) * 180.0 / PI;
  float nuevaPos = 90 - (*angleX * GANANCIA);
  nuevaPos = constrain(nuevaPos, 0, 180);

  // Aplicar suavizado
  float anguloCalculado = (SUAVIZADO * (*anguloServo)) + ((1 - SUAVIZADO) * nuevaPos);

  // --- 10. Sin llaves (usando el operador coma para ejecutar dos acciones en una linea) ---
  if (abs(anguloCalculado - *anguloServo) > UMBRAL_MOVIMIENTO) *anguloServo = anguloCalculado, MOVER_SERVO(*anguloServo);
}

// --- TAREA 1: Funcion encargada UNICAMENTE de enviar datos por serie ---
void TxSerie(float angleX, float anguloServo) {
  IMPRIMIR("Angulo X: ");
  IMPRIMIR(angleX);
  IMPRIMIR("  Servo: ");
  IMPRIMIR_LN(anguloServo);
}