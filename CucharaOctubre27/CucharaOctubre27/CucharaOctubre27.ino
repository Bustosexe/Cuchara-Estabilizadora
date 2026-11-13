// Placa: ESP32 Dev Module
#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

// =====================================================
// HARDWARE
// Solo esta parte usa pines, Wire, Servo, Serial, etc.
// Si cambia la placa o el sensor, se toca solo esto.
// =====================================================
#define PIN_SERVO_ROLL   19
#define PIN_SERVO_PITCH  23
#define PIN_BOTON        0    // boton BOOT
#define PIN_SDA          21
#define PIN_SCL          22

#define SERVO_ROLL   0
#define SERVO_PITCH  1

MPU6050 mpu;
Servo servoRoll, servoPitch;

bool iniciarHardware() {
  Serial.begin(115200);
  pinMode(PIN_BOTON, INPUT_PULLUP);

  Wire.begin(PIN_SDA, PIN_SCL);
  mpu.initialize();
  mpu.setDLPFMode(3);  // filtro interno ~44 Hz

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servoRoll.setPeriodHertz(50);
  servoPitch.setPeriodHertz(50);
  servoRoll.attach(PIN_SERVO_ROLL, 500, 2400);
  servoPitch.attach(PIN_SERVO_PITCH, 500, 2400);

  return mpu.testConnection();
}

// Devuelve aceleracion en g y giro en grados/s
void leerImu(float &ax, float &ay, float &az, float &gx, float &gy) {
  int16_t rax, ray, raz, rgx, rgy, rgz;
  mpu.getMotion6(&rax, &ray, &raz, &rgx, &rgy, &rgz);
  ax = rax / 16384.0;
  ay = ray / 16384.0;
  az = raz / 16384.0;
  gx = rgx / 131.0;
  gy = rgy / 131.0;
}

void escribirServo(int servo, float grados) {
  int us = 500 + (int)(grados * 1900.0 / 180.0);
  if (servo == SERVO_ROLL) servoRoll.writeMicroseconds(us);
  else                     servoPitch.writeMicroseconds(us);
}

bool botonPresionado() { return digitalRead(PIN_BOTON) == LOW; }

unsigned long tiempoUs() { return micros(); }
unsigned long tiempoMs() { return millis(); }
void esperarMs(unsigned long ms) { delay(ms); }

void mensaje(const char *texto) { Serial.println(texto); }

void enviarDatos(float roll, float pitch, float sRoll, float sPitch) {
  Serial.print("Roll:");        Serial.print(roll);
  Serial.print(",Pitch:");      Serial.print(pitch);
  Serial.print(",ServoRoll:");  Serial.print(sRoll);
  Serial.print(",ServoPitch:"); Serial.println(sPitch);
}

// =====================================================
// LOGICA
// Solo usa las funciones de arriba, nunca el hardware directo.
// =====================================================

// Ajustes
const float DIR_ROLL  = -1.0;   // poner 1.0 si el servo gira al reves
const float DIR_PITCH = -1.0;
const float CENTRO    = 90.0;
const float MAX_ROLL  = 85.0;
const float MAX_PITCH = 70.0;

const float ZONA_MUERTA = 0.8;  // empieza a moverse si el error supera esto
const float ZONA_PARADA = 0.3;  // se detiene si el error baja de esto
const float VEL_MIN = 0.6;
const float VEL_MAX = 8.0;
const float FACTOR_VEL = 0.5;

const float PITCH_BLOQUEO = 65.0;  // con tanto pitch el roll no es confiable
const float PITCH_LIBERA  = 55.0;

const unsigned long CICLO_US = 10000;  // 100 Hz

// Estado
float offGx = 0, offGy = 0;
float roll = 0, pitch = 0;
float refRoll = 0, refPitch = 0;
bool rollBloqueado = false;

float posRoll = CENTRO, posPitch = CENTRO;
bool movRoll = false, movPitch = false;

unsigned long ultimoCiclo = 0, ultimoEnvio = 0;
bool botonAntes = false;

void leerSensor(float &accRoll, float &accPitch, float &velRoll, float &velPitch, float &g) {
  float ax, ay, az, gx, gy;
  leerImu(ax, ay, az, gx, gy);
  accRoll  = atan2(ay, az) * 180.0 / PI;
  accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  velRoll  = gx - offGx;
  velPitch = gy - offGy;
  g = sqrt(ax * ax + ay * ay + az * az);
}

void calibrarGiroscopio() {
  float ax, ay, az, gx, gy;
  float sumaX = 0, sumaY = 0;
  for (int i = 0; i < 300; i++) {
    leerImu(ax, ay, az, gx, gy);
    sumaX += gx;
    sumaY += gy;
    esperarMs(3);
  }
  offGx = sumaX / 300;
  offGy = sumaY / 300;
}

void tarar() {
  refRoll = roll;
  refPitch = pitch;
  mensaje("Nivel tarado");
}

// Filtro complementario
void actualizarAngulos(float dt) {
  float accRoll, accPitch, velRoll, velPitch, g;
  leerSensor(accRoll, accPitch, velRoll, velPitch, g);

  // Si hay mucha aceleracion extra se confia menos en el acelerometro
  float peso = (fabs(g - 1.0) < 0.15) ? 0.04 : 0.002;

  pitch = (1 - peso) * (pitch + velPitch * dt) + peso * accPitch;

  if (!rollBloqueado && fabs(pitch) > PITCH_BLOQUEO) {
    rollBloqueado = true;
  } else if (rollBloqueado && fabs(pitch) < PITCH_LIBERA) {
    rollBloqueado = false;
    roll = accRoll;
  }

  if (!rollBloqueado) {
    roll = (1 - peso) * (roll + velRoll * dt) + peso * accRoll;
  }
}

// Mueve el servo de a poco hacia el objetivo
void moverSuave(int servo, float &pos, float objetivo, bool &moviendo) {
  float error = objetivo - pos;
  float absError = fabs(error);

  if (absError > ZONA_MUERTA) moviendo = true;
  if (absError < ZONA_PARADA) moviendo = false;
  if (!moviendo) return;

  float vel = constrain(absError * FACTOR_VEL, VEL_MIN, VEL_MAX);
  if (vel > absError) vel = absError;

  pos += (error > 0) ? vel : -vel;
  escribirServo(servo, pos);
}

void controlarServos() {
  float objRoll  = rollBloqueado ? posRoll : CENTRO + DIR_ROLL * (roll - refRoll);
  float objPitch = CENTRO + DIR_PITCH * (pitch - refPitch);

  objRoll  = constrain(objRoll,  CENTRO - MAX_ROLL,  CENTRO + MAX_ROLL);
  objPitch = constrain(objPitch, CENTRO - MAX_PITCH, CENTRO + MAX_PITCH);

  moverSuave(SERVO_ROLL,  posRoll,  objRoll,  movRoll);
  moverSuave(SERVO_PITCH, posPitch, objPitch, movPitch);
}

void setup() {
  if (iniciarHardware()) mensaje("MPU6050 OK");
  else                   mensaje("ERROR: MPU6050 no responde");

  escribirServo(SERVO_ROLL, CENTRO);
  escribirServo(SERVO_PITCH, CENTRO);

  calibrarGiroscopio();

  float accRoll, accPitch, velRoll, velPitch, g;
  leerSensor(accRoll, accPitch, velRoll, velPitch, g);
  roll = accRoll;
  pitch = accPitch;
  tarar();

  botonAntes = botonPresionado();
  ultimoCiclo = tiempoUs();
}

void loop() {
  unsigned long ahora = tiempoUs();
  if (ahora - ultimoCiclo >= CICLO_US) {
    float dt = (ahora - ultimoCiclo) / 1000000.0;
    ultimoCiclo = ahora;
    actualizarAngulos(dt);
    controlarServos();
  }

  // Al apretar el boton se toma la posicion actual como nivel
  bool boton = botonPresionado();
  if (boton && !botonAntes) tarar();
  botonAntes = boton;

  if (tiempoMs() - ultimoEnvio >= 50) {
    ultimoEnvio = tiempoMs();
    enviarDatos(roll - refRoll, pitch - refPitch, posRoll - CENTRO, posPitch - CENTRO);
  }
}
