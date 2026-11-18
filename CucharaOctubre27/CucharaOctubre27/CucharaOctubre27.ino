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

#define INICIAR_SERIAL()         Serial.begin(115200)
#define INICIAR_I2C()            Wire.begin(PIN_SDA, PIN_SCL)
#define INICIAR_BOTON()          pinMode(PIN_BOTON, INPUT_PULLUP)
#define INICIAR_IMU(m)           (m).initialize(), (m).setDLPFMode(3)   // filtro interno ~44 Hz
#define INICIAR_TIMERS_PWM()     ESP32PWM::allocateTimer(0), ESP32PWM::allocateTimer(1)
#define INICIAR_SERVO(s, pin)    (s).setPeriodHertz(50), (s).attach(pin, 500, 2400)

#define IMU_OK(m)                ((m).testConnection())
#define LEER_IMU_CRUDO(m, ax, ay, az, gx, gy, gz)  (m).getMotion6(&ax, &ay, &az, &gx, &gy, &gz)
#define ESCRIBIR_SERVO(s, grados)  (s).writeMicroseconds(500 + (int)((grados) * 1900.0 / 180.0))
#define BOTON_PRESIONADO()       (digitalRead(PIN_BOTON) == LOW)

#define TIEMPO_US()              micros()
#define TIEMPO_MS()              millis()

#define IMPRIMIR(x)              Serial.print(x)
#define IMPRIMIR_LN(x)           Serial.println(x)

// =====================================================
// LOGICA
// Solo usa las macros de arriba, nunca el hardware directo.
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
const float UMBRAL_ENVIO = 0.1;    // cambio minimo para volver a imprimir

const float ACEL_POR_G      = 16384.0;
const float GIRO_POR_GRADO  = 131.0;

const unsigned long CICLO_US = 10000;   // 100 Hz
const unsigned long PERIODO_ENVIO_MS = 50;
const int MUESTRAS_CALIBRACION = 300;
const unsigned long PERIODO_CALIBRACION_MS = 3;

// Estado (vive dentro de loop(), no hay variables globales)
enum Fase { INICIO, CALIBRANDO, ACTIVA };

struct Estado {
  MPU6050 mpu;
  Servo servoRoll, servoPitch;
  Fase fase = INICIO;

  int muestras = 0;
  float sumaGx = 0, sumaGy = 0;
  unsigned long ultimaMuestraMs = 0;

  float offGx = 0, offGy = 0;
  float roll = 0, pitch = 0;
  float refRoll = 0, refPitch = 0;
  bool rollBloqueado = false;

  float posRoll = CENTRO, posPitch = CENTRO;
  bool movRoll = false, movPitch = false;

  unsigned long ultimoCiclo = 0, ultimoEnvio = 0;
  bool botonAntes = false;
  float ultimoRoll = 0, ultimoPitch = 0, ultimoServoRoll = 0, ultimoServoPitch = 0;
};

// Devuelve aceleracion en g y giro en grados/s
void leerImu(Estado &e, float &ax, float &ay, float &az, float &gx, float &gy) {
  int16_t rax, ray, raz, rgx, rgy, rgz;
  LEER_IMU_CRUDO(e.mpu, rax, ray, raz, rgx, rgy, rgz);
  ax = rax / ACEL_POR_G;
  ay = ray / ACEL_POR_G;
  az = raz / ACEL_POR_G;
  gx = rgx / GIRO_POR_GRADO;
  gy = rgy / GIRO_POR_GRADO;
}

void leerSensor(Estado &e, float &accRoll, float &accPitch, float &velRoll, float &velPitch, float &g) {
  float ax, ay, az, gx, gy;
  leerImu(e, ax, ay, az, gx, gy);
  accRoll  = atan2(ay, az) * 180.0 / PI;
  accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  velRoll  = gx - e.offGx;
  velPitch = gy - e.offGy;
  g = sqrt(ax * ax + ay * ay + az * az);
}

void enviarDato(const char *nombre, float valor) {
  IMPRIMIR(nombre);
  IMPRIMIR(":");
  IMPRIMIR_LN(valor);
}

// Imprime el valor solo si cambio respecto al ultimo enviado
void enviarSiCambia(const char *nombre, float valor, float &ultimo) {
  if (fabs(valor - ultimo) < UMBRAL_ENVIO) return;
  ultimo = valor;
  enviarDato(nombre, valor);
}

void tarar(Estado &e) {
  e.refRoll = e.roll;
  e.refPitch = e.pitch;
  IMPRIMIR_LN("Nivel tarado");
}

void iniciar(Estado &e) {
  INICIAR_SERIAL();
  INICIAR_BOTON();
  INICIAR_I2C();
  INICIAR_IMU(e.mpu);
  INICIAR_TIMERS_PWM();
  INICIAR_SERVO(e.servoRoll, PIN_SERVO_ROLL);
  INICIAR_SERVO(e.servoPitch, PIN_SERVO_PITCH);

  IMPRIMIR_LN(IMU_OK(e.mpu) ? "MPU6050 OK" : "ERROR: MPU6050 no responde");

  ESCRIBIR_SERVO(e.servoRoll, CENTRO);
  ESCRIBIR_SERVO(e.servoPitch, CENTRO);

  e.botonAntes = BOTON_PRESIONADO();
  e.fase = CALIBRANDO;
}

// Con el mango quieto: una muestra cada 3 ms, sin frenar el programa
void calibrarGiroscopio(Estado &e) {
  if (TIEMPO_MS() - e.ultimaMuestraMs < PERIODO_CALIBRACION_MS) return;
  e.ultimaMuestraMs = TIEMPO_MS();

  float ax, ay, az, gx, gy;
  leerImu(e, ax, ay, az, gx, gy);
  e.sumaGx += gx;
  e.sumaGy += gy;
  if (++e.muestras < MUESTRAS_CALIBRACION) return;

  e.offGx = e.sumaGx / MUESTRAS_CALIBRACION;
  e.offGy = e.sumaGy / MUESTRAS_CALIBRACION;

  // Partir del acelerometro
  float accRoll, accPitch, velRoll, velPitch, g;
  leerSensor(e, accRoll, accPitch, velRoll, velPitch, g);
  e.roll = accRoll;
  e.pitch = accPitch;
  tarar(e);

  e.ultimoCiclo = TIEMPO_US();
  e.fase = ACTIVA;
}

// Filtro complementario
void actualizarAngulos(Estado &e, float dt) {
  float accRoll, accPitch, velRoll, velPitch, g;
  leerSensor(e, accRoll, accPitch, velRoll, velPitch, g);

  // Si hay mucha aceleracion extra se confia menos en el acelerometro
  float peso = (fabs(g - 1.0) < 0.15) ? 0.04 : 0.002;

  e.pitch = (1 - peso) * (e.pitch + velPitch * dt) + peso * accPitch;

  // Histeresis para bloquear/liberar el roll cerca de la vertical
  bool libera = e.rollBloqueado && fabs(e.pitch) < PITCH_LIBERA;
  e.roll = libera ? accRoll : e.roll;   // al liberar, resincroniza de golpe
  e.rollBloqueado = (e.rollBloqueado && !libera) || fabs(e.pitch) > PITCH_BLOQUEO;

  if (!e.rollBloqueado) e.roll = (1 - peso) * (e.roll + velRoll * dt) + peso * accRoll;
}

// Mueve el servo de a poco hacia el objetivo
void moverSuave(Servo &servo, float &pos, float objetivo, bool &moviendo) {
  float error = objetivo - pos;
  float absError = fabs(error);

  // Histeresis: arranca por encima de ZONA_MUERTA, frena por debajo de ZONA_PARADA
  moviendo = moviendo ? (absError >= ZONA_PARADA) : (absError > ZONA_MUERTA);
  if (!moviendo) return;

  float vel = constrain(absError * FACTOR_VEL, VEL_MIN, VEL_MAX);
  vel = (vel > absError) ? absError : vel;

  pos += (error > 0) ? vel : -vel;
  ESCRIBIR_SERVO(servo, pos);
}

void controlarServos(Estado &e) {
  float objRoll  = e.rollBloqueado ? e.posRoll : CENTRO + DIR_ROLL * (e.roll - e.refRoll);
  float objPitch = CENTRO + DIR_PITCH * (e.pitch - e.refPitch);

  objRoll  = constrain(objRoll,  CENTRO - MAX_ROLL,  CENTRO + MAX_ROLL);
  objPitch = constrain(objPitch, CENTRO - MAX_PITCH, CENTRO + MAX_PITCH);

  moverSuave(e.servoRoll,  e.posRoll,  objRoll,  e.movRoll);
  moverSuave(e.servoPitch, e.posPitch, objPitch, e.movPitch);
}

void cicloControl(Estado &e) {
  unsigned long ahora = TIEMPO_US();
  if (ahora - e.ultimoCiclo < CICLO_US) return;

  float dt = (ahora - e.ultimoCiclo) / 1000000.0;
  e.ultimoCiclo = ahora;
  actualizarAngulos(e, dt);
  controlarServos(e);
}

// Al apretar el boton se toma la posicion actual como nivel
void atenderBoton(Estado &e) {
  bool boton = BOTON_PRESIONADO();
  if (boton && !e.botonAntes) tarar(e);
  e.botonAntes = boton;
}

void enviarDatos(Estado &e) {
  if (TIEMPO_MS() - e.ultimoEnvio < PERIODO_ENVIO_MS) return;
  e.ultimoEnvio = TIEMPO_MS();

  enviarSiCambia("Roll",       e.roll - e.refRoll,      e.ultimoRoll);
  enviarSiCambia("Pitch",      e.pitch - e.refPitch,    e.ultimoPitch);
  enviarSiCambia("ServoRoll",  e.posRoll - CENTRO,      e.ultimoServoRoll);
  enviarSiCambia("ServoPitch", e.posPitch - CENTRO,     e.ultimoServoPitch);
}

void actualizar(Estado &e) {
  switch (e.fase) {
    case INICIO:
      iniciar(e);
      break;
    case CALIBRANDO:
      calibrarGiroscopio(e);
      break;
    case ACTIVA:
      cicloControl(e);
      atenderBoton(e);
      enviarDatos(e);
      break;
  }
}

void setup() {}

void loop() {
  static Estado estado;
  actualizar(estado);
}
