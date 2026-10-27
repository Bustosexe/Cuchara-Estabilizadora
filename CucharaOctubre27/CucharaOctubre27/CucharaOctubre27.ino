#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

// ==========================================================
//  CUCHARA ESTABILIZADORA
//  servo19 (atras): gira el conjunto alrededor del eje del mango (ROLL)
//  servo23 (cuchara): sube/baja la cuchara (PITCH)
//  MPU en el mango: el eje X del sensor va a lo largo del mango
// ==========================================================

MPU6050 mpu;
Servo servo19;
Servo servo23;

const int PIN_SERVO19 = 19;
const int PIN_SERVO23 = 23;
const int PIN_TARA    = 0;   // boton BOOT: la posicion actual pasa a ser "nivel"

// ---------- AJUSTES QUE PROBABLEMENTE HAYA QUE TOCAR ----------
// Signo de cada angulo medido (si al inclinar el angulo sale al reves, cambialo)
const float SIGNO_ROLL  = 1.0;
const float SIGNO_PITCH = 1.0;

// Direccion en que se mueve cada servo para compensar (si empeora, cambialo)
const float DIR_19 = -1.0;
const float DIR_23 = -1.0;

// Limites de los servos
const float CENTRO_19   = 90.0;
const float MAX_DESV_19 = 85.0;   // 5..175 (90 grados hacia cada lado, con margen)
const float CENTRO_23   = 90.0;
const float MAX_DESV_23 = 70.0;   // 20..160

// Comportamiento
const float ZONA_MUERTA = 0.8;    // grados de error para ARRANCAR a mover
const float ZONA_PARADA = 0.3;    // grados de error para FRENAR (histeresis)
const float VEL_MIN     = 0.6;    // grados por ciclo (suave)
const float VEL_MAX     = 8.0;    // grados por ciclo (rapido)
const float FACTOR_VEL  = 0.5;
const float GANANCIA    = 1.0;

// Bloqueo del roll cerca de la vertical (el calculo del roll se vuelve inestable)
const float PITCH_BLOQUEO = 65.0; // por encima de este cabeceo, el roll se congela
const float PITCH_LIBERA  = 55.0; // por debajo de este valor, vuelve a funcionar

// ---------- ESTADO ----------
const unsigned long INTERVALO_US = 10000; // ciclo de 10 ms
unsigned long ultimoCiclo = 0;
unsigned long ultimoSerial = 0;

float offGx = 0, offGy = 0;
float roll = 0, pitch = 0;
float refRoll = 0, refPitch = 0;
bool rollBloqueado = false;

float pos19 = CENTRO_19, pos23 = CENTRO_23;
bool mov19 = false, mov23 = false;
bool botonAntes = true;

// ==========================================================
void leerCrudo(float &accRoll, float &accPitch, float &rateRoll, float &ratePitch, float &magG) {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float fax = ax, fay = ay, faz = az;

  // Roll: giro alrededor del eje del mango (eje X del sensor)
  accRoll   = SIGNO_ROLL  * atan2(fay, faz) * 180.0 / PI;
  rateRoll  = SIGNO_ROLL  * ((gx - offGx) / 131.0);

  // Pitch: punta arriba/abajo (giro alrededor de Y)
  accPitch  = SIGNO_PITCH * atan2(-fax, sqrt(fay * fay + faz * faz)) * 180.0 / PI;
  ratePitch = SIGNO_PITCH * ((gy - offGy) / 131.0);

  magG = sqrt(fax * fax + fay * fay + faz * faz) / 16384.0;
}

void calibrarGiro() {
  long sx = 0, sy = 0;
  int16_t a, b, c, gx, gy, gz;
  for (int i = 0; i < 300; i++) {
    mpu.getMotion6(&a, &b, &c, &gx, &gy, &gz);
    sx += gx;
    sy += gy;
    unsigned long t = millis();
    while (millis() - t < 3) {}   // solo en el arranque
  }
  offGx = sx / 300.0;
  offGy = sy / 300.0;
}

void tararNivel() {
  refRoll = roll;
  refPitch = pitch;
  Serial.println("Nivel tarado");
}

void actualizarAngulos(float dt) {
  float accRoll, accPitch, rateRoll, ratePitch, magG;
  leerCrudo(accRoll, accPitch, rateRoll, ratePitch, magG);

  // Si la aceleracion no es ~1g, la mano sacude: confiar en el giroscopio
  float pesoAcc = (fabs(magG - 1.0) < 0.15) ? 0.04 : 0.002;

  // El pitch siempre es confiable
  pitch = (1.0 - pesoAcc) * (pitch + ratePitch * dt) + pesoAcc * accPitch;

  // Histeresis para bloquear/liberar el roll cerca de la vertical
  if (!rollBloqueado && fabs(pitch) > PITCH_BLOQUEO) {
    rollBloqueado = true;
  } else if (rollBloqueado && fabs(pitch) < PITCH_LIBERA) {
    rollBloqueado = false;
    roll = accRoll;   // resincroniza de golpe con el acelerometro
  }

  if (!rollBloqueado) {
    roll = (1.0 - pesoAcc) * (roll + rateRoll * dt) + pesoAcc * accRoll;
  }
}

void moverServo(Servo &s, float &pos, float objetivo, bool &moviendo) {
  float error = objetivo - pos;
  float absErr = fabs(error);

  if (!moviendo && absErr > ZONA_MUERTA) moviendo = true;
  if (moviendo && absErr < ZONA_PARADA) moviendo = false;

  if (moviendo) {
    float vel = constrain(absErr * FACTOR_VEL, VEL_MIN, VEL_MAX);
    if (vel > absErr) vel = absErr;
    pos += (error > 0) ? vel : -vel;
    s.writeMicroseconds(500 + (int)(pos * 1900.0 / 180.0));
  }
}

void controlServos() {
  float dRoll  = roll  - refRoll;
  float dPitch = pitch - refPitch;

  // Con el mango vertical, el servo19 se queda donde esta
  float obj19 = rollBloqueado ? pos19 : CENTRO_19 + DIR_19 * GANANCIA * dRoll;
  float obj23 = CENTRO_23 + DIR_23 * GANANCIA * dPitch;

  obj19 = constrain(obj19, CENTRO_19 - MAX_DESV_19, CENTRO_19 + MAX_DESV_19);
  obj23 = constrain(obj23, CENTRO_23 - MAX_DESV_23, CENTRO_23 + MAX_DESV_23);

  moverServo(servo19, pos19, obj19, mov19);
  moverServo(servo23, pos23, obj23, mov23);
}

// ==========================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  pinMode(PIN_TARA, INPUT_PULLUP);

  mpu.initialize();
  mpu.setDLPFMode(3); // filtro interno ~44 Hz
  if (mpu.testConnection()) Serial.println("MPU6050 OK");
  else Serial.println("ERROR: MPU6050 no responde");

  // Con el mango quieto: calibrar giroscopio y partir del acelerometro
  calibrarGiro();
  float accRoll, accPitch, r1, r2, m;
  leerCrudo(accRoll, accPitch, r1, r2, m);
  roll = accRoll;
  pitch = accPitch;
  tararNivel();

  // Servos: fijar posicion al energizar
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo19.setPeriodHertz(50);
  servo23.setPeriodHertz(50);
  servo19.attach(PIN_SERVO19, 500, 2400);
  servo23.attach(PIN_SERVO23, 500, 2400);
  servo19.writeMicroseconds(500 + (int)(pos19 * 1900.0 / 180.0));
  servo23.writeMicroseconds(500 + (int)(pos23 * 1900.0 / 180.0));

  ultimoCiclo = micros();
}

void loop() {
  unsigned long ahora = micros();

  if (ahora - ultimoCiclo >= INTERVALO_US) {
    float dt = (ahora - ultimoCiclo) / 1000000.0;
    ultimoCiclo = ahora;

    actualizarAngulos(dt);
    controlServos();
  }

  // Boton BOOT: tarar nivel
  bool boton = digitalRead(PIN_TARA);
  if (!boton && botonAntes) tararNivel();
  botonAntes = boton;

  // Telemetria para el Serial Plotter (cada 50 ms)
  if (millis() - ultimoSerial >= 50) {
    ultimoSerial = millis();
    Serial.print("Roll:");   Serial.print(roll - refRoll);
    Serial.print(",Pitch:"); Serial.print(pitch - refPitch);
    Serial.print(",S19:");   Serial.print(pos19 - CENTRO_19);
    Serial.print(",S23:");   Serial.println(pos23 - CENTRO_23);
  }
}