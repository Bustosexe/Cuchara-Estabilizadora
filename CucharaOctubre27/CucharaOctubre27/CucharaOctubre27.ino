//Device: ESP32 Dev Module
#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h> //by Keving Harrington

// ==========================================================
//  CUCHARA ESTABILIZADORA (version con capa de abstraccion)
//  Solo la seccion "HARDWARE" toca pines, I2C, PWM, reloj o Serial.
//  La seccion "LOGICA" depende unicamente de las interfaces.
// ==========================================================

// ##########################################################
//  1) INTERFACES (contrato con el hardware)
// ##########################################################
struct LecturaImu { int16_t ax, ay, az, gx, gy, gz; };

class IImu {
public:
  virtual bool iniciar() = 0;
  virtual void leer(LecturaImu &l) = 0;
  virtual ~IImu() {}
};

class IServo {
public:
  virtual void iniciar() = 0;
  virtual void escribirUs(int microsegundos) = 0;
  virtual ~IServo() {}
};

class IBoton {
public:
  virtual void iniciar() = 0;
  virtual bool presionado() = 0;
  virtual ~IBoton() {}
};

class IReloj {
public:
  virtual unsigned long micros() = 0;
  virtual unsigned long millis() = 0;
  virtual void esperarMs(unsigned long ms) = 0;
  virtual ~IReloj() {}
};

class IConsola {
public:
  virtual void iniciar() = 0;
  virtual void texto(const char *t) = 0;
  virtual void numero(float v) = 0;
  virtual void nuevaLinea() = 0;
  virtual ~IConsola() {}
};

// ##########################################################
//  2) HARDWARE (unica parte que usa digitalRead, Wire, etc.)
// ##########################################################
const int PIN_SERVO19 = 19;
const int PIN_SERVO23 = 23;
const int PIN_TARA    = 0;   // boton BOOT

class ImuMPU6050 : public IImu {
  MPU6050 mpu;
public:
  bool iniciar() override {
    Wire.begin(21, 22);
    mpu.initialize();
    mpu.setDLPFMode(3); // filtro interno ~44 Hz
    return mpu.testConnection();
  }
  void leer(LecturaImu &l) override {
    mpu.getMotion6(&l.ax, &l.ay, &l.az, &l.gx, &l.gy, &l.gz);
  }
};

class ServoEsp32 : public IServo {
  Servo servo;
  int pin;
public:
  explicit ServoEsp32(int p) : pin(p) {}
  void iniciar() override {
    servo.setPeriodHertz(50);
    servo.attach(pin, 500, 2400);
  }
  void escribirUs(int us) override { servo.writeMicroseconds(us); }
};

class BotonGpio : public IBoton {
  int pin;
public:
  explicit BotonGpio(int p) : pin(p) {}
  void iniciar() override { pinMode(pin, INPUT_PULLUP); }
  bool presionado() override { return digitalRead(pin) == LOW; } // activo en bajo
};

class RelojArduino : public IReloj {
public:
  unsigned long micros() override { return ::micros(); }
  unsigned long millis() override { return ::millis(); }
  void esperarMs(unsigned long ms) override {
    unsigned long t = ::millis();
    while (::millis() - t < ms) {}
  }
};

class ConsolaSerial : public IConsola {
public:
  void iniciar() override { Serial.begin(115200); }
  void texto(const char *t) override { Serial.print(t); }
  void numero(float v) override { Serial.print(v); }
  void nuevaLinea() override { Serial.println(); }
};

// ##########################################################
//  3) LOGICA (no incluye ninguna funcion de hardware)
// ##########################################################
class Estabilizador {
  IImu &imu; IServo &servo19; IServo &servo23;
  IBoton &boton; IReloj &reloj; IConsola &con;

  // ---------- AJUSTES ----------
  const float SIGNO_ROLL  = 1.0;
  const float SIGNO_PITCH = 1.0;
  const float DIR_19 = -1.0;
  const float DIR_23 = -1.0;

  const float CENTRO_19   = 90.0;
  const float MAX_DESV_19 = 85.0;
  const float CENTRO_23   = 90.0;
  const float MAX_DESV_23 = 70.0;

  const float ZONA_MUERTA = 0.8;
  const float ZONA_PARADA = 0.3;
  const float VEL_MIN     = 0.6;
  const float VEL_MAX     = 8.0;
  const float FACTOR_VEL  = 0.5;
  const float GANANCIA    = 1.0;

  const float PITCH_BLOQUEO = 65.0;
  const float PITCH_LIBERA  = 55.0;

  const unsigned long INTERVALO_US = 10000;

  // ---------- ESTADO ----------
  unsigned long ultimoCiclo = 0, ultimoSerial = 0;
  float offGx = 0, offGy = 0;
  float roll = 0, pitch = 0, refRoll = 0, refPitch = 0;
  bool rollBloqueado = false;
  float pos19 = CENTRO_19, pos23 = CENTRO_23;
  bool mov19 = false, mov23 = false;
  bool botonAntes = false;

  static int aMicros(float pos) { return 500 + (int)(pos * 1900.0 / 180.0); }

  void leerCrudo(float &accRoll, float &accPitch, float &rateRoll, float &ratePitch, float &magG) {
    LecturaImu l;
    imu.leer(l);
    float fax = l.ax, fay = l.ay, faz = l.az;

    accRoll   = SIGNO_ROLL  * atan2(fay, faz) * 180.0 / PI;
    rateRoll  = SIGNO_ROLL  * ((l.gx - offGx) / 131.0);
    accPitch  = SIGNO_PITCH * atan2(-fax, sqrt(fay * fay + faz * faz)) * 180.0 / PI;
    ratePitch = SIGNO_PITCH * ((l.gy - offGy) / 131.0);
    magG = sqrt(fax * fax + fay * fay + faz * faz) / 16384.0;
  }

  void calibrarGiro() {
    long sx = 0, sy = 0;
    LecturaImu l;
    for (int i = 0; i < 300; i++) {
      imu.leer(l);
      sx += l.gx;
      sy += l.gy;
      reloj.esperarMs(3);
    }
    offGx = sx / 300.0;
    offGy = sy / 300.0;
  }

  void tararNivel() {
    refRoll = roll;
    refPitch = pitch;
    con.texto("Nivel tarado"); con.nuevaLinea();
  }

  void actualizarAngulos(float dt) {
    float accRoll, accPitch, rateRoll, ratePitch, magG;
    leerCrudo(accRoll, accPitch, rateRoll, ratePitch, magG);

    float pesoAcc = (fabs(magG - 1.0) < 0.15) ? 0.04 : 0.002;
    pitch = (1.0 - pesoAcc) * (pitch + ratePitch * dt) + pesoAcc * accPitch;

    if (!rollBloqueado && fabs(pitch) > PITCH_BLOQUEO) {
      rollBloqueado = true;
    } else if (rollBloqueado && fabs(pitch) < PITCH_LIBERA) {
      rollBloqueado = false;
      roll = accRoll;
    }

    if (!rollBloqueado) {
      roll = (1.0 - pesoAcc) * (roll + rateRoll * dt) + pesoAcc * accRoll;
    }
  }

  void moverServo(IServo &s, float &pos, float objetivo, bool &moviendo) {
    float error = objetivo - pos;
    float absErr = fabs(error);

    if (!moviendo && absErr > ZONA_MUERTA) moviendo = true;
    if (moviendo && absErr < ZONA_PARADA) moviendo = false;

    if (moviendo) {
      float vel = constrain(absErr * FACTOR_VEL, VEL_MIN, VEL_MAX);
      if (vel > absErr) vel = absErr;
      pos += (error > 0) ? vel : -vel;
      s.escribirUs(aMicros(pos));
    }
  }

  void controlServos() {
    float dRoll  = roll  - refRoll;
    float dPitch = pitch - refPitch;

    float obj19 = rollBloqueado ? pos19 : CENTRO_19 + DIR_19 * GANANCIA * dRoll;
    float obj23 = CENTRO_23 + DIR_23 * GANANCIA * dPitch;

    obj19 = constrain(obj19, CENTRO_19 - MAX_DESV_19, CENTRO_19 + MAX_DESV_19);
    obj23 = constrain(obj23, CENTRO_23 - MAX_DESV_23, CENTRO_23 + MAX_DESV_23);

    moverServo(servo19, pos19, obj19, mov19);
    moverServo(servo23, pos23, obj23, mov23);
  }

  void telemetria() {
    con.texto("Roll:");   con.numero(roll - refRoll);
    con.texto(",Pitch:"); con.numero(pitch - refPitch);
    con.texto(",S19:");   con.numero(pos19 - CENTRO_19);
    con.texto(",S23:");   con.numero(pos23 - CENTRO_23);
    con.nuevaLinea();
  }

public:
  Estabilizador(IImu &i, IServo &s19, IServo &s23, IBoton &b, IReloj &r, IConsola &c)
    : imu(i), servo19(s19), servo23(s23), boton(b), reloj(r), con(c) {}

  void iniciar() {
    con.iniciar();
    boton.iniciar();

    if (imu.iniciar()) { con.texto("MPU6050 OK"); }
    else               { con.texto("ERROR: MPU6050 no responde"); }
    con.nuevaLinea();

    calibrarGiro();
    float accRoll, accPitch, r1, r2, m;
    leerCrudo(accRoll, accPitch, r1, r2, m);
    roll = accRoll;
    pitch = accPitch;
    tararNivel();

    servo19.iniciar();
    servo23.iniciar();
    servo19.escribirUs(aMicros(pos19));
    servo23.escribirUs(aMicros(pos23));

    botonAntes = boton.presionado();
    ultimoCiclo = reloj.micros();
  }

  void actualizar() {
    unsigned long ahora = reloj.micros();
    if (ahora - ultimoCiclo >= INTERVALO_US) {
      float dt = (ahora - ultimoCiclo) / 1000000.0;
      ultimoCiclo = ahora;
      actualizarAngulos(dt);
      controlServos();
    }

    // Flanco de bajada del boton: tarar nivel
    bool p = boton.presionado();
    if (p && !botonAntes) tararNivel();
    botonAntes = p;

    if (reloj.millis() - ultimoSerial >= 50) {
      ultimoSerial = reloj.millis();
      telemetria();
    }
  }
};

// ##########################################################
//  4) ENSAMBLADO (aqui se elige el hardware concreto)
// ##########################################################
ImuMPU6050    imu;
ServoEsp32    servo19(PIN_SERVO19);
ServoEsp32    servo23(PIN_SERVO23);
BotonGpio     botonTara(PIN_TARA);
RelojArduino  reloj;
ConsolaSerial consola;

Estabilizador cuchara(imu, servo19, servo23, botonTara, reloj, consola);

void setup() {
  // Los timers PWM del ESP32 se reservan antes de crear los servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  cuchara.iniciar();
}

void loop() {
  cuchara.actualizar();
}
