#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

//ENTRADAS
//Serial
#define CFG_SERIAL          Serial.begin(9600)
#define IMPRIMIR_SERIAL(X)  Serial.println(X)

//MPU6050 (acelerometro)
#define CFG_I2C             Wire.begin()
#define CFG_MPU             mpu.initialize()
#define MPU_CONECTADO       mpu.testConnection()

//SALIDAS
//SERVO (corrector de movimiento)
#define PIN_SERVO           11
#define CFG_SERVO           servo.attach(PIN_SERVO)
#define MOVER_SERVO(x)      servo.write(x)

//LED_TEST
#define PIN_LED_TEST        13
#define CFG_LED_TEST        pinMode(PIN_LED_TEST, OUTPUT)
#define AJUSTAR_LED_TEST(x) digitalWrite(PIN_LED_TEST, x)

//—---------------------------------
MPU6050 mpu;
Servo servo;

//Parametros de control del servo
float anguloServo         = 90;    // Posicion inicial (nivelada)
const float suavizado         = 0.9;   // 0 = sin suavizado, 1 = muy suave
const float ganancia          = 1.5;   // Sensibilidad de correccion
const float umbralMovimiento  = 1.0;   // Umbral minimo de cambio (grados) para mover el servo

//—---------------------------------
void setup()
  {
  CFG_SERIAL;
  CFG_I2C;
  CFG_MPU;
  CFG_SERVO;
  CFG_LED_TEST;

  if (MPU_CONECTADO)  IMPRIMIR_SERIAL("MPU6050 conectado correctamente");
  else                IMPRIMIR_SERIAL("Error al conectar el MPU6050");

  MOVER_SERVO(anguloServo);
  }

void loop()
  {
  LedTest();
  CtrlServo();
  }

//Blink no bloqueante de testeo (misma logica que el codigo de referencia)
void LedTest()
  {
  static unsigned long millis_ant = 0;
  static bool blink = 0;
  static unsigned int tiempo_destello = 1000;

  if (millis() - millis_ant < tiempo_destello) return;
  millis_ant = millis();

  blink = !blink;
  AJUSTAR_LED_TEST(blink);
  }

//Lee el acelerometro y corrige la posicion del servo en sentido contrario al movimiento
void CtrlServo()
  {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  //Angulo aproximado de inclinacion en X
  float angleX = atan2(ax, az) * 180.0 / PI;

  //Posicion objetivo: se invierte el angulo para corregir en sentido contrario
  float nuevaPos = 90 - (angleX * ganancia);
  nuevaPos = constrain(nuevaPos, 0, 180);

  //Suavizado exponencial para evitar movimientos bruscos
  float anguloCalculado = (suavizado * anguloServo) + ((1 - suavizado) * nuevaPos);

  //Solo se mueve el servo si el cambio supera el umbral (evita jitter)
  if (abs(anguloCalculado - anguloServo) > umbralMovimiento)
    {
    anguloServo = anguloCalculado;
    MOVER_SERVO(anguloServo);
    }

  IMPRIMIR_SERIAL(String("Angulo X: ") + angleX + "  Servo: " + anguloServo);

  delay(20);
  }
