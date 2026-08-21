//Realice un sistema que permita la comunicación de 3 dispositivos entre sí.
//1)- Uno de los Micros, debe Leer datos de 2 sensores distintos (LDR y Ultrasonido)

//implementar:
//Maquina de estado finito   XX (Hecho)
//Interrupciones externas    XX (Hecho - GPIO 0)
//Timer                      XX (Hecho - Ticker 30s)
//Led builtin                XX (Hecho)
//Low Power                  XX (Hecho)

// --- LIBRERÍAS ---
#include <WiFi.h>              // Conexión WiFi
#include <PubSubClient.h>      // Cliente MQTT
#include <Ticker.h>            // Timer

// --- PINES DE SENSORES (CORREGIDOS) ---

// Sensor de Luz (LDR)
#define LDR_PIN 34        // Pin P34 (¡Correcto!)
int valorLDR = 0;
const int UMBRAL_LUZ = 1500; // Umbral de disparo (0-4095). Ajusta este valor.

// Sensor Ultrasónico (HC-SR04)
// <-- ¡¡CORRECCIÓN CRÍTICA!!
#define TRIG_PIN 27       // Pin P27 (Pin seguro, NO USAR 12)
#define ECHO_PIN 14       // Pin P14 (¡Correcto!)
long distancia = 0;
const long UMBRAL_DISTANCIA = 50; // Umbral de disparo (50 cm)


// --- VARIABLES DE ESTADO (EXISTENTES) ---
unsigned long tiempoActual = 0;
unsigned long ultimoTiempo = 0;
int contadorEsperas = 0;
unsigned long controlParpadeo = 0;
unsigned long controlDormir = 0;
int contadorDurmiendo = 0;

// <-- Conexión a red abierta "Alumnos"
const char* ssid = "Alumnos";
const char* password = ""; // Vacío para red abierta
const char* mqtt_server = "broker.hivemq.com"; // <-- Broker HiveMQ
const int   mqtt_port = 1883;
// <-- ¡¡CORRECCIÓN!! FALTABA ESTA LÍNEA
const char* mqtt_client_id = "ESP32_Ezequiel_TP10"; // ID Unico

// <-- Tópicos actualizados
const char* Topico_LDR = "ITES/2/A/Microcontroladores1/SGBK/TP10/LDR";
const char* Topico_UltraSonico = "ITES/2/A/Microcontroladores1/SGBK/TP10/SensorUltraSonico";

// ---- Objetos globales ----
WiFiClient espClient;
PubSubClient client(espClient);

// <-- Enum de estados actualizado
enum Estado
{
  EsperandoSensor,
  LeyendoLDR,
  LeyendoUltrasonico,
  PublicandoTopicoLDR,
  PublicandoTopicoUltrasonico,
  Reconectando,
  Dormido
};

Estado EstadoActual = EsperandoSensor;

Ticker timerInactividad;
volatile bool hayQueMandarADormir = false;
volatile bool hayQueDespertar = false;
bool PonerADormir = false;

void DormirTimer()
{
  hayQueMandarADormir = true;
}

void iniciarTimerInactividad()
{
  timerInactividad.attach(30, DormirTimer); // 30 segundos
}

void reiniciarTimerInactividad()
{
  timerInactividad.detach();
  timerInactividad.attach(30, DormirTimer);
}

#define BOTON_BUILTIN 0
#define LED_BUILTIN 2

void IRAM_ATTR Despertar()
{
  hayQueDespertar = true;
}

// --- FUNCIÓN LECTURA ULTRASONIDO (SIN CAMBIOS) ---
long leerUltrasonido() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duracion = pulseIn(ECHO_PIN, HIGH, 30000); 
  if (duracion == 0) return -1;
  return (duracion * 0.034) / 2;
}


//
// SET UP
//
void setup()
{
  Serial.begin(115200);
  pinMode(BOTON_BUILTIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(BOTON_BUILTIN), Despertar, FALLING);

  // --- CONFIGURACIÓN SENSORES (ACTUALIZADO) ---
  pinMode(LDR_PIN, INPUT); // Configura el pin LDR como entrada
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // --- CONEXIÓN WIFI ---
  Serial.print("Conectando a WiFi ");
  Serial.println(ssid);
  WiFi.begin(ssid, password); // Conecta a la red (abierta)

  int intentos = 0;
  while(WiFi.status() != WL_CONNECTED && intentos < 20)
  {
    delay(500);
    Serial.print(".");
    intentos++;
  }

  if(WiFi.status() != WL_CONNECTED)
  {
    Serial.println("\nFallo conexión WiFi inicial");
    // NOTA: Si "Alumnos" es un Portal Cautivo, fallará aquí.
    EstadoActual = Reconectando;
  } else {
    Serial.println("\nWiFi Conectado");
  }

  //Conexión MQTT
  client.setServer(mqtt_server, mqtt_port);

  if(client.connect(mqtt_client_id)) // <-- Ahora 'mqtt_client_id' existe
  {
    Serial.println("MQTT conectado");
    EstadoActual = EsperandoSensor;
  }
  else
  {
    Serial.print("Fallo MQTT, rc=");
    Serial.println(client.state());
    EstadoActual = Reconectando;
  }

  iniciarTimerInactividad();
}
//
// LOOP
//
void loop()
{
  tiempoActual = millis();

  switch (EstadoActual)
  {
    case EsperandoSensor:
      // Mismo código de parpadeo...
      if(tiempoActual - ultimoTiempo >= 3000)
      {
        contadorEsperas++;
        ultimoTiempo = tiempoActual;
        Serial.print("Esperando... ");
        Serial.println(contadorEsperas);
      }
      if(contadorEsperas < 7) { 
        if(tiempoActual - controlParpadeo >= 500) {
          controlParpadeo = tiempoActual;
          digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        }
      }
      else { 
        if(tiempoActual - controlParpadeo >= 200) {
          controlParpadeo = tiempoActual;
          digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        }
      }

      // Mismo código de banderas (Dormir y Despertar)...
      if (hayQueMandarADormir)
      {
        contadorEsperas = 0;
        hayQueMandarADormir = false;
        PonerADormir = true;
        EstadoActual = Dormido;
        Serial.println("\nTiempo transcurrido, enviando micro a dormir...");
      }
      if(hayQueDespertar)
      {
        reiniciarTimerInactividad();
        contadorEsperas = 0;
        hayQueDespertar = false;
        Serial.println("\nReinicio del conteo para dormir, se mantiene el micro despierto");
      }

      // --- LECTURA DE SENSORES (ACTUALIZADO) ---
      static unsigned long ultimoCheckSensores = 0;
      if (tiempoActual - ultimoCheckSensores >= 3000) {
        ultimoCheckSensores = tiempoActual;
        
        // 1. Leer LDR
        valorLDR = analogRead(LDR_PIN);
        // NOTA: Este umbral depende de tu circuito (pull-up o pull-down)
        // Asumimos pull-down: más luz = valor más alto.
        if (valorLDR > UMBRAL_LUZ) {
            Serial.println("--- DISPARO LUZ (LDR) ---");
            reiniciarTimerInactividad(); // Hubo actividad
            EstadoActual = LeyendoLDR;
            break; 
        }

        // 2. Leer Ultrasonido
        distancia = leerUltrasonido();
        if (distancia > 0 && distancia < UMBRAL_DISTANCIA) {
            Serial.println("--- DISPARO ULTRASONIDO ---");
            reiniciarTimerInactividad(); // Hubo actividad
            EstadoActual = LeyendoUltrasonico;
            break;
        }
      }
      break;

    // --- ESTADOS DE MÁQUINA ACTUALIZADOS ---
    case LeyendoLDR:
      Serial.print("Leyendo LDR: ");
      Serial.println(valorLDR);
      EstadoActual = PublicandoTopicoLDR;
      break;

    case PublicandoTopicoLDR:
      Serial.println("Publicando en MQTT...");
      char payload_ldr[16];
      sprintf(payload_ldr, "%d", valorLDR); // Formato "1501"
      
      client.publish(Topico_LDR, payload_ldr);
      
      reiniciarTimerInactividad(); // Reinicia timer por actividad
      EstadoActual = EsperandoSensor;
      break;

    case LeyendoUltrasonico:
      Serial.print("Leyendo Distancia: ");
      Serial.println(distancia);
      EstadoActual = PublicandoTopicoUltrasonico;
      break;

    case PublicandoTopicoUltrasonico:
      Serial.println("Publicando en MQTT...");
      char payload_ultra[16];
      sprintf(payload_ultra, "%ld", distancia); // Formato "40"
      
      client.publish(Topico_UltraSonico, payload_ultra);
      
      reiniciarTimerInactividad(); // Reinicia timer por actividad
      EstadoActual = EsperandoSensor;
      break;

    // --- ESTADOS EXISTENTES (SIN CAMBIOS) ---
    case Reconectando:
      Serial.println(" ");
      Serial.println("Reconectando...");
      reiniciarTimerInactividad(); 
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED)
      {
        delay(500);
        Serial.print(".");
      }
      if(WiFi.status() == WL_CONNECTED) { Serial.println("Conectado a internet"); }
      while (!client.connected())
      {
        Serial.println("Intentando conexión MQTT...");
        if (client.connect(mqtt_client_id))
        {
          Serial.println("Conectado.");
          EstadoActual = EsperandoSensor;
          reiniciarTimerInactividad();
        }
        else
        {
          Serial.print(" falló, rc=");
          Serial.print(client.state());
          Serial.println(" -> nuevo intento en 2s");
          delay(2000);
          reiniciarTimerInactividad();
        }
      }
      break;
    
    case Dormido:
      if(PonerADormir)
      {
        PonerADormir = false;
        contadorDurmiendo = 0;
        timerInactividad.detach();
        Serial.println("Entrando en modo Dormido (Modem Sleep)...");
        WiFi.setSleep(true);
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
      }
      if(hayQueDespertar)
      {
        hayQueDespertar = false;
        contadorDurmiendo = 0;
        Serial.println("Botón detectado -> Despertando...");
        WiFi.mode(WIFI_STA);
        reiniciarTimerInactividad();
        EstadoActual = EsperandoSensor;
      }
      if (millis() - controlDormir >= 3000)
      {
        contadorDurmiendo++;
        controlDormir = millis();
        Serial.print("Durmiendo... ");
        Serial.println(contadorDurmiendo);
      }
      break;
  }

  // Verificación de conexión (fuera del switch)
  if(EstadoActual != Dormido && !hayQueMandarADormir)
  {
    if(WiFi.status() != WL_CONNECTED || !client.connected())
    {
      EstadoActual = Reconectando;
    }
  }

  // El client.loop() es crucial
  if(EstadoActual != Dormido) {
    client.loop();
  }
}