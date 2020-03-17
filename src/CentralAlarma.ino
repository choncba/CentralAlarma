//////////////////////////////////////////////////////////////////////////////////////
//        Central de Alarma con conectividad MQTT a través de ESP-Link              //
//                                                                                  //
//    Hardware:                                                                     //
//            - ATMEGA1284P                                                         //
//            - ESP8266 - ESP01 4MBIT                                               //
//            - Módulo GSM SIM800L                                                  //
//            - DS18B20                                                             //
//            - DHT22                                                               //
//            - LCR                                                                 //
//            - Módulos DC/DC para carga de Batería 12v y regulación +5v            //
//            - Salidas a sirenas con MOSFET                                        //
//            - Entradas para llavero RF con PT2272-L4                              //
//            - Modulo de medicion de Potencia AC                                   //
//    Esquematico: https://easyeda.com/editor#id=|d1a352035b8044ac87127abeea01cdcf  //
//                                                                                  //
//    Software:                                                                     //
//            - ATMEGA1284: Mighty Core (cargar bootloader desde Arduino usando la  //
//                          opcion USBASP (Mighty Core))                            //
//                          ELClient para protocolo SLIP y MQTT                     //
//                          FastSIM - Modificacion de SimSMS para que funcione sin  //
//                          delay
//            - ESP8266: esp-link v3.0.14-g963ffbb                                  //
//            - Controller: HomeAssistant - Ver configuracion al final              //
// Panel de alarma HA:                                                              //
// https://www.home-assistant.io/components/alarm_control_panel.mqtt/               //
// Envia los comandos: 'DISARM', 'ARM_HOME', 'ARM_AWAY' desde el front end          //
//                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////
//  Script para subir el sketch con avrdude:
//  C:\Program Files (x86)\Arduino\hardware\tools\avr\bin>avrdude -DV -patmega1284p -Pnet:192.168.1.20:23 -carduino -b115200 -U flash:w:C:\Users\ChoN\OneDrive\Platform\CentralAlarma\.pioenvs\mightycore1284\firmware.hex:i -C C:\Users\ChoN\AppData\Local\Arduino15\packages\MightyCore\hardware\avr\2.0.1\avrdude.conf
//////////////////////////////////////////////////////////////////////////////////////

// https://github.com/MCUdude/MightyCore#pinout
// Setup: Definicion de pines Standard
//      Nombre  Arduino   PIN
#define LED     0         // PB0
#define OW_PIN  1         // PB1
#define VAC     2         // PB2
#define NA1     3         // PB3
#define NA2     4         // PB4
#define pwrRX   5         // PB5
#define pwrTX   6         // PB6
#define SIM_RES 7         // PB7
#define RX0     8         // PD0
#define TX0     9         // PD1
#define RX1     10        // PD2
#define TX1     11        // PD3
#define SIR2    12        // PD4
#define SIR1    13        // PD5
#define LED_EXT 14        // PD6
#define DHT_PIN 15        // PD7
#define IN1     16        // PC0
#define IN2     17        // PC1
#define IN3     18        // PC2
#define IN4     19        // PC3
#define IN5     20        // PC4
#define IN6     21        // PC5
#define IN7     22        // PC6
#define IN8     23        // PC7
#define VBAT    24        // PA0
#define RFD     25        // PA1
#define RFC     26        // PA2
#define RFB     27        // PA3
#define RFA     28        // PA4
#define NA5     29        // PA5
#define NA6     30        // PA6
#define LCR     31        // PA7

#define USE_GSM         // Modulo GSM SIM800L
#define USE_SENSORES    // Habilita/Deshabilita DHT22, 18B20 y LCR
#define USE_RF          // Habilita/Deshabilita entradas RF
#define USE_POWER_METER // Habilita/Deshabilita medidor de potencia

#include <ELClient.h>
#include <ELClientMqtt.h>

ELClient esp(&Serial);
ELClientMqtt mqtt(&esp);

bool connected;
#define RETAIN  true // Flag retain para publicaciones mqtt
#define QoS     0

#ifdef USE_SENSORES
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define DHTTYPE DHT22
DHT_Unified dht(DHT_PIN, DHTTYPE);

#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(OW_PIN);
DallasTemperature sensors(&oneWire);
#endif

#ifdef USE_GSM
#include <FastSIM.h>
#define PIN "1234"        // PIN incluído en los SMS para habilitar comandos
FastSIM GSM;              // Inicializo librería SIM_SMS
#endif

#ifdef USE_POWER_METER
#include <PZEM004T.h>         // Libreria de manejo del modulo medidor de potencia, usa softwareserial
PZEM004T pzem(pwrRX, pwrTX);  // (RX,TX) connect to TX,RX of PZEM
IPAddress ip(192,168,1,1);
#endif

#include "Timer.h"  // Ver MightyCore
#ifdef USE_GSM
Timer t_gsm, t_leds, t_sensores, t_mqtt;
#else
Timer t_leds, t_sensores, t_mqtt;
#endif

int timerLEDS;
const unsigned long unSegundo = 1000L;  // 1000 mSeg
const unsigned long treintaSegundos = 30 * unSegundo; // 30000 ms / 30 seg
const unsigned long unMinuto = 60 * unSegundo; // 60000 ms / 1 min
const unsigned long diezMinutos = 10 * unMinuto; // 600000 ms/10 min
const unsigned long treintaMinutos = 30 * unMinuto; // 600000 ms/10 min

#pragma region Variables Globales
// Entradas Alarma
#define INPUTS 8
const uint8_t Entrada[INPUTS] = { IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8 };
enum Entrada_enum {in1 = 0, in2, in3, in4, in5, in6, in7, in8 };
const char Entrada_txt[INPUTS][4] = { "in1", "in2", "in3", "in4", "in5", "in6", "in7", "in8" };

#define NUM_STATUS 5
enum Alarm_Status_enum { DISARMED=0, ARMED_HOME, ARMED_AWAY, PENDING, TRIGGERED };
const char Alarm_Status_txt[NUM_STATUS][11] = { "disarmed", "armed_home", "armed_away", "pending", "triggered"};

// Salidas
#define OUTPUTS 3
const uint8_t Salida[OUTPUTS] = { SIR1, SIR2, LED_EXT };
enum Salidas_enum { SIRENA1 = 0, SIRENA2, LED_CAMPANA };
const char Salida_txt [OUTPUTS][11] = { "sirena_ext", "sirena_int", "led_ext"};
// Entradas RF
#define RF_INPUTS 4
const uint8_t RfIN[RF_INPUTS] = { RFA, RFB, RFC, RFD };
enum teclas_rf { TECLA_A = 0, TECLA_B, TECLA_C, TECLA_D };
const char TeclaRf_txt [RF_INPUTS][8] = { "tecla_a", "tecla_b", "tecla_c", "tecla_d" };

struct Status {
	uint8_t Entrada[INPUTS] = { 1,1,1,1,1,1,1,1 };
  uint8_t AlarmStatus = 0;
  uint8_t AlarmNextStatus = 0;
  uint8_t TriggerCause = 0;
	bool Vac = 0;
  float Vbat = 0;
  float OutsideTemp = 0;
  float OutsideHum = 0;
  float InsideTemp = 0;
  uint8_t LumExt = 0;
  uint8_t RFin[RF_INPUTS] = { 1,1,1,1 };
  bool GsmStatus;
#ifdef USE_POWER_METER
  float voltage;
  float current;
  float power;
  float energy;
#endif
}Central;

const char NombreTopic[] = {"/CentralAlarma"};
String StringTopic = "";
char CharTopic[100];
String StringData = "";
char CharData[10];

#ifdef USE_GSM 
char telefono[20];
char texto[100];
int id_timerAviso = 0;
#endif
#pragma endregion

#pragma region Lectura de entradas y sensores
#ifdef USE_SENSORES
void LeerSensores(void* context)
{
  (void)context;
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if(!isnan(event.temperature)) Central.OutsideTemp = event.temperature;

  dht.humidity().getEvent(&event);
  if(!isnan(event.relative_humidity)) Central.OutsideHum = event.relative_humidity;

  sensors.requestTemperatures();
  float int_temp = sensors.getTempCByIndex(0);
  if(!isnan(int_temp))  Central.InsideTemp = int_temp;

  Central.LumExt = 0;
  int lectura_vbat=0;
  int lectura_lcr = 0;
  for(int i=0; i<10; i++)
  {
    lectura_lcr += analogRead(LCR);
    lectura_vbat += analogRead(VBAT);
    delay(10);
  }
  lectura_lcr = lectura_lcr/10;
  Central.LumExt = map(lectura_lcr, 0, 1023, 100, 0);
  lectura_vbat = lectura_vbat/10;
  Central.Vbat = lectura_vbat * 0.01967;  // Vbat = Vad.(R1+R2)/R2; Vad=Lectura.5/1024 => Vbat = Lectura.(R1+R2)/R2).(5/1024)
  Central.Vac = digitalRead(VAC);

#ifdef USE_POWER_METER
  float v_med = pzem.voltage(ip);
  if (v_med > 0.0) Central.voltage = v_med;

  float i_med = pzem.current(ip);
  if(i_med >= 0.0) Central.current = i_med;
  
  float p_med = pzem.power(ip);
  if(p_med >= 0.0) Central.power = p_med;
  
  float e_med = pzem.energy(ip);
  if(e_med >= 0.0) Central.energy = e_med;
#endif
}
#endif

void CheckEntradas(){
  static uint8_t RoundCheck[INPUTS] = { 0,0,0,0,0,0,0,0 };
  const uint8_t RoundCheckThreshole = 8;

  for (uint8_t cnt = 0; cnt < INPUTS; cnt++)
  {
    uint8_t curStatus = digitalRead(Entrada[cnt]);
    if (Central.Entrada[cnt] != curStatus)
    {
      delay(5);
      curStatus = digitalRead(Entrada[cnt]);
      if (Central.Entrada[cnt] != curStatus)
      {
        RoundCheck[cnt]++;
      }
      else  RoundCheck[cnt] = 0;

      if (RoundCheck[cnt] >= RoundCheckThreshole)
      {
        Central.Entrada[cnt] = curStatus;
        RoundCheck[cnt] = 0;
        if(Central.AlarmStatus == ARMED_HOME) // Verifico si la alarma está activada en casa
        {
          if(cnt < 6) // Reservo las entradas 7 y 8 para sensores dentro de la casa cuando estoy en ARMED_HOME
          {
            Central.AlarmNextStatus = TRIGGERED;  // Disparo la alarma con entradas 1 a 6
            Central.TriggerCause = cnt;           // Guardo que entrada causo el disparo
          }
        }
        if(Central.AlarmStatus == ARMED_AWAY)
        {
          Central.AlarmNextStatus = TRIGGERED;    // Disparo la alarma
          Central.TriggerCause = cnt;             // Guardo que entrada causo el disparo
        }
        PublicarEntrada(cnt);
      }
    }
  }
}

#ifdef USE_RF
void CheckRF(){
  static uint8_t RoundCheck[RF_INPUTS] = { 0,0,0,0 };
  const uint8_t RoundCheckThreshole = 8;

  for (uint8_t cnt = 0; cnt < RF_INPUTS; cnt++)
  {
    uint8_t curStatus = digitalRead(RfIN[cnt]);
    if (Central.RFin[cnt] != curStatus)
    {
      delay(5);
      curStatus = digitalRead(RfIN[cnt]);
      if (Central.RFin[cnt] != curStatus)
      {
        RoundCheck[cnt]++;
      }
      else  RoundCheck[cnt] = 0;

      if (RoundCheck[cnt] >= RoundCheckThreshole)
      {
        Central.RFin[cnt] = curStatus;
        RoundCheck[cnt] = 0;
        switch(cnt)
        {
          case TECLA_A: Central.AlarmNextStatus = ARMED_AWAY;
                        break;
          case TECLA_B: Central.AlarmNextStatus = ARMED_HOME;
                        break;
          case TECLA_C: Central.AlarmNextStatus = DISARMED;
                        break;
          case TECLA_D:
                        break;
          default:      break;
        }
        PublicarRF(cnt);
      }
    }
  }
}
#endif
#pragma endregion

#pragma region Funciones MQTT
// Callback made from esp-link to notify of wifi status changes
void wifiCb(void* response) {
  ELClientResponse *res = (ELClientResponse*)response;
  if (res->argc() == 1) {
    uint8_t status;
    res->popArg(&status, 1);

    if(status == STATION_GOT_IP) {
      Serial.println(F("WIFI CONNECTED"));
    } else {
      Serial.print(F("WIFI NOT READY: "));
      Serial.println(status);
    }
  }
}

// Callback when MQTT is connected
void mqttConnected(void* response) {
  Serial.println(F("MQTT connected!"));
  connected = true;

  StringTopic = String(NombreTopic) + "/set";
  StringTopic.toCharArray(CharTopic, sizeof(CharTopic));
  mqtt.subscribe(CharTopic);

  StringTopic = String(NombreTopic) + "/status/LWT";
  StringTopic.toCharArray(CharTopic, sizeof(CharTopic));
  mqtt.publish(CharTopic,"online",QoS,RETAIN);

#ifdef USE_SENSORES
  LeerSensores((void*)0);
#endif  

  PublicarTodo((void*)0);
}

// Callback when MQTT is disconnected
void mqttDisconnected(void* response) {
  Serial.println(F("MQTT disconnected"));
  connected = false;
}

// Callback when an MQTT message arrives for one of our subscriptions
void mqttData(void* response) {
  ELClientResponse *res = (ELClientResponse *)response;

  Serial.print(F("Received: topic="));
  String topic = res->popString();
  Serial.println(topic);

  Serial.print(F("data="));
  String data = res->popString();
  Serial.println(data);

  if(topic.indexOf("/set")>0)  // Si el topic contiene "/set"
  {
    if(data.equals("DISARM"))
    {
      Central.AlarmNextStatus = DISARMED;
      Serial.println("Alarma desactivada via MQTT");
    }
    if(data.equals("ARM_HOME"))
    {
      Central.AlarmNextStatus = ARMED_HOME;
      Serial.println("Alarma Activada en casa via MQTT");
    } 
    if(data.equals("ARM_AWAY"))
    {
      Central.AlarmNextStatus = ARMED_AWAY;
      Serial.println("Alarma Activada Fuera de casa via MQTT");
    }
  }
}

void mqttPublished(void* response) {
  Serial.println(F("MQTT published"));
}
#pragma endregion

#pragma region Publicaciones MQTT
void PublicarSensores(void* context)
{
  (void)context;
  if(connected)
  {
    StringTopic = String(NombreTopic) + "/temp_int";
    StringTopic.toCharArray(CharTopic, sizeof(CharTopic));
    StringData = Central.InsideTemp;
    StringData.toCharArray(CharData, sizeof(CharData));
    mqtt.publish(CharTopic, CharData,QoS,RETAIN);

    StringTopic = String(NombreTopic) + "/temp_ext";
    StringTopic.toCharArray(CharTopic, sizeof(CharTopic));
    StringData = Central.OutsideTemp;
    StringData.toCharArray(CharData, sizeof(CharData));
    mqtt.publish(CharTopic, CharData,QoS,RETAIN);

    StringTopic = String(NombreTopic) + "/hum_ext";
    StringTopic.toCharArray(CharTopic, sizeof(CharTopic));
    StringData = Central.OutsideHum;
    StringData.toCharArray(CharData, sizeof(CharData));
    mqtt.publish(CharTopic, CharData,QoS,RETAIN);

    StringTopic = String(NombreTopic) + "/lcr";
    StringTopic.toCharArray(CharTopic, sizeof(CharTopic));
    StringData = Central.LumExt;
    StringData.toCharArray(CharData, sizeof(CharData));
    mqtt.publish(CharTopic, CharData,QoS,RETAIN);

    StringTopic = String(NombreTopic) + "/vbat";
    StringTopic.toCharArray(CharTopic, sizeof(CharTopic));
    StringData = Central.Vbat;
    StringData.toCharArray(CharData, sizeof(CharData));
    mqtt.publish(CharTopic, CharData,QoS,RETAIN);

    StringTopic = String(NombreTopic) + "/vac";
    StringTopic.toCharArray(CharTopic, sizeof(CharTopic));
    mqtt.publish(CharTopic, (Central.Vac)?"0":"1",QoS,RETAIN);

    StringTopic = String(NombreTopic) + "/voltage";
    StringTopic.toCharArray(CharTopic, sizeof(CharTopic));
    StringData = Central.voltage;
    StringData.toCharArray(CharData, sizeof(CharData));
    mqtt.publish(CharTopic, CharData,QoS,RETAIN);

    StringTopic = String(NombreTopic) + "/current";
    StringTopic.toCharArray(CharTopic, sizeof(CharTopic));
    StringData = Central.current;
    StringData.toCharArray(CharData, sizeof(CharData));
    mqtt.publish(CharTopic, CharData,QoS,RETAIN);

    StringTopic = String(NombreTopic) + "/power";
    StringTopic.toCharArray(CharTopic, sizeof(CharTopic));
    StringData = Central.power;
    StringData.toCharArray(CharData, sizeof(CharData));
    mqtt.publish(CharTopic, CharData,QoS,RETAIN);

    StringTopic = String(NombreTopic) + "/energy";
    StringTopic.toCharArray(CharTopic, sizeof(CharTopic));
    StringData = Central.energy;
    StringData.toCharArray(CharData, sizeof(CharData));
    mqtt.publish(CharTopic, CharData,QoS,RETAIN);
  }
}

void PublicarEntrada(uint8_t num_entrada)
{
  if(connected)
  {
    StringTopic = String(NombreTopic) + "/" + String(Entrada_txt[num_entrada]);
    StringTopic.toCharArray(CharTopic, sizeof(CharTopic));
    mqtt.publish(CharTopic, (Central.Entrada[num_entrada])?"0":"1",QoS,RETAIN);
  }
}

void PublicarAlarma()
{
  if(connected)
  {
    StringTopic = String(NombreTopic) + "/status";
    StringTopic.toCharArray(CharTopic, sizeof(CharTopic));
    mqtt.publish(CharTopic, Alarm_Status_txt[Central.AlarmStatus],QoS,RETAIN);
    if(Central.AlarmStatus == TRIGGERED)
    {
      StringTopic = String(NombreTopic) + "/status/cause";
      StringTopic.toCharArray(CharTopic, sizeof(CharTopic));
      mqtt.publish(CharTopic, Entrada_txt[Central.TriggerCause],QoS,RETAIN);
    }
  }
}

#ifdef USE_RF
void PublicarRF(uint8_t num_entrada)
{
  if(connected)
  {
    StringTopic = String(NombreTopic) + "/" + String(TeclaRf_txt[num_entrada]);
    StringTopic.toCharArray(CharTopic, sizeof(CharTopic));
    mqtt.publish(CharTopic, (Central.RFin[num_entrada])?"0":"1",QoS,RETAIN);
  }
}
#endif

void PublicarTodo(void* context)
{
  (void)context;
  uint8_t i;
  PublicarAlarma();
  PublicarSensores((void*)0);
  for(i = 0; i<INPUTS; i++)
  {
    PublicarEntrada(i);
  }
#ifdef USE_RF
  for(i = 0; i<RF_INPUTS; i++)
  {
    PublicarRF(i);
  }
#endif
}
#pragma endregion

#pragma region SETUP
void setup() {
  pinMode(LED, OUTPUT);
  pinMode(SIM_RES, OUTPUT);
  pinMode(SIR1, OUTPUT);
  pinMode(SIR2, OUTPUT);
  pinMode(LED_EXT, OUTPUT);
  pinMode(IN1, INPUT_PULLUP);
  pinMode(IN2, INPUT_PULLUP);
  pinMode(IN3, INPUT_PULLUP);
  pinMode(IN4, INPUT_PULLUP);
  pinMode(IN5, INPUT_PULLUP);
  pinMode(IN6, INPUT_PULLUP);
  pinMode(IN7, INPUT_PULLUP);
  pinMode(IN8, INPUT_PULLUP);
  pinMode(RFA, INPUT);
  pinMode(RFB, INPUT);
  pinMode(RFC, INPUT);
  pinMode(RFD, INPUT);

  Serial.begin(115200);
  
  Serial.println(F("EL-Client starting!"));
  // Sync-up with esp-link, this is required at the start of any sketch and initializes the
  // callbacks to the wifi status change callback. The callback gets called with the initial
  // status right after Sync() below completes.
  esp.wifiCb.attach(wifiCb); // wifi status change callback, optional (delete if not desired)
  bool ok;
  do {
    ok = esp.Sync();      // sync up with esp-link, blocks for up to 2 seconds
    if (!ok) Serial.println(F("EL-Client sync failed!"));
  } while(!ok);
  Serial.println(F("EL-Client synced!"));

  // Set-up callbacks for events and initialize with es-link.
  mqtt.connectedCb.attach(mqttConnected);
  mqtt.disconnectedCb.attach(mqttDisconnected);
  mqtt.publishedCb.attach(mqttPublished);
  mqtt.dataCb.attach(mqttData);
  mqtt.setup();
  mqtt.lwt("/CentralAlarma/status/LWT","offline",QoS,RETAIN);

  Serial.println(F("EL-MQTT ready"));

#ifdef USE_POWER_METER
pzem.setAddress(ip);
#endif

#ifdef USE_SENSORES
  dht.begin();
  sensors.begin();
  t_sensores.every(unMinuto, LeerSensores, (void*)0);        // Lee la informacion de sonsores cada 1 minuto 
  t_mqtt.every(diezMinutos, PublicarSensores, (void*)0); // Publica la informacion de sensores cada 10 minutos 
#endif

#ifdef USE_GSM
  digitalWrite(SIM_RES, HIGH);
  Serial1.begin(SIM_BAUD);
  t_gsm.every(unMinuto, CheckGSM, (void*)0);
#endif

  t_mqtt.every(treintaMinutos, PublicarTodo, (void*)0);  // Publica todo cada media hora
}
#pragma endregion

#pragma region LOOP
void loop() {
#ifdef USE_GSM
  if(GSM.update() == IDLE)
  {
    esp.Process();
  }
  else
  {
    ControlGSM();
  }
#else
  esp.Process();
#endif

  CheckEntradas();

#ifdef USE_RF
  CheckRF();
#endif

#ifdef USE_GSM
  t_gsm.update();
#endif

  t_leds.update();
  t_mqtt.update();
  t_sensores.update();

  if(Central.AlarmStatus != Central.AlarmNextStatus)
  {
    switch(Central.AlarmNextStatus)
    {
      case DISARMED:    t_leds.stop(timerLEDS);
                        digitalWrite(LED, LOW);
                        digitalWrite(LED_EXT, LOW);
                        ActivarSirenas(LOW);
                        if(Central.AlarmStatus == ARMED_AWAY)
                        {
                          beep(); // Suenan las sirenas durante 100 mSeg
                        }
#ifdef USE_GSM          
                        id_timerAviso = 0;
                        t_gsm.stop(id_timerAviso);
#endif
                        break;
      case ARMED_HOME:  t_leds.stop(timerLEDS);
                        timerLEDS = t_leds.every(1000, BlinkLeds, (void*)0); // Parpadean los leds cada 1 segundo
                        break;
      case ARMED_AWAY:  t_leds.stop(timerLEDS);
                        timerLEDS = t_leds.every(1000, BlinkLeds, (void*)0); // Parpadean los leds cada 1 segundo
                        beep(); // Suenan las sirenas durante 100 mSeg
                        break;
      case PENDING:     break;
      case TRIGGERED:   ActivarSirenas(HIGH);
                        t_leds.stop(timerLEDS);
                        timerLEDS = t_leds.every(100, BlinkLeds, (void*)0); // Parpadean los leds cada 100 mSseg
#ifdef USE_GSM
                        if(id_timerAviso == 0)
                        {
                          id_timerAviso = t_gsm.after(unMinuto, AvisoGSM, (void*)0); // Manda SMS luego de un minuto
                        }
#endif
                        break;
      default:          break;      
    }
    Central.AlarmStatus = Central.AlarmNextStatus;
    PublicarAlarma();
  }
  
}
#pragma endregion

void ActivarSirenas(uint8_t modo)
{
  digitalWrite(Salida[SIRENA1], modo);
  digitalWrite(Salida[SIRENA2], modo);
}

void BlinkLeds(void* context)
{
  (void) context;
  digitalWrite(LED,!digitalRead(LED));
  digitalWrite(LED_EXT,!digitalRead(LED_EXT));
}

void beep()
{
  // int pulso1 = t_leds.pulseImmediate(Salida[SIRENA1],100, HIGH);  //Produce un pulso en alto de 100 mSeg y luego queda en bajo
  // int pulso2 = t_leds.pulseImmediate(Salida[SIRENA2],100, HIGH);
  t_leds.pulseImmediate(Salida[SIRENA1],100, HIGH);  //Produce un pulso en alto de 100 mSeg y luego queda en bajo
  t_leds.pulseImmediate(Salida[SIRENA2],100, HIGH);
}

#pragma region Funciones GSM
#ifdef USE_GSM

void CheckGSM(void * context)
{
  (void)context;
  GSM.CheckOK();
}

void ControlGSM()
{
  switch(GSM.update())
	{
    case SMS_RECEIVED:	GSM.GetSMS(telefono, texto, 20);
                        Serial.print("Mensaje recibido de ");
                        Serial.print(telefono);
                        Serial.print(": ");
                        Serial.println(texto);
                        ProcesarComando(texto);
                        break;
		case SMS_SEND:		  if(GSM.SendSMS(telefono, texto))
                        {
                          Serial.println("Mensaje enviado");
                        }
                        break;
		case CHECK_OK:		  Central.GsmStatus = GSM.CheckOK();  // Revisar esto
                        break;
		default:			      break;
	}
}

void ProcesarComando(const char *msj)
{
	String msg = "";
	
	if (GSM.StringFind(msj, PIN))	// PIN de identificacion
	{
		if (GSM.StringFind(msj, " ACTIVAR"))
		{
			Central.AlarmNextStatus = ARMED_AWAY;
			msg = "Alarma Activada";
      msg.toCharArray(texto, sizeof(texto));
      GSM.SendSMS(telefono, texto);
			Serial.println(F("Activando alarma por GSM"));	
		}
		if (GSM.StringFind(msj, " DESACTIVAR"))
		{
			Central.AlarmNextStatus = DISARMED;
      msg = "Alarma Desactivada";
      msg.toCharArray(texto, sizeof(texto));
			GSM.SendSMS(telefono, texto);
			Serial.println(F("Desactivando Alarma por GSM"));
		}
		if (GSM.StringFind(msj, " INFO"))
		{
			msg = "Alarma ";
			msg += (Central.AlarmStatus == ARMED_AWAY) ? "ACTIVADA" : "DESACTIVADA";
			msg += ", Entradas: ";
			for (int i = 0; i < INPUTS; i++)
			{
				msg += String(i+1);
				msg += " ";
				msg += (Central.Entrada[i]) ? "ON" : "OFF";
				msg += ", ";
			}
			msg += "220VAC ";
			msg += (!Central.Vac) ? "OK" : "ERROR";
			msg.toCharArray(texto, sizeof(texto));
			
			GSM.SendSMS(telefono, texto);
		}
	}
	else
	{
		Serial.println(F("Error de PIN, o no es un comando"));
	}
}

void AvisoGSM(void * context)
{
  (void)context;
  String aux = "3516510632";
  aux.toCharArray(telefono, sizeof(telefono));
  aux = "ALARMA DISPARADA! Motivo: Entrada " + String(Central.TriggerCause + 1);
  aux.toCharArray(texto, sizeof(texto));
  GSM.SendSMS(telefono, texto); // Poner !!! TILDA TODO
	//GSM.SendSMS("3512679336", "ALARMA DISPARADA!");			// Cel Pilu
	//GSM.SendSMS("3516878690", "ALARMA CASA LU DISPARADA!"); // Cel Patri
}
#endif
#pragma endregion
