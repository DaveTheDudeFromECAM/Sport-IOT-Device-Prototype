# Feather M0 Code
Dans cette section, nous allons prendre le temps de passer en revue le code et expliquer l'utilisté de diverses fonctions et variables.
### Librairies
Nous employons un acceleromètre XXX pour détecter les chues, ce dernier est connecté en I²C à notre board.
```cpp
#include "LIS3DHTR.h"
#include <Wire.h>
LIS3DHTR<TwoWire> LIS;
#define WIRE Wire
```
Le cepteur de saturation en oxygène et rythme cardiaque est le XXXX.
```cpp
#include <Arduino.h>
#include <SPI.h>
#include "algorithm_by_RF.h"
#include "algorithm.h"
#include "max30102.h"
```
Le LORA est employé pour communiquer entre la board et une passerelle TTN (The Things Network).
```cpp
#include <lmic.h>
#include <hal/hal.h>
```
### Definitions
Variables qui serviront à le détection de chuttes. Les lettres majuscules contiennesnt les valeur actuelles de l'acceleromètre et les minucules l'ancienne valeur. Les deltas permettent de quantifier les fluctuations du capteur.
```cpp
int led = 13;                             // Integrated LED indicator
float x, y, z, X, Y, Z  = 0;              // Capital letter = actual value, small letter = old value
float deltaX, deltaY, deltaZ = 0;
int fall = 0;                             // Data sent to TTN gateway; 1 if fall detected
```
Configuration de la communication LORA avec TTN.
```cpp
// EUI -> little-endian format, LSB first. If copying EUI from ttnctl output -> reverse the bytes.
// TTN issued EUIs -> last bytes should be 0xD5, 0xB3,0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// Also in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x3E, 0x97, 0x04, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}p^))))))))))))))))))))))))))))))))))))))))))))))))))))))))))

// Key should be in big endian format. Or, since it is not really a number but a block of memory, endianness does 
// not really apply. In practice, a key taken from the TTN console can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = {0x89, 0xC3, 0x14, 0x49, 0x73, 0x84, 0x35, 0x42, 
                                        0x8B, 0xA0, 0x74, 0x3F, 0xEC, 0xF2, 0xEE, 0xDE };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
static osjob_t sendjob;
```
Payload est la variable qui contient les éléments à transmettre, elle fait 7 bytes. La fréquence cardiaque, la saturation en oxygène, la temperature et 
```cpp
static uint8_t payload[7];                // Payload sent to TTN gateway
```
Variable pour instaurer le temps entre deux transmissions LORA, exprumé en secondes.
```cpp
const unsigned TX_INTERVAL = 1;           // TX every x seconds (might be longer due to duty cycle limitations).
```
Configuration des la board pour la communication LORA
```cpp
const lmic_pinmap lmic_pins = {           // LoRa Pin mapping. Pin 6 & DIO1 have to be connected
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,                         // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};
```
Gestion des evenements pour la communication LORA, obtension des informations sur la connexion. Désactivation des checks de validation pour gagner en performances.
```cpp
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_JOINED:
          Serial.println(F("EV_JOINED"));
          {
            u4_t netid = 0;
            devaddr_t devaddr = 0;
            u1_t nwkKey[16];
            u1_t artKey[16];
            LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
            
            Serial.print("netid: ");
            Serial.println(netid, DEC);
            
            Serial.print("devaddr: ");
            Serial.println(devaddr, HEX);
            
            Serial.print("AppSKey: ");
            for (size_t i=0; i<sizeof(artKey); ++i) {
              if (i != 0)
                Serial.print("-");
              printHex2(artKey[i]);
            }
              
            Serial.println("NwkSKey: ");
            for (size_t i=0; i<sizeof(nwkKey); ++i) {
              if (i != 0)
                Serial.print("-");
              printHex2(nwkKey[i]);
            }
            Serial.println();
          }
            
            LMIC_setLinkCheckMode(0);       // Disable link check validation
            break;
            
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE "));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);  
            break;
            
        default:
           Serial.print(F("Unknown event: "));
           Serial.println((unsigned) ev);
           break;
    }                                       // end of switch case
}  
```

```cpp
void do_send(osjob_t* j){
    if (LMIC.opmode & OP_TXRXPEND) {        // = TX/RX job running?
       Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
```
Acquisition des données de fréquance cardiaque et saturation oxygène. Réalisation de 100 mesures placées dans la FIFO du capteur, puis recupérées et manipul&es par la board et l'algorithme fourni par MAXIM.
```cpp
        /*HEARTBEAT DATA*/
        float n_spo2,ratio,correl;          // SPO2 value
        int8_t ch_spo2_valid;               // Show if the SPO2 is valid
        int32_t n_heart_rate;               // HR value
        int8_t  ch_hr_valid;                // Show if the HR is valid
        int32_t i;
        
        for(i=0;i<BUFFER_SIZE;i++)          // 100 samples
        {
          while(digitalRead(oxiInt)==1);    // HEARTBEAT interrupt pin asserts
          maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i)); //read from MAX30102 FIFO
        }
        //Get HR & SpO2 after BUFFER_SIZE samples(ST seconds of samples)
        rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, 
                                            BUFFER_SIZE, 
                                            aun_red_buffer, 
                                            &n_spo2, 
                                            &ch_spo2_valid, 
                                            &n_heart_rate, 
                                            &ch_hr_valid, 
                                            &ratio, 
                                            &correl); 
```
Acquisition du capteur acceleromètre, détection d'une grande variatoin sur un des trois axs, X, Y et Z. si c'esr le cas, on met le flag de chutte à 1 sinon 0.
```cpp
        /*FALL DETECTION DATA*/
        LIS.getAcceleration(&X, &Y, &Z);
        deltaX, deltaY, deltaZ = X-x, Y-y, Z-z;
        x, y, z = X, Y, Z;
        if(deltaX > 0.9 || deltaY > 0.9 || deltaZ > 0.9){fall = 1;} // 0.9 value was found by testing
        else{fall = 0;}
```
Acquisitoin de la température depuis le capteur MAXIMM.
```cpp
        /*TEMPERATURE DATA*/
        int8_t integer_temperature;
        uint8_t fractional_temperature;
        maxim_max30102_read_temperature(&integer_temperature, &fractional_temperature);
        float temperature = integer_temperature + ((float)fractional_temperature)/16.0;

```
La payload sera constitué de toutes nos valeurs récupérées sur nos capteur et converties sur deux bytes sauf pour le flag de chutte.
```cpp
        // PAYLOAD CREATION
        int shiftTEMP = int(temperature); 
        payload[0] = byte(shiftTEMP);
        payload[1] = shiftTEMP >>8;
        
        int shiftHR = int(n_heart_rate); 
        payload[2] = byte(shiftHR);
        payload[3] = shiftHR >>8;        
        
        int shiftSPO = int(n_spo2); 
        payload[4] = byte(shiftSPO);
        payload[5] = shiftSPO >>8;        
        
        payload[6] = byte(fall);
        
        LMIC_setTxData2(1, payload, sizeof(payload)-1, 0); 
```
        
### Setup
Extinction de la led interne permet de diminuer la consommation electrique de la board et par conséquant allonger la durée de fonctionnement sur batteries.
```cpp
  pinMode(led,OUTPUT);
  digitalWrite(led,LOW);                    // Turn off internal LED to save power
```
Initialisation du bus I²C à une fréquence de transmissoin de 10Hz et en mode basse puissance.
```cpp
  /*FALL DETECTION*/
  LIS.begin(WIRE, 0x19);                    // I²C init
  delay(100);
   
  LIS.setOutputDataRate(LIS3DHTR_DATARATE_10HZ);
  LIS.setPowerMode(POWER_MODE_LOW);         //Low-Power enable
```
configuration de la Pin d'interruption du capteur et initialisation de ce dernier.
```cpp
/*HEARTBEAT*/
  pinMode(oxiInt, INPUT);                   // pin D10 -> MAX30102 interrupt
  maxim_max30102_init();                    // initialize the MAX30102
```
établissement de la communication série à 9600 baud, initialisation de LORA, configuratoin du Spreading factor, lié à la puissance et consommatoin lors de la communicatoin et demmarage de sa tâche principale.
```cpp
/*LORA*/
  delay(3000);
  while (! Serial);
  Serial.begin(9600);
  Serial.println(F("Starting"));
  os_init();                                // LMIC init
  
  LMIC_reset();                             // Reset MAC state->Session & pending data transfers discarded
  
  LMIC_setLinkCheckMode(0);                 // Link-check mode & ADR OFF-> ADR complicate testing
  
  LMIC_setDrTxpow(DR_SF7,14);               // Set data rate to Spreading Factor 7,max supported rate for 125kHz 
                                            // channels, minimizes air time & batt power.
  
  do_send(&sendjob);                        //Start job(sending automatically starts OTAA too)
```
Réalisation des tâches citées plus haut 
```cpp
void loop() {
  os_runloop_once();
}
```
