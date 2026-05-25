#pragma once
#include <Arduino.h>
#include <LittleFS.h>
#include "wifi_manager.h"

// ============================================================
// 1. ACCESS POINT (AP) KONFIGURATION (STATISCH)
// ============================================================
#define AP_SSID "AutopilotSetup"  // Name des Einrichtungs-WLANs
#define AP_PASSWORD "auto1234"    // Passwort des Einrichtungs-WLANs
#define AP_CHANNEL 3              // WLAN-Kanal (1-13)
#define AP_HIDDEN 0               // Sichtbarkeit: 0=Sichtbar, 1=Versteckt
#define AP_MAX_CONN 4             // Maximale Anzahl gleichzeitiger Clients

// ============================================================
// 2. WLAN-CLIENT & TIMING KONFIGURATION
// ============================================================
struct WifiConfig {
  String ssid;         // Ziel-SSID (z.B. Boot-Router)
  String password;     // Ziel-Passwort
  uint16_t port = 80;  // Port für Webdienste / WebSocket

  unsigned long gpsIntervalMs = 900;   // Abfrageintervall GPS-Daten (ms)
  unsigned long imuIntervalMs = 10;    // Abfrageintervall Lage-Sensoren (ms)
  unsigned long sendIntervalMs = 100;  // Intervall für Telemetrie-Versand (ms)
};
extern WifiConfig wifiConfig;

void loadConfig();  // Lädt Einstellungen aus config.json
void saveConfig();  // Speichert Einstellungen in config.json

// ============================================================
// 3. DEBUG-EINSTELLUNGEN (SERIAL MONITOR)
// ============================================================
#define DEBUG_MODE false       // Allgemeiner System-Debug
#define DEBUG_MODE_WIFI false  // WLAN-Status & Verbindungs-Logs
#define DEBUG_MODE_GPS false   // NMEA-Rohdaten & Fix-Status
#define DEBUG_MODE_IMU true    // Lage- & Kompassdaten (Roll/Pitch/Heading)

// ============================================================
// 4. NETZWERK FALLBACK & TIMEOUTS
// ============================================================
// Zeit in ms bis zum Wechsel in den AP-Modus bei Verbindungsverlust
#define WIFI_AP_FALLBACK_TIMEOUT 90000

// ============================================================
// 5. GPS HARDWARE & FILTER (UART)
// ============================================================
#define GPS_SERIAL_PORT Serial1  // Genutzte Hardware-Schnittstelle
#define GPS_BAUDRATE 9600        // Standard-Baudrate für NMEA-Module
#define GPS_RX_PIN 5             // Dateneingang (GPS -> ESP32)
#define GPS_TX_PIN 17            // Datenausgang (ESP32 -> GPS)

// GPS-Filter & Prognose-Parameter
#define GPS_PREDICT_S 1.0f         // Prognose-Zeitraum für Position (Sekunden)
#define GPS_MIN_VALID_SPEED 0.13f  // Schwellwert für Bewegungserkennung (kn)
#define GPS_HDOP_MAX_WEIGHT 20.0f  // Gewichtungsfaktor für GPS-Präzision (HDOP)
#define GPS_AHRS_SYSTEM true       // true: AHRS-Stabilisierung / false: Rohdaten

// ============================================================
// 6. IMU HARDWARE (I2C)
// ============================================================
#define IMU_I2C_SDA 18  // I2C Datenleitung
#define IMU_I2C_SCL 19  // I2C Taktleitung

// ============================================================
// 7. STEPPERMOTOR HARDWARE (PINS)
// ============================================================
#define NAVIS_PIN_STEPPER_STEP 26   // Taktsignal (Step)
#define NAVIS_PIN_STEPPER_DIR 27    // Richtungssignal (Direction)
#define NAVIS_PIN_STEPPER_EN 25     // Treiber-Freigabe (Enable)
#define NAVIS_PIN_STEPPER_ALARM 34  // Fehlereingang vom Treiber (Alarm)

// Physikalische Endlagenschalter
#define NAVIS_PIN_STEPPER_END_MIN 32  // Endschalter eingefahren (Minimum)
#define NAVIS_PIN_STEPPER_END_MAX 33  // Endschalter ausgefahren (Maximum)

// ============================================================
// 8. MECHANISCHE LIMITS (RUDER)
// ============================================================
struct RudderConfig {
  double minAngle = -45.0;  // Minimal zulässiger Ruderwinkel (°)
  double maxAngle = 45.0;   // Maximal zulässiger Ruderwinkel (°)
};
extern RudderConfig rudderConfig;
