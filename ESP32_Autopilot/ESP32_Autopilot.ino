#include <Arduino.h>
#include <LittleFS.h>

#include "Config.h"
#include "GPS.h"
#include "IMU.h"
#include "wifi_manager.h"
#include "navis_stepper.h"
#include "adaptive_autopilot_navis.h"

// ============================================================
// ZEITSTEUERUNG (TIMING)
// ============================================================
unsigned long lastSend = 0;  // Zeitstempel für den Datenversand (Websocket/Serial)
unsigned long lastgps = 10;  // Zeitstempel für die GPS-Abfrage
unsigned long lastimu = 20;  // Zeitstempel für die IMU-Abfrage (Lage/Kompass)

// ============================================================
// SETUP: SYSTEM-INITIALISIERUNG
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  if (DEBUG_MODE) Serial.println("\n--- Autopilot Modul Start ---");

  // --- Dateisystem (LittleFS) ---
  if (!LittleFS.begin()) {
    if (DEBUG_MODE) Serial.println("LittleFS Fehler! Formatiere und starte neu...");
    LittleFS.format();
    ESP.restart();
  }

  // --- Konfiguration & Netzwerk ---
  loadConfig();    // Globale Einstellungen aus dem Flash laden
  wifiStart();     // WLAN Verbindung (Client oder AP-Modus)
  initWSClient();  // WebSocket-Verbindung zum Master/Server herstellen

  // --- Sensorik ---
  // setup_GPS();    // GPS-Empfänger via UART (Pins 5/17) - Aktuell deaktiviert
  setup_imu();  // IMU via I2C (SDA=8, SCL=9) initialisieren

  // --- Antrieb & Regler ---
  navisStepperInit();  // Schrittmotor-Treiber (Pins/Parameter) vorbereiten
  setup_autopilot();   // Autopilot-Regler (PID/Modi) starten

  if (DEBUG_MODE) Serial.println("--- Setup abgeschlossen ---");
}

// ============================================================
// MAIN LOOP: HAUPTPROGRAMM-SCHLEIFE
// ============================================================
void loop() {
  // --- Kommunikation & Netzwerk-Wartung ---
  wifiLoop();  // Überwachung der WLAN-Verbindung
  wsLoop();    // Abwicklung der WebSocket-Datenpakete

  // --- Zyklische GPS-Auswertung ---
  if (millis() - lastgps >= wifiConfig.gpsIntervalMs) {
    lastgps = millis();
    // read_GPS(); // GPS-Buffer auslesen und parsen
  }

  // --- Zyklische IMU-Auswertung (Lagedaten) ---
  if (millis() - lastimu >= wifiConfig.imuIntervalMs) {
    lastimu = millis();
    read_imu();  // Roll, Pitch und Kompass-Heading aktualisieren
  }

  // --- Zyklischer Datenversand (Telemetrie) ---
  if (millis() - lastSend >= wifiConfig.sendIntervalMs) {
    lastSend = millis();
    sende_daten();  // Aktuelle Zustände an das Dashboard übertragen
  }

  // --- Motorsteuerung & Sicherheit ---
  navisStepperUpdateEndlagen();   // Abfrage der physikalischen Endstopps (Min/Max)
  adaptiveAutopilotNavis_loop();  // Kursberechnung und PID-Regler (ca. 10Hz)
  adaptiveAutopilotMotor_loop();  // Hochfrequente Stepper-Logik (Soll-Positionierung)
  navisStepperLoop();             // Schnelle Hardware-Ansteuerung (Steps ausgeben)

  // --- Hintergrund-Tasks (Calibration & Flags) ---
  // Startet die Magnetometer-Kalibrierung bei gesetztem Flag
  if (mag_cal && !mag_cal_running) {
    start_mag_calibration(30);
  }

  // Verarbeitet die Kalibrierungswerte im Hintergrund
  update_mag_calibration();
}
