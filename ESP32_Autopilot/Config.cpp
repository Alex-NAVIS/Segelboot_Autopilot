#include "Config.h"
#include "autopilot_data.h"
#include <ArduinoJson.h>

// ============================================================
// GLOBALE VARIABLEN & INSTANZEN
// ============================================================
bool mag_cal = false;  // Flag für die Magnetometer-Kalibrierung

// Standard-WLAN Konfiguration
WifiConfig wifiConfig;

// Physische Ruder-Limits (Winkel/Wegbegrenzung)
RudderConfig rudderConfig = {
  .minAngle = 25.0,   // Minimaler mechanischer Anschlag (°)
  .maxAngle = 1155.0  // Maximaler mechanischer Anschlag (°)
};

// ============================================================
// KONFIGURATION LADEN (LITTLEFS -> JSON)
// ============================================================
void loadConfig() {
  // Dateisystem initialisieren
  if (!LittleFS.begin(true)) {
    Serial.println("[Config] LittleFS Mount fehlgeschlagen.");
    return;
  }

  // Prüfen, ob Konfigurationsdatei existiert
  if (!LittleFS.exists("/config.json")) {
    Serial.println("[Config] Keine Datei gefunden, nutze Hardcoded-Defaults.");
    return;
  }

  // Datei zum Lesen öffnen
  File configFile = LittleFS.open("/config.json", "r");
  if (!configFile) return;

  // JSON-Dokument vorbereiten (Größe für alle Parameter optimiert)
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, configFile);
  configFile.close();

  if (error) {
    Serial.println("[Config] Fehler beim Parsen der JSON-Datei.");
    return;
  }

  // ---- WLAN & NETZWERK ----
  if (doc.containsKey("ssid")) wifiConfig.ssid = doc["ssid"].as<String>();
  if (doc.containsKey("password")) wifiConfig.password = doc["password"].as<String>();
  if (doc.containsKey("port")) wifiConfig.port = doc["port"];

  // ---- SYSTEM-INTERVALLE ----
  if (doc.containsKey("gpsInt")) wifiConfig.gpsIntervalMs = doc["gpsInt"];
  if (doc.containsKey("imuInt")) wifiConfig.imuIntervalMs = doc["imuInt"];
  if (doc.containsKey("sendInt")) wifiConfig.sendIntervalMs = doc["sendInt"];

  // ---- AUTOPILOT REGELUNG (PID & MOTOR) ----
  if (doc.containsKey("invertPinne")) autopilotSystem.invertPinne = doc["invertPinne"];
  if (doc.containsKey("P_base")) autopilotSystem.P_base = doc["P_base"];
  if (doc.containsKey("I_base")) autopilotSystem.I_base = doc["I_base"];
  if (doc.containsKey("D_base")) autopilotSystem.D_base = doc["D_base"];
  if (doc.containsKey("maxStepperSpeed")) autopilotSystem.maxStepperSpeed = doc["maxStepperSpeed"];
  if (doc.containsKey("maxStepperAccel")) autopilotSystem.maxStepperAccel = doc["maxStepperAccel"];

  // ---- MECHANISCHE RUDER-LIMITS ----
  if (doc.containsKey("rudderMin")) rudderConfig.minAngle = doc["rudderMin"];
  if (doc.containsKey("rudderMax")) rudderConfig.maxAngle = doc["rudderMax"];

  // ---- SICHERHEITSPRÜFUNG (PLAUSIBILITÄT) ----
  if (autopilotSystem.maxStepperSpeed < 0) autopilotSystem.maxStepperSpeed = 0;
  if (autopilotSystem.maxStepperAccel < 0) autopilotSystem.maxStepperAccel = 0;

  Serial.println("[Config] Konfiguration erfolgreich geladen.");
}

// ============================================================
// KONFIGURATION SPEICHERN (JSON -> LITTLEFS)
// ============================================================
void saveConfig() {
  StaticJsonDocument<1024> doc;

  // ---- Daten in JSON-Struktur schreiben ----
  doc["ssid"] = wifiConfig.ssid;
  doc["password"] = wifiConfig.password;
  doc["port"] = wifiConfig.port;
  doc["gpsInt"] = wifiConfig.gpsIntervalMs;
  doc["imuInt"] = wifiConfig.imuIntervalMs;
  doc["sendInt"] = wifiConfig.sendIntervalMs;
  doc["invertPinne"] = autopilotSystem.invertPinne;
  doc["P_base"] = autopilotSystem.P_base;
  doc["I_base"] = autopilotSystem.I_base;
  doc["D_base"] = autopilotSystem.D_base;
  doc["maxStepperSpeed"] = autopilotSystem.maxStepperSpeed;
  doc["maxStepperAccel"] = autopilotSystem.maxStepperAccel;
  doc["rudderMin"] = rudderConfig.minAngle;
  doc["rudderMax"] = rudderConfig.maxAngle;

  // Datei zum Schreiben öffnen (überschreiben)
  File configFile = LittleFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("[Config] Fehler beim Öffnen zum Schreiben.");
    return;
  }

  // JSON serialisieren und in Datei schreiben
  if (serializeJson(doc, configFile) == 0) {
    Serial.println("[Config] Fehler beim Schreiben der Daten.");
  }

  configFile.close();
  Serial.println("[Config] Konfiguration erfolgreich im Flash gespeichert.");
}
