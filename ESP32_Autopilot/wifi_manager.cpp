#include "wifi_manager.h"

// ============================================================
// GLOBALE INSTANZEN
// ============================================================
AsyncWebServer *server = nullptr;  // Zeiger auf den Webserver
AsyncWebSocket ws("/ws");          // Lokaler WebSocket-Endpunkt
WebsocketsClient wsClient;         // Client für Master-Kommunikation

// ============================================================
// GLOBALE VARIABLEN & STATUS
// ============================================================
bool apModeActive = false;        // AP-Modus aktiv/inaktiv
bool everConnected = false;       // Erfolgshistorie der Verbindung
int failedAttempts = 0;           // Zähler für Fehlversuche
unsigned long wifiStartTime = 0;  // Startzeit für AP-Fallback Timer

// ============================================================
// WLAN-STEUERUNG (START & MANAGEMENT)
// ============================================================

/**
 * Initialisiert die WLAN-Schnittstelle
 */
void wifiStart() {
  wifiStartTime = millis();  // Zeitstempel für Fallback setzen

  if (loadWifiConfig()) {
    WiFi.mode(WIFI_STA);  // Client-Modus vorbereiten
    WiFi.begin(wifiConfig.ssid.c_str(), wifiConfig.password.c_str());
    apModeActive = false;
  } else {
    startAPMode();  // Direkt in AP-Modus bei fehlender Config
  }

  // Webserver sicher neu starten
  if (server) {
    delete server;
    server = nullptr;
  }
  server = new AsyncWebServer(80);
  setupAPWebServer();
  delay(100);
  server->begin();
}

/**
 * Überwacht den WLAN-Status im Haupt-Loop
 */
void wifiLoop() {
  // Wenn verbunden: Status zurücksetzen
  if (WiFi.status() == WL_CONNECTED) {
    everConnected = true;
    failedAttempts = 0;
    return;
  }

  // Wenn getrennt und nicht im AP-Modus: Reconnect-Logik
  if (!apModeActive) {
    static unsigned long lastAttempt = 0;
    if (millis() - lastAttempt > 5000) {  // Alle 5 Sekunden probieren
      lastAttempt = millis();
      failedAttempts++;
      if (DEBUG_MODE_WIFI) Serial.printf("[WLAN] Getrennt, Versuch %d...\n", failedAttempts);
      WiFi.disconnect();
      WiFi.begin(wifiConfig.ssid.c_str(), wifiConfig.password.c_str());
    }

    // Wechsel in AP-Modus nach definiertem Timeout
    if (!everConnected && (millis() - wifiStartTime > WIFI_AP_FALLBACK_TIMEOUT)) {
      if (DEBUG_MODE_WIFI) Serial.println("[WLAN] Timeout: Starte AP-Modus");
      startAPMode();
    }
  }
}

bool wifiIsConnected() {
  return WiFi.status() == WL_CONNECTED;
}

String getConnectedSSID() {
  return WiFi.status() == WL_CONNECTED ? WiFi.SSID() : "";
}

/**
 * Startet den lokalen Access-Point zur Konfiguration
 */
void startAPMode() {
  apModeActive = true;
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, AP_HIDDEN, AP_MAX_CONN);

  if (!ok) {
    if (DEBUG_MODE_WIFI) Serial.println("[WLAN] Fehler beim AP-Start!");
    return;
  }

  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Maximale Sendeleistung
  IPAddress ip = WiFi.softAPIP();
  if (DEBUG_MODE_WIFI) {
    Serial.printf("[WLAN] AP aktiv. IP: %s\n", ip.toString().c_str());
  }
}

// ============================================================
// WEBSOCKET: NACHRICHTEN-VERARBEITUNG
// ============================================================

/**
 * Event-Handler für eingehende WebSocket-Nachrichten vom Master
 */
void onWSMessage(WebsocketsMessage msg) {
  if (!msg.isText()) return;

  String payloadStr = msg.data();
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, payloadStr);

  if (error) {
    Serial.printf("[WS] JSON-Fehler: %s\n", error.c_str());
    return;
  }

  // --- Master-Sensordaten aktualisieren ---
  MasterSensors.roll = doc["roll"] | MasterSensors.roll;
  MasterSensors.pitch = doc["pitch"] | MasterSensors.pitch;
  MasterSensors.kompass = doc["kompass"] | MasterSensors.kompass;
  MasterSensors.winddir_gemessen = doc["winddir_gemessen"] | MasterSensors.winddir_gemessen;
  MasterSensors.windspeed_gemessen = doc["windspeed_gemessen"] | MasterSensors.windspeed_gemessen;
  MasterSensors.winddir_berechnet = doc["winddir_berechnet"] | MasterSensors.winddir_berechnet;
  MasterSensors.windspeed_berechnet = doc["windspeed_berechnet"] | MasterSensors.windspeed_berechnet;
  MasterSensors.gps_lat = doc["gps_lat"] | MasterSensors.gps_lat;
  MasterSensors.gps_lon = doc["gps_lon"] | MasterSensors.gps_lon;
  MasterSensors.gps_speed = doc["gps_speed"] | MasterSensors.gps_speed;
  MasterSensors.gps_kurs = doc["gps_kurs"] | MasterSensors.gps_kurs;
  MasterSensors.gps_jahr = doc["gps_jahr"] | MasterSensors.gps_jahr;
  MasterSensors.gps_monat = doc["gps_monat"] | MasterSensors.gps_monat;
  MasterSensors.gps_tag = doc["gps_tag"] | MasterSensors.gps_tag;
  MasterSensors.gps_stunde = doc["gps_stunde"] | MasterSensors.gps_stunde;
  MasterSensors.gps_minute = doc["gps_minute"] | MasterSensors.gps_minute;
  MasterSensors.gps_sekunde = doc["gps_sekunde"] | MasterSensors.gps_sekunde;
  MasterSensors.missweisung = doc["missweisung"] | MasterSensors.missweisung;
  MasterSensors.Echolot = doc["Echolot"] | MasterSensors.Echolot;
  MasterSensors.relay_a = doc["relay_a"] | MasterSensors.relay_a;
  MasterSensors.relay_b = doc["relay_b"] | MasterSensors.relay_b;
  MasterSensors.relay_c = doc["relay_c"] | MasterSensors.relay_c;
  MasterSensors.relay_d = doc["relay_d"] | MasterSensors.relay_d;
  MasterSensors.mast_sensor = doc["mast_sensor"] | MasterSensors.mast_sensor;

  // --- Autopilot-Zustände verarbeiten ---
  if (doc.containsKey("autopilot")) {
    JsonObject ap = doc["autopilot"];

    // Modus (0=OFF, 1=Kompass, 2=GPS, 3=Wind)
    int newMode = ap["mode"] | autopilotSystem.currentMode;
    if (newMode >= 0 && newMode <= 3 && newMode != autopilotSystem.currentMode) {
      autopilotSystem.currentMode = newMode;
      autopilotEvents.modeChanged = true;
    }

    // Kurs-Offset (+- 45°)
    autopilotSystem.offsetWinkel = ap["offset"] | autopilotSystem.offsetWinkel;

    // Manuelle Pinnenbewegung
    int newPinne = ap["pinne"] | autopilotSystem.currentPinne;
    if (newPinne != autopilotSystem.currentPinne) {
      autopilotSystem.currentPinne = newPinne;
      autopilotEvents.manualChanged = true;
    }

    // GPS-Zielkoordinaten
    autopilotSystem.autopilot_lat = ap["target_lat"] | autopilotSystem.autopilot_lat;
    autopilotSystem.autopilot_lon = ap["target_lon"] | autopilotSystem.autopilot_lon;
  }
}

// ============================================================
// WEBSOCKET: CLIENT-INITIALISIERUNG
// ============================================================

/**
 * Bereitet den WebSocket Client für die Verbindung zum Boot-ESP vor
 */
void initWSClient() {
  wsClient.onMessage(onWSMessage);
  wsClient.onEvent([](WebsocketsEvent e, String data) {
    switch (e) {
      case WebsocketsEvent::ConnectionOpened: Serial.println("[WS] Verbunden zum Master"); break;
      case WebsocketsEvent::ConnectionClosed: Serial.println("[WS] Verbindung getrennt"); break;
      case WebsocketsEvent::GotPing: Serial.println("[WS] Ping erhalten"); break;
      case WebsocketsEvent::GotPong: Serial.println("[WS] Pong erhalten"); break;
    }
  });
  wsClient.connect("ws://192.168.4.1:80");  // Standard-IP des Boot-Masters
}

/**
 * Muss im Hauptloop zur Paketverarbeitung aufgerufen werden
 */
void wsLoop() {
  wsClient.poll();
}

// ============================================================
// WEB-SERVER: SETUP & API-ROUTEN
// ============================================================
void setupAPWebServer() {
  if (server == nullptr) {
    if (DEBUG_MODE_WIFI) Serial.println(F("[HTTP] Fehler: Server-Instanz ist NULL!"));
    return;
  }

  // --- API: Autopilot Status (JSON) ---
  server->on("/autopilot_status", HTTP_GET, [](AsyncWebServerRequest *request) {
    StaticJsonDocument<256> doc;
    doc["mode"] = autopilotSystem.currentMode;
    doc["pinne"] = autopilotSystem.currentPinne;
    doc["offset"] = autopilotSystem.offsetWinkel;
    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
  });

  // --- API: Direkte Steuerbefehle ---
  server->on("/autopilot_data", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("modus")) {
      int newMode = request->getParam("modus")->value().toInt();
      if (newMode != autopilotSystem.currentMode) {
        autopilotSystem.currentMode = newMode;
        autopilotEvents.modeChanged = true;
      }
    }
    if (request->hasParam("pinne")) {
      int newPinne = request->getParam("pinne")->value().toInt();
      if (newPinne != autopilotSystem.currentPinne) {
        autopilotSystem.currentPinne = newPinne;
        autopilotEvents.manualChanged = true;
      }
    }
    if (request->hasParam("winkel")) {
      autopilotSystem.offsetWinkel =
        request->getParam("winkel")->value().toInt();
    }
    request->send(200, "text/plain", "OK");
  });


  // --- API: WLAN-Umgebung scannen ---
  server->on("/scan", HTTP_GET, [](AsyncWebServerRequest *request) {
    int n = WiFi.scanNetworks();
    String json = "[";
    for (int i = 0; i < n; i++) {
      json += "\"" + WiFi.SSID(i) + "\"";
      if (i < n - 1) json += ",";
    }
    json += "]";
    request->send(200, "application/json", json);
  });

  // --- API: Konfiguration speichern (POST) ---
  server->on(
    "/save", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleSaveConfigRequest(request, data, len, index, total);
    });

  // --- API: Sensor-Kalibrierung ---
  server->on("/imu_calibrate_mag", HTTP_POST, [](AsyncWebServerRequest *request) {
    mag_cal = true;  // Trigger für Hintergrund-Task
    request->send(200, "text/plain", "Magnetometer-Kalibrierung gestartet!");
  });

  server->on("/imu_calibrate_gyro", HTTP_POST, [](AsyncWebServerRequest *request) {
    calibrate_gyro();
    request->send(200, "text/plain", "Gyro-Kalibrierung gestartet!");
  });

  // --- API: Dateisystem-Übersicht (JSON) ---
  server->on("/list_files", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "[";
    bool first = true;
    listDir(LittleFS, "/", json, first);

    // Speicher-Metriken anhängen
    size_t total = LittleFS.totalBytes();
    size_t used = LittleFS.usedBytes();
    if (!first) json += ",";
    json += "{\"fs_total\":" + String(total) + ",\"fs_used\":" + String(used) + "}";

    json += "]";
    request->send(200, "application/json", json);
  });

  // --- API: Datei-Upload (mit Speicherprüfung) ---
  server->on(
    "/upload", HTTP_POST,
    [](AsyncWebServerRequest *request) {
      if (request->_tempObject != nullptr) {
        request->send(507, "text/plain", "Speicher voll oder Schreibfehler");
      } else {
        request->send(200, "text/plain", "Upload erfolgreich");
      }
    },
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      if (index == 0) {
        size_t freeSpace = LittleFS.totalBytes() - LittleFS.usedBytes();
        if (request->contentLength() > freeSpace) {
          request->_tempObject = (void *)1;  // Fehler-Flag setzen
          return;
        }
      }
      if (request->_tempObject == nullptr) {
        String path = "/" + filename;
        File file = LittleFS.open(path, index == 0 ? "w" : "a");
        if (file) {
          if (file.write(data, len) != len) request->_tempObject = (void *)1;
          file.close();
        } else {
          request->_tempObject = (void *)1;
        }
      }
    });

  // --- WebSocket & Statische Dateien ---
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) Serial.printf("[WS] Client %u verbunden\n", client->id());
    else if (type == WS_EVT_DISCONNECT) Serial.printf("[WS] Client %u getrennt\n", client->id());
  });
  server->addHandler(&ws);

  // LittleFS Root als statische Webseite (liefert index.html als Default)
  server->serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server->onNotFound([](AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Pfad nicht gefunden");
  });
}

// ============================================================
// DATEISYSTEM: REKURSIVE DATEILISTE
// ============================================================
void listDir(fs::FS &fs, String dirname, String &json, bool &first) {
  File root = fs.open(dirname);
  if (!root || !root.isDirectory()) return;

  File file = root.openNextFile();
  while (file) {
    if (!first) json += ",";
    first = false;

    json += "{\"name\":\"" + String(file.path()) + "\",";
    json += "\"size\":" + String(file.size()) + ",";
    json += "\"type\":\"" + String(file.isDirectory() ? "dir" : "file") + "\"}";

    if (file.isDirectory()) {
      listDir(fs, file.path(), json, first);
    }
    file = root.openNextFile();
  }
}

// ============================================================
// API-HANDLER: KONFIGURATION SPEICHERN
// ============================================================
void handleSaveConfigRequest(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
  if (index != 0 || (index + len) != total) return;

  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, data, len);
  if (err) {
    request->send(400, "text/plain", "JSON ungültig");
    return;
  }

  // --- WLAN ---
  wifiConfig.ssid = doc["ssid"].as<String>();
  wifiConfig.password = doc["password"].as<String>();
  wifiConfig.port = doc["port"] | 80;

  // --- Intervalle ---
  wifiConfig.gpsIntervalMs = doc["gpsIntervalMs"] | 250;
  wifiConfig.imuIntervalMs = doc["imuIntervalMs"] | 250;
  wifiConfig.sendIntervalMs = doc["sendIntervalMs"] | 500;

  // --- Autopilot PID & Motor ---
  autopilotSystem.P_base = doc["P_base"] | 0.0;
  autopilotSystem.I_base = doc["I_base"] | 0.0;
  autopilotSystem.D_base = doc["D_base"] | 0.0;
  autopilotSystem.invertPinne = doc["invert"] | 0;
  autopilotSystem.maxStepperSpeed = doc["maxStepperSpeed"] | 100;
  autopilotSystem.maxStepperAccel = doc["maxStepperAccel"] | 50;

  // --- Ruder-Limits ---
  rudderConfig.minAngle = doc["rudderMin"] | rudderConfig.minAngle;
  rudderConfig.maxAngle = doc["rudderMax"] | rudderConfig.maxAngle;

  saveConfig();  // Permanent im Flash speichern

  request->send(200, "text/plain", "Gespeichert. Neustart...");
  delay(300);
  ESP.restart();
}

// ============================================================
// KONFIGURATION: WLAN-SPEZIFISCH LADEN
// ============================================================
bool loadWifiConfig() {
  if (!LittleFS.exists("/config.json")) return false;

  File f = LittleFS.open("/config.json", "r");
  if (!f) return false;

  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, f);
  f.close();

  if (error) return false;

  // ---- WLAN-Zugangsdaten ----
  wifiConfig.ssid = doc["ssid"].as<String>();
  wifiConfig.password = doc["password"].as<String>();
  wifiConfig.port = doc["port"] | 80;

  // ---- System-Intervalle ----
  wifiConfig.gpsIntervalMs = doc["gpsIntervalMs"] | 250;
  wifiConfig.imuIntervalMs = doc["imuIntervalMs"] | 250;
  wifiConfig.sendIntervalMs = doc["sendIntervalMs"] | 500;

  // ---- Autopilot PID & Motorik ----
  autopilotSystem.P_base = doc["P_base"] | 0.0;
  autopilotSystem.I_base = doc["I_base"] | 0.0;
  autopilotSystem.D_base = doc["D_base"] | 0.0;
  autopilotSystem.invertPinne = doc["invert"] | 0;
  autopilotSystem.maxStepperSpeed = doc["maxStepperSpeed"] | 100;
  autopilotSystem.maxStepperAccel = doc["maxStepperAccel"] | 50;

  // ---- Ruder-Mechanik ----
  rudderConfig.minAngle = doc["rudderMin"] | rudderConfig.minAngle;
  rudderConfig.maxAngle = doc["rudderMax"] | rudderConfig.maxAngle;

  return wifiConfig.ssid.length() > 0;
}

// ============================================================
// TELEMETRIE: DATEN-VERSAND STEUERUNG
// ============================================================
void sende_daten() {
  if (DEBUG_MODE_WIFI) Serial.println("[DATA] Erstelle Telemetrie-Paket (GPS/IMU)...");

  static char jsonBuffer[768];
  size_t len = buildAutopilotJson(jsonBuffer, sizeof(jsonBuffer));

  if (len == 0) return;

  wifi_mode_t mode = WiFi.getMode();

  // Im Client-Modus (STA): Daten zum Boot-Master senden
  if (mode == WIFI_STA) {
    sendDataToBoot(jsonBuffer, len);
  }

  // Im AP-Modus: Daten via WebSocket an verbundene Browser senden
  if (mode == WIFI_AP || mode == WIFI_AP_STA) {
    wsBroadcastSensorData(jsonBuffer, len);
  }
}

// ============================================================
// TELEMETRIE: JSON-GENERATOR (SENSORDATEN)
// ============================================================
size_t buildAutopilotJson(char *buffer, size_t bufferSize) {
  StaticJsonDocument<768> doc;

  // --- GPS Daten ---
  doc["gps_lat"] = autopilotdaten.gps_lat;
  doc["gps_lon"] = autopilotdaten.gps_lon;
  doc["gps_speed"] = autopilotdaten.gps_speed;
  doc["gps_kurs"] = autopilotdaten.gps_kurs;
  doc["gps_sats"] = autopilotdaten.gps_sats;
  doc["gps_hdop"] = autopilotdaten.gps_hdop;
  doc["gps_jahr"] = autopilotdaten.gps_jahr;
  doc["gps_monat"] = autopilotdaten.gps_monat;
  doc["gps_tag"] = autopilotdaten.gps_tag;
  doc["gps_stunde"] = autopilotdaten.gps_stunde;
  doc["gps_minute"] = autopilotdaten.gps_minute;
  doc["gps_sekunde"] = autopilotdaten.gps_sekunde;

  // --- IMU / Lage ---
  doc["imu_roll"] = autopilotdaten.roll;
  doc["imu_pitch"] = autopilotdaten.pitch;
  doc["imu_kompass"] = autopilotdaten.kompass;

  // --- Kalibrierungs-Status ---
  doc["magCalRunning"] = mag_cal;
  if (mag_cal) {
    doc["magProgress"] = magProgress;  // Fortschritt 0-100%
    doc["magX"] = web_mx;              // Magnetometer X-Rohwert
    doc["magY"] = web_my;              // Magnetometer Y-Rohwert
    doc["magZ"] = web_mz;              // Magnetometer Z-Rohwert
  }

  // --- Mechanik ---
  doc["rudderAngle"] = autopilotdaten.rudderAngle;  // Aktueller Ruderwinkel (°)

  return serializeJson(doc, buffer, bufferSize);
}

// ============================================================
// NETZWERK: DATEN AN MASTER-ESP (HTTP POST)
// ============================================================
void sendDataToBoot(const char *payload, size_t len) {
  if (!wifiIsConnected()) {
    if (DEBUG_MODE_WIFI) Serial.println("[HTTP] Warnung: WLAN nicht verbunden");
    return;
  }

  WiFiClient client;
  HTTPClient http;

  // Ziel-URL des Boot-Masters
  http.begin(client, "http://192.168.4.1/autopilot");
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.POST((uint8_t *)payload, len);

  if (DEBUG_MODE_WIFI) {
    if (httpCode > 0)
      Serial.printf("[HTTP] POST erfolgreich: %d\n", httpCode);
    else
      Serial.printf("[HTTP] Fehler: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
}

// ============================================================
// NETZWERK: DATEN AN WEB-CLIENTS (WEBSOCKET BROADCAST)
// ============================================================
void wsBroadcastSensorData(const char *payload, size_t len) {
  if (ws.count() == 0) return;  // Keine Clients verbunden

  // Nur senden, wenn der Buffer bereit ist (verhindert Blockaden)
  if (ws.availableForWriteAll()) {
    ws.textAll(payload, len);
  }
}