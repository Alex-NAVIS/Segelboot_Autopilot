#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <ArduinoWebsockets.h>
#include <HTTPClient.h>

using namespace websockets;

#include "Config.h"
#include "Autopilot_Data.h"
#include "IMU.h"

// ============================================================
// EXTERNE OBJEKTE & STATUS
// ============================================================
extern AsyncWebServer *server;     // Instanz des lokalen Webservers
extern AsyncWebSocket ws;          // Lokaler WebSocket Server (für Web-UI)
extern WebsocketsClient wsClient;  // WebSocket Client (Verbindung zum Boot-Master)
extern bool apModeActive;          // Status: true, wenn das Modul als Access Point läuft

// ============================================================
// WLAN-STEUERUNG & STATUS
// ============================================================
void wifiStart();           // Initialisiert WLAN (Client-Suche oder AP-Start)
void wifiLoop();            // Überwachung der Verbindung und Reconnect-Logik
bool wifiIsConnected();     // Prüft, ob eine aktive WLAN-Verbindung besteht
String getConnectedSSID();  // Gibt den Namen des aktuell verbundenen Netzwerks zurück
bool loadWifiConfig();      // Lädt spezifische WLAN-Zugangsdaten aus dem Flash
void startAPMode();         // Erzwingt den Start des eigenen Access Points

// ============================================================
// WEB-SERVER & API HANDLER
// ============================================================
void setupAPWebServer();                                                                                              // Konfiguriert die Routen des Webservers (UI/API)
void handleWifiScanRequest(AsyncWebServerRequest *request);                                                           // Scannt verfügbare WLANs
void handleSaveConfigRequest(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);  // Speichert UI-Einstellungen
void listDir(fs::FS &fs, String dirname, String &json, bool &first);                                                  // Listet Dateien für das Web-Interface

// ============================================================
// WEBSOCKET CLIENT (KOMMUNIKATION ZUM BOOT-MASTER)
// ============================================================
void initWSClient();                      // Initialisiert die Verbindung zum Master-ESP
void wsLoop();                            // Verarbeitet eingehende Pakete vom Master
void onWSMessage(WebsocketsMessage msg);  // Event-Handler für Nachrichten vom Master

// ============================================================
// DATEN-ÜBERTRAGUNG (TELEMETRIE)
// ============================================================
void sende_daten();                                           // Hauptfunktion zum Versenden aller Sensorpakete
size_t buildAutopilotJson(char *buffer, size_t bufferSize);   // Erstellt das JSON-Datenpaket
void sendDataToBoot(const char *payload, size_t len);         // Sendet Rohdaten zum Master-ESP
void wsBroadcastSensorData(const char *payload, size_t len);  // Sendet Daten an alle Browser-Clients
