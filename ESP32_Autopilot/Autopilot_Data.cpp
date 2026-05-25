#include "autopilot_data.h"

// ============================================================
// INITIALISIERUNG: SENSOR DATEN (LIVE-WERTE)
// ============================================================
Autopilot_Sensoren autopilotdaten = {
  .gps_lat = 0.0,             // Aktuelle Breitengrad-Koordinate (Dezimalgrad)
  .gps_lon = 0.0,             // Aktuelle Längengrad-Koordinate (Dezimalgrad)
  .gps_speed = 0.0,           // Geschwindigkeit über Grund (SOG) in Knoten
  .gps_kurs = 0.0,            // Kurs über Grund (COG) in Grad
  .gps_sats = 0,              // Anzahl der aktuell empfangenen Satelliten
  .gps_hdop = 0.0f,           // Horizontale Genauigkeit des GPS-Fix
  .gps_jahr = 0,              // Aktuelles Jahr aus dem NMEA-Datensatz
  .gps_monat = 0,             // Aktueller Monat (1-12)
  .gps_tag = 0,               // Aktueller Tag (1-31)
  .gps_stunde = 0,            // UTC Stunde
  .gps_minute = 0,            // UTC Minute
  .gps_sekunde = 0,           // UTC Sekunde
  .roll = 0.0,                // Neigung um die Längsachse in Grad (Krängung)
  .pitch = 0.0,               // Neigung um die Querachse in Grad (Stampfen)
  .kompass = 0.0,             // Magnetischer Kurs oder gefilterter Heading-Wert
  .winddir_gemessen = 0.0,    // Scheinbare Windrichtung relativ zum Bug (0-360°)
  .windspeed_gemessen = 0.0,  // Scheinbare Windgeschwindigkeit in Knoten
  .rudderAngle = 0.0          // Aktueller physikalischer Winkel des Ruderblatts
};

// ============================================================
// INITIALISIERUNG: SYSTEM EINSTELLUNGEN (KONFIGURATION)
// ============================================================
Autopilot_System autopilotSystem = {
  .invertPinne = false,        // Wirkrichtung des Stepper-Antriebs umkehren
  .P_base = 1.0f,              // Proportional-Anteil des PID-Reglers
  .I_base = 0.1f,              // Integral-Anteil des PID-Reglers
  .D_base = 0.05f,             // Differenzial-Anteil des PID-Reglers
  .maxStepperSpeed = 1000.0f,  // Maximale Taktrate für den Schrittmotor
  .maxStepperAccel = 500.0f,   // Maximale Beschleunigung für den Schrittmotor
  .currentMode = 0,            // Betriebsmodus: 0=Aus, 1=Kompass, 2=GPS, 3=Wind
  .currentPinne = 0,           // Manueller Steuerbefehl: 0=Stop, 1=In, 2=Out
  .offsetWinkel = 0,           // Kurskorrektur/User-Offset zum Sollkurs (+-45°)
  .endlageMin = false,         // Status-Flag für den minimalen Endanschlag
  .endlageMax = false,         // Status-Flag für den maximalen Endanschlag
  .stepperPosition = 0L,       // Aktueller Schrittzähler des Motors (Ist-Position)
  .autopilot_lat = 0.0,        // Ziel-Breitengrad für Wegpunkt-Navigation
  .autopilot_lon = 0.0         // Ziel-Längengrad für Wegpunkt-Navigation
};

// ============================================================
// INITIALISIERUNG: SYSTEM EVENTS (FLAGS)
// ============================================================
AutopilotEvents autopilotEvents = {
  .modeChanged = false,   // Flag: Modus wurde via Web/Taster geändert
  .manualChanged = false  // Flag: Manuelle Bewegung wurde angefordert
};

// ============================================================
// INITIALISIERUNG: MASTER DATEN (GATEWAY)
// ============================================================
MasterSensorData MasterSensors = {
  .roll = 0.0f,                 // Rollwinkel vom Master-Sensor
  .pitch = 0.0f,                // Pitchwinkel vom Master-Sensor
  .kompass = 0.0f,              // Kompasskurs vom Master-Sensor
  .missweisung = 0.0f,          // Magnetische Deklination am aktuellen Standort
  .winddir_gemessen = 0.0f,     // Rohwert Windrichtung (apparent)
  .windspeed_gemessen = 0.0f,   // Rohwert Windgeschwindigkeit (apparent)
  .winddir_berechnet = 0.0f,    // Berechnete wahre Windrichtung (true)
  .windspeed_berechnet = 0.0f,  // Berechnete wahre Windgeschwindigkeit (true)
  .gps_lat = 0.0,               // GPS Breitengrad vom Master
  .gps_lon = 0.0,               // GPS Längengrad vom Master
  .gps_speed = 0.0f,            // GPS Geschwindigkeit vom Master
  .gps_kurs = 0.0f,             // GPS Kurs vom Master
  .gps_jahr = 0,                // Master Zeitstempel: Jahr
  .gps_monat = 0,               // Master Zeitstempel: Monat
  .gps_tag = 0,                 // Master Zeitstempel: Tag
  .gps_stunde = 0,              // Master Zeitstempel: Stunde
  .gps_minute = 0,              // Master Zeitstempel: Minute
  .gps_sekunde = 0,             // Master Zeitstempel: Sekunde
  .Echolot = 0.0f,              // Gemessene Wassertiefe unter dem Kiel
  .mast_sensor = false,         // Status-Flag: Mast-Einheit aktiv/verbunden
  .relay_a = false,             // Schaltzustand Relais A (z.B. Licht)
  .relay_b = false,             // Schaltzustand Relais B
  .relay_c = false,             // Schaltzustand Relais C
  .relay_d = false              // Schaltzustand Relais D
};
