#ifndef AUTOPILOT_DATA_H
#define AUTOPILOT_DATA_H

#include <Arduino.h>

// ============================================================
// SENSOR DATEN (LIVE-WERTE)
// ============================================================
struct Autopilot_Sensoren {
  double gps_lat;            // Aktuelle Breitengrad-Koordinate (Dezimalgrad)
  double gps_lon;            // Aktuelle Längengrad-Koordinate (Dezimalgrad)
  double gps_speed;          // Geschwindigkeit über Grund (SOG) in Knoten
  double gps_kurs;           // Kurs über Grund (COG) in Grad
  int gps_sats;              // Anzahl der aktuell empfangenen Satelliten
  float gps_hdop;            // Horizontale Genauigkeit des GPS-Fix (niedriger ist besser)
  int gps_jahr;              // Aktuelles Jahr aus dem NMEA-Datensatz
  int gps_monat;             // Aktueller Monat (1-12)
  int gps_tag;               // Aktueller Tag (1-31)
  int gps_stunde;            // UTC Stunde
  int gps_minute;            // UTC Minute
  int gps_sekunde;           // UTC Sekunde
  double roll;               // Neigung um die Längsachse in Grad (Krängung)
  double pitch;              // Neigung um die Querachse in Grad (Stampfen)
  double kompass;            // Magnetischer Kurs oder gefilterter Heading-Wert
  double winddir_gemessen;   // Scheinbare Windrichtung relativ zum Bug (0-360°)
  double windspeed_gemessen; // Scheinbare Windgeschwindigkeit in Knoten
  double rudderAngle;        // Aktueller physikalischer Winkel des Ruderblatts
};
extern Autopilot_Sensoren autopilotdaten;

// ============================================================
// SYSTEM EINSTELLUNGEN (KONFIGURATION & STATUS)
// ============================================================
struct Autopilot_System {
  bool invertPinne;          // Wirkrichtung des Stepper-Antriebs umkehren
  float P_base;              // Proportional-Anteil des PID-Reglers
  float I_base;              // Integral-Anteil des PID-Reglers
  float D_base;              // Differenzial-Anteil des PID-Reglers
  float maxStepperSpeed;     // Maximale Taktrate für den Schrittmotor
  float maxStepperAccel;     // Maximale Beschleunigung für den Schrittmotor
  int currentMode;           // Betriebsmodus: 0=Aus, 1=Kompass, 2=GPS, 3=Wind
  int currentPinne;          // Manueller Steuerbefehl: 0=Stop, 1=In, 2=Out
  int offsetWinkel;          // Kurskorrektur/User-Offset zum Sollkurs (+-45°)
  bool endlageMin;           // Status-Flag für den minimalen Endanschlag
  bool endlageMax;           // Status-Flag für den maximalen Endanschlag
  long stepperPosition;      // Aktueller Schrittzähler des Motors (Ist-Position)
  double autopilot_lat;      // Ziel-Breitengrad für Wegpunkt-Navigation
  double autopilot_lon;      // Ziel-Längengrad für Wegpunkt-Navigation
};
extern Autopilot_System autopilotSystem;

// ============================================================
// SYSTEM EVENTS (COMMUNICATION FLAGS)
// ============================================================
struct AutopilotEvents {
  volatile bool modeChanged;   // Flag: Modus wurde via Web/Taster geändert
  volatile bool manualChanged; // Flag: Manuelle Bewegung wurde angefordert
};
extern AutopilotEvents autopilotEvents;

// ============================================================
// MASTER DATEN (Schnittstelle zum Haupt-ESP32)
// ============================================================
struct MasterSensorData {
  float roll;                // Rollwinkel vom Master-Sensor
  float pitch;               // Pitchwinkel vom Master-Sensor
  float kompass;             // Kompasskurs vom Master-Sensor
  float missweisung;         // Magnetische Deklination am aktuellen Standort
  float winddir_gemessen;    // Rohwert Windrichtung (apparent)
  float windspeed_gemessen;  // Rohwert Windgeschwindigkeit (apparent)
  float winddir_berechnet;   // Berechnete wahre Windrichtung (true)
  float windspeed_berechnet; // Berechnete wahre Windgeschwindigkeit (true)
  double gps_lat;            // GPS Breitengrad vom Master
  double gps_lon;            // GPS Längengrad vom Master
  float gps_speed;           // GPS Geschwindigkeit vom Master
  float gps_kurs;            // GPS Kurs vom Master
  int gps_jahr;              // Master Zeitstempel: Jahr
  int gps_monat;             // Master Zeitstempel: Monat
  int gps_tag;               // Master Zeitstempel: Tag
  int gps_stunde;            // Master Zeitstempel: Stunde
  int gps_minute;            // Master Zeitstempel: Minute
  int gps_sekunde;           // Master Zeitstempel: Sekunde
  float Echolot;             // Gemessene Wassertiefe unter dem Kiel
  bool mast_sensor;          // Status-Flag: Mast-Einheit aktiv/verbunden
  bool relay_a;              // Schaltzustand Relais A (z.B. Licht)
  bool relay_b;              // Schaltzustand Relais B
  bool relay_c;              // Schaltzustand Relais C
  bool relay_d;              // Schaltzustand Relais D
};
extern MasterSensorData MasterSensors;

#endif // AUTOPILOT_DATA_H