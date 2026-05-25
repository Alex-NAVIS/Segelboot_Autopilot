#include "IMU.h"


// --- Definition der Konstante fuer den Dateinamen ---
// -------------------- FS Save/Load --------------------
#define CAL_FILE "/settings/IMU_gyro.json"
#define MAG_CAL_FILE "/settings/IMU_mag.json"

// --- Instanzen ---
static ICM_20948_I2C myICM;
static Madgwick filter;
static bool imu_initialized = false;

// --- Skalierung / Konstanten ---
float MAG_SCALE = 0.15;  // µT pro LSB

// --- veränderbare Hard-Iron Offsets ---
float mag_offset_x = 64.0;
float mag_offset_y = 40.0;
float mag_offset_z = -203.0;

//Mag Calibrierung Daten für den Webserver
float web_mx = 0;
float web_my = 0;
float web_mz = 0;
int magProgress = 0;

// --- veränderbare Gyro Offsets ---
float gyro_offset_x = 0.0;
float gyro_offset_y = 0.0;
float gyro_offset_z = 0.0;

// Magnetometer Calibration Variablen
bool mag_cal_running = false;
unsigned long mag_cal_start = 0;
static int mag_cal_duration = 30;

float mxmin, mxmax;
float mymin, mymax;
float mzmin, mzmax;
bool mag_cal_gotData = false;


// -------------------- Gyro Calibration Load --------------------
static void load_gyro_cal() {
  // Stellen Sie sicher, dass LittleFS.begin(true) in setup() aufgerufen wurde!

  if (!LittleFS.exists(CAL_FILE)) {
    if (DEBUG_MODE_IMU) Serial.println("Info: Gyro-Kalibrierungsdatei nicht gefunden, verwende Standardwerte (0.0).");
    return;
  }

  File f = LittleFS.open(CAL_FILE, "r");
  if (!f) {
    if (DEBUG_MODE_IMU) Serial.println("Fehler beim Öffnen der Kalibrierungsdatei zum Lesen.");
    return;
  }

  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, f);
  f.close();

  if (error) {
    if (DEBUG_MODE_IMU) Serial.print("Fehler beim Deserialisieren der Gyro-Kalibrierung: ");
    if (DEBUG_MODE_IMU) Serial.println(error.c_str());
    return;
  }

  // Lese Werte aus JSON und weise sie den neuen Variablen zu
  gyro_offset_x = doc["gx"] | 0.0f;
  gyro_offset_y = doc["gy"] | 0.0f;
  gyro_offset_z = doc["gz"] | 0.0f;

  if (DEBUG_MODE_IMU) Serial.println("Gyroskop Kalibrierung geladen.");
}

// -------------------- Gyro Calibration Save --------------------
static void save_gyro_cal() {
  if (!LittleFS.exists("/settings")) {
    LittleFS.mkdir("/settings");
  }

  File f = LittleFS.open(CAL_FILE, "w");
  if (!f) {
    if (DEBUG_MODE_IMU) Serial.println("Fehler beim Öffnen der Gyro-Kalibrierungsdatei zum Schreiben.");
    return;
  }

  StaticJsonDocument<128> doc;
  // Schreibe Werte der neuen Variablen in das JSON-Dokument
  doc["gx"] = gyro_offset_x;
  doc["gy"] = gyro_offset_y;
  doc["gz"] = gyro_offset_z;

  if (serializeJson(doc, f) == 0) {
    if (DEBUG_MODE_IMU) Serial.println("Fehler beim Schreiben der Gyro-Kalibrierung in die Datei.");
  }
  f.close();
  if (DEBUG_MODE_IMU) Serial.println("Gyroskop Kalibrierung gespeichert.");
}

// -------------------- Gyro Calibration --------------------
void calibrate_gyro() {
  if (DEBUG_MODE_IMU) Serial.println("Gyro calibration: keep sensor still...");
  const int N = 200;
  float sumX = 0, sumY = 0, sumZ = 0;
  int valid = 0;

  for (int i = 0; i < N; i++) {
    if (myICM.dataReady()) {
      myICM.getAGMT();  // Daten lesen (befüllt die internen Strukturen)

      // Die SparkFun Library liefert hier die skalierten DPS (Degrees per Second)
      sumX += myICM.gyrX();
      sumY += myICM.gyrY();
      sumZ += myICM.gyrZ();
      valid++;
    } else {
      delay(2);  // Gibt der Hardware Zeit, neue Daten bereitzustellen
    }
  }

  if (valid == 0) valid = 1;  // Division durch Null verhindern

  // Berechne die Offsets (Durchschnitt der Ruhewerte) und weise sie den globalen Variablen zu:
  gyro_offset_x = sumX / valid;
  gyro_offset_y = sumY / valid;
  gyro_offset_z = sumZ / valid;

  save_gyro_cal();  // Ruft Ihre Speicherfunktion auf
  if (DEBUG_MODE_IMU) Serial.println("Gyro calibration done");
}

// -------------------- Mag Calibration Load --------------------
static void load_mag_cal() {

  if (!LittleFS.exists(MAG_CAL_FILE)) {
    if (DEBUG_MODE_IMU) Serial.println("Info: Kalibrierungsdatei nicht gefunden, verwende Standardwerte.");
    return;
  }

  File f = LittleFS.open(MAG_CAL_FILE, "r");
  if (!f) {
    if (DEBUG_MODE_IMU) Serial.println("Fehler beim Öffnen der Kalibrierungsdatei zum Lesen.");
    return;
  }

  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, f);
  f.close();

  if (error) {
    if (DEBUG_MODE_IMU) Serial.print("Fehler beim Deserialisieren der Kalibrierung: ");
    if (DEBUG_MODE_IMU) Serial.println(error.c_str());
    return;
  }

  // Lese Werte aus JSON und weise sie den neuen Variablen zu
  mag_offset_x = doc["xOff"] | 0.0f;
  mag_offset_y = doc["yOff"] | 0.0f;
  mag_offset_z = doc["zOff"] | 0.0f;

  if (DEBUG_MODE_IMU) Serial.println("Magnetometer Kalibrierung geladen.");
}

// -------------------- Mag Calibration Save --------------------
static void save_mag_cal() {
  if (!LittleFS.exists("/settings")) {
    LittleFS.mkdir("/settings");
  }

  File f = LittleFS.open(MAG_CAL_FILE, "w");
  if (!f) {
    if (DEBUG_MODE_IMU) Serial.println("Fehler beim Öffnen der Kalibrierungsdatei zum Schreiben.");
    return;
  }

  StaticJsonDocument<128> doc;
  // Schreibe Werte der neuen Variablen in das JSON-Dokument
  doc["xOff"] = mag_offset_x;
  doc["yOff"] = mag_offset_y;
  doc["zOff"] = mag_offset_z;

  if (serializeJson(doc, f) == 0) {
    if (DEBUG_MODE_IMU) Serial.println("Fehler beim Schreiben der Kalibrierung in die Datei.");
  }
  f.close();
  if (DEBUG_MODE_IMU) Serial.println("Magnetometer Kalibrierung gespeichert.");
}

// -------------------- Mag Calibration --------------------
// Startfunktion MAG Kallibrierung
void start_mag_calibration(int durationSeconds) {
  mag_cal = false;  // Web-Flag quittieren
  mag_cal_running = true;
  mag_cal_duration = durationSeconds;
  mag_cal_start = millis();

  mxmin = mymin = mzmin = 1e6;
  mxmax = mymax = mzmax = -1e6;
  mag_cal_gotData = false;
  magProgress = 0;

  if (DEBUG_MODE_IMU)
    Serial.println("Mag calibration STARTED — rotate sensor...");
}

//Update Mag Kallibrierung
void update_mag_calibration() {
  if (!mag_cal_running) return;
  // 🔸 Sample einsammeln
  if (myICM.dataReady()) {
    myICM.getAGMT();
    float mx = myICM.agmt.mag.axes.x;
    float my = myICM.agmt.mag.axes.y;
    float mz = myICM.agmt.mag.axes.z;
    web_mx = mx;
    web_my = my;
    web_mz = mz;
    magProgress = magProgress + 1;

    mag_cal_gotData = true;
    if (mx < mxmin) mxmin = mx;
    if (mx > mxmax) mxmax = mx;
    if (my < mymin) mymin = my;
    if (my > mymax) mymax = my;
    if (mz < mzmin) mzmin = mz;
    if (mz > mzmax) mzmax = mz;
  }
  // 🔸 Zeit abgelaufen?
  if (millis() - mag_cal_start < (unsigned long)mag_cal_duration * 1000UL)
    return;
  // =============================
  // Abschluss
  // =============================
  mag_cal_running = false;
  if (!mag_cal_gotData) {
    if (DEBUG_MODE_IMU)
      Serial.println("Mag calibration FAILED: no data.");
    return;
  }
  if ((mxmax - mxmin) < 50 || (mymax - mymin) < 50 || (mzmax - mzmin) < 50) {

    if (DEBUG_MODE_IMU)
      Serial.println("Mag calibration FAILED: not enough movement.");
    return;
  }
  // Offsets
  mag_offset_x = (mxmin + mxmax) / 2.0f;
  mag_offset_y = (mymin + mymax) / 2.0f;
  mag_offset_z = (mzmin + mzmax) / 2.0f;
  save_mag_cal();
  if (DEBUG_MODE_IMU) {
    Serial.println("Mag calibration DONE");
    Serial.print("Offset X: ");
    Serial.println(mag_offset_x);
    Serial.print("Offset Y: ");
    Serial.println(mag_offset_y);
    Serial.print("Offset Z: ");
    Serial.println(mag_offset_z);
  }
}

// ------------------------------------------------------------
// IMU Einrichten
// ------------------------------------------------------------
void setup_imu(TwoWire &wirePort) {
  wirePort.begin(IMU_I2C_SDA, IMU_I2C_SCL);  // SDA, SCL
  wirePort.setClock(400000);
  int ad0_val = 1;  // feste Adresse 0x69

  while (!imu_initialized) {
    myICM.begin(wirePort, ad0_val);
    if (myICM.status == ICM_20948_Stat_Ok) {
      imu_initialized = true;
    } else {
      delay(500);
    }
  }
  filter.begin(100);  // 200 Hz Update-Rate
  // lade offset für mag und gyro
  load_mag_cal();
  load_gyro_cal();
}


// ------------------------------------------------------------
// IMU auswerten
// ------------------------------------------------------------
void read_imu() {
  if (!imu_initialized) return;
  if (!myICM.dataReady()) return;

  myICM.getAGMT();

  // --- Beschleunigung ---
  float ax = myICM.accX() / 1000.0;
  float ay = myICM.accY() / 1000.0;
  float az = myICM.accZ() / 1000.0;

  // --- Gyroskop ---
  float gx = (myICM.gyrX() - gyro_offset_x) * DEG_TO_RAD;
  float gy = (myICM.gyrY() - gyro_offset_y) * DEG_TO_RAD;
  float gz = (myICM.gyrZ() - gyro_offset_z) * DEG_TO_RAD;

  // --- Magnetometer Rohdaten + Skalierung + Hard-Iron ---
  float mx_raw = (myICM.agmt.mag.axes.x - mag_offset_x) * MAG_SCALE;
  float my_raw = (myICM.agmt.mag.axes.y - mag_offset_y) * MAG_SCALE;
  float mz_raw = (myICM.agmt.mag.axes.z - mag_offset_z) * MAG_SCALE;

  // --- Achsen-Mapping (Axis_Map 2) ---
  float mx_korr = mx_raw;
  float my_korr = -my_raw;
  float mz_korr = -mz_raw;

  // --- Madgwick Filter Update ---
  filter.update(gx, gy, gz, ax, ay, az, mx_korr, my_korr, mz_korr);

  // --- Ergebnisse direkt in autopilotdaten schreiben ---
  autopilotdaten.roll = filter.getRoll();
  autopilotdaten.pitch = filter.getPitch();
  autopilotdaten.kompass = -filter.getYaw();  // Kompass-Richtung korrigieren

  // --- Normalisierung Yaw 0..360° ---
  if (autopilotdaten.kompass < 0) autopilotdaten.kompass += 360.0;
  if (autopilotdaten.kompass > 360) autopilotdaten.kompass -= 360.0;
}
