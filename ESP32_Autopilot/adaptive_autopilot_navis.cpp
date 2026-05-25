// adaptive_autopilot_navis.cpp
// -----------------------------------------------------
// NAVIS integrierter Autopilot
// - Kompass, GPS, Windmodus
// - Adaptive PID
// - Steppersteuerung
// - Integrierter WaveAnalyzer (inline)
// -----------------------------------------------------

#include <Arduino.h>
#include <math.h>
#include "adaptive_autopilot_navis.h"



// ======================================================
// ================= WAVE ANALYZER ======================
// ======================================================
static constexpr unsigned long WAVE_SAMPLE_INTERVAL = 50;
static constexpr float PEAK_THRESHOLD = 0.15f;
static constexpr unsigned long MIN_PEAK_DIST = 2000;
static constexpr float WAVE_TWO_PI = 6.28318530718f;

static unsigned long wave_lastTime = 0;
static unsigned long wave_lastPeakTime = 0;
static float wave_lastRoll = 0, wave_lastPitch = 0, wave_lastHeading = 0;
static float smoothDPitch = 0, smoothDRoll = 0;
static float wavePeriod = 7000.0f;
static float waveEnergy = 0;
static float seaEnergyLong = 0;
static float waveConfidence = 0;
static float predictedYaw = 0;

static float waveTargetHeading = 0.0f;

static SeaState currentSea = SEA_CALM;

static float wave_angleDiff(float a, float b) {
  float d = a - b;
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

static void setup_waveAnalyzer_internal() {
  waveTargetHeading = 0.0f;
  wave_lastTime = millis();
  wave_lastPeakTime = 0;
  seaEnergyLong = 0;
  waveConfidence = 0;
}

static void update_waveAnalyzer_internal() {
  unsigned long now = millis();
  if (now - wave_lastTime < WAVE_SAMPLE_INTERVAL) return;
  wave_lastTime = now;

  float dRoll = autopilotdaten.roll - wave_lastRoll;
  float dPitch = autopilotdaten.pitch - wave_lastPitch;

  wave_lastRoll = autopilotdaten.roll;
  wave_lastPitch = autopilotdaten.pitch;
  wave_lastHeading = autopilotdaten.kompass;

  smoothDPitch = 0.7f * dPitch + 0.3f * smoothDPitch;
  smoothDRoll = 0.7f * dRoll + 0.3f * smoothDRoll;

  waveEnergy = fabsf(smoothDPitch) + fabsf(smoothDRoll);
  seaEnergyLong = 0.98f * seaEnergyLong + 0.02f * waveEnergy;

  if (seaEnergyLong < 0.3f) currentSea = SEA_CALM;
  else if (seaEnergyLong < 0.9f) currentSea = SEA_MODERATE;
  else currentSea = SEA_HEAVY;

  if (smoothDPitch < 0 && dPitch > 0) {
    if (fabsf(dPitch) > PEAK_THRESHOLD && (now - wave_lastPeakTime > MIN_PEAK_DIST)) {
      unsigned long p = now - wave_lastPeakTime;
      if (p < 15000)
        wavePeriod = 0.8f * wavePeriod + 0.2f * (float)p;

      wave_lastPeakTime = now;
    }
  }

  float basePred = smoothDPitch + (smoothDRoll * 0.7f);
  float headingError = wave_angleDiff(autopilotdaten.kompass, waveTargetHeading);

  float phaseCorr = 0;
  if (wave_lastPeakTime > 0) {
    float phase = (float)(now - wave_lastPeakTime) / wavePeriod;
    phaseCorr = sinf((phase + 0.15f) * WAVE_TWO_PI) * 0.5f;
  }

  waveConfidence = fminf(seaEnergyLong * 2.0f, 1.0f);
  predictedYaw = (basePred * (1.0f + phaseCorr)) * waveConfidence;

  if (fabsf(headingError) > 2.0f)
    predictedYaw *= 0.3f;
}

// Getter
float navis_getPredictedYaw() {
  return predictedYaw;
}
float navis_getWaveConfidence() {
  return waveConfidence;
}
SeaState navis_getSeaState() {
  return currentSea;
}

// ======================================================
// ================ ADAPTIVE PID CORE ==================
// ======================================================

static float baseP, baseI, baseD;
static float pGain = 1.0f, iGain = 1.0f, dGain = 1.0f;
static float integrator = 0.0f;
static float lastRoll = 0.0f, lastPitch = 0.0f, lastHeading = 0.0f;
static float costFiltered = 0.0f, lastCost = 0.0f;
static float internalTargetHeading = 0.0f;

enum CoreSeaState { CORE_CALM = 0,
                    CORE_WAVE = 1,
                    CORE_GUST = 2,
                    CORE_UNKNOWN = 3 };
static CoreSeaState currentState = CORE_CALM;
static float seaConfidenceCore = 0.0f;

struct AdaptiveStat {
  float mean = 0.0f, var = 1.0f;
  void update(float x, float a) {
    float d = x - mean;
    mean += a * d;
    var += a * ((d * d) - var);
  }
  float stddev() const {
    return sqrtf(var + 0.0001f);
  }
};

static AdaptiveStat driftStat, rollStat, windStat, yawStat;

// Hilfsfunktion
static float core_angleDiff(float a, float b) {
  float d = a - b;
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

// SeaState Update
static void updateSeaStateCore(const Autopilot_Sensoren& d, float dRoll, float dHeading) {
  driftStat.update(core_angleDiff(d.gps_kurs, d.kompass), 0.01f);
  rollStat.update(fabsf(dRoll), 0.01f);
  windStat.update(d.windspeed_gemessen, 0.01f);
  yawStat.update(fabsf(dHeading), 0.01f);

  float scores[4] = {
    fabsf(fabsf(dRoll) - rollStat.mean) / rollStat.stddev(),
    fabsf(d.windspeed_gemessen - windStat.mean) / windStat.stddev(),
    0.0f,
    fabsf(fabsf(dHeading) - yawStat.mean) / yawStat.stddev()
  };
  int maxIdx = 0;
  for (int i = 1; i < 4; i++)
    if (scores[i] > scores[maxIdx]) maxIdx = i;
  currentState = static_cast<CoreSeaState>(maxIdx + 1);
  seaConfidenceCore = 0.8f;
}

// Adaptive PID Anpassung
static void adaptPID(float error, float yawRate, float rudder) {
  float cost = (error * error) + (0.3f * yawRate * yawRate);
  costFiltered = 0.99f * costFiltered + 0.01f * cost;
  if (costFiltered > lastCost) {
    pGain -= 0.0001f;
    dGain -= 0.0001f;
  } else {
    pGain += 0.0001f;
    dGain += 0.0001f;
  }
  pGain = constrain(pGain, 0.5f, 2.5f);
  lastCost = costFiltered;
}

// Setup / Target
void setup_adaptiveAutopilot(float P, float I, float D) {
  baseP = P;
  baseI = I;
  baseD = D;
  pGain = iGain = dGain = 1.0f;
  integrator = 0.0f;
}

void adaptiveAutopilot_setTarget(float h) {
  internalTargetHeading = h;
}

void adaptiveAutopilot_reset() {
  pGain = iGain = dGain = 1.0f;
  integrator = 0.0f;
}

// Adaptive PID Update
CoreOutput adaptiveAutopilot_update(const Autopilot_Sensoren& data) {
  CoreOutput out = {};

  float dRoll = data.roll - lastRoll;
  float dPitch = data.pitch - lastPitch;
  float dHeading = core_angleDiff(data.kompass, lastHeading);

  updateSeaStateCore(data, dRoll, dHeading);
  update_waveAnalyzer_internal();
  float waveComp = navis_getPredictedYaw();

  float error = core_angleDiff(internalTargetHeading, data.kompass);

  float modP = 1.0f, modD = 1.0f;
  if (currentState == CORE_WAVE) {
    modP = 0.7f;
    modD = 1.8f;
  }
  if (currentState == CORE_GUST) { modP = 1.3f; }

  float P = baseP * pGain * modP;
  float D = baseD * dGain * modD;
  integrator = constrain(integrator + (error * 0.1f), -15.0f, 15.0f);

  float pidOut = (P * error) + (baseI * iGain * integrator) - (D * dHeading);
  float finalRudder = pidOut + (waveComp * navis_getWaveConfidence());

  adaptPID(error, dHeading, finalRudder);

  out.rudderOutput = constrain(finalRudder, -45.0f, 45.0f);
  out.predictedYaw = waveComp;
  out.seaState = currentSea;
  out.seaConfidence = seaConfidenceCore;
  out.pGain = pGain;
  out.iGain = iGain;
  out.dGain = dGain;

  lastRoll = data.roll;
  lastPitch = data.pitch;
  lastHeading = data.kompass;

  return out;
}

// ======================================================
// ================= AUTOPILOT ==========================
// ======================================================

static double targetHeading = 0.0;
static double targetWindAngle = 0.0;

static long targetSteps = 250000;
static const long centerPosition = 250000;
static const long maxSteps = 500000;

static unsigned long lastControlTime = 0;
static const unsigned long controlInterval = 100;

static double normalize360(double a) {
  while (a >= 360.0) a -= 360.0;
  while (a < 0.0) a += 360.0;
  return a;
}

void setAutopilotMode(int mode) {
  autopilotSystem.currentMode = mode;
  autopilotSystem.offsetWinkel = 0;
  switch (mode) {
    case 1: targetHeading = autopilotdaten.kompass; break;
    case 3: targetWindAngle = autopilotdaten.winddir_gemessen; break;
  }
}

static float getSetpointByMode() {
  float baseHeading = 0.0f;
  switch (autopilotSystem.currentMode) {
    case 1:  // Kompass
      baseHeading = targetHeading;
      break;
    case 2:
      {  // GPS Navigation
        baseHeading = calculateBearing(
          autopilotdaten.gps_lat,
          autopilotdaten.gps_lon,
          autopilotSystem.autopilot_lat,
          autopilotSystem.autopilot_lon);
        break;
      }
    case 3:  // Wind
      baseHeading = targetWindAngle;
      break;

    default:
      baseHeading = autopilotdaten.kompass;
      break;
  }
  return normalize360(baseHeading + (float)autopilotSystem.offsetWinkel);
}

// ======================================================
// SETUP
// ======================================================

void setup_autopilot() {
  navisStepperInit();
  setup_adaptiveAutopilot(autopilotSystem.P_base, autopilotSystem.I_base, autopilotSystem.D_base);
  setup_waveAnalyzer_internal();
  targetSteps = centerPosition;
}

// ======================================================
// Hilfsfunktion für adaptiveAutopilotNavis_loop Winkel berechnen
// ======================================================
double angleDiff(double a, double b) {
  double d = a - b;
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

// ==================================================
// GPS Bearing berechnen (Grad 0..360)
// ==================================================
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  float phi1 = lat1 * DEG_TO_RAD;
  float phi2 = lat2 * DEG_TO_RAD;
  float dLon = (lon2 - lon1) * DEG_TO_RAD;
  float y = sinf(dLon) * cosf(phi2);
  float x = cosf(phi1) * sinf(phi2) - sinf(phi1) * cosf(phi2) * cosf(dLon);
  float bearing = atan2f(y, x) * RAD_TO_DEG;
  if (bearing < 0) bearing += 360.0f;
  return bearing;
}

// ======================================================
// MAIN LOOP 10 Hz
// ======================================================
void adaptiveAutopilotNavis_loop() {
  // Event-Auswertung Autopilot Webserver
  // ----- Moduswechsel -----
  if (autopilotEvents.modeChanged) {
    autopilotEvents.modeChanged = false;
    setAutopilotMode(autopilotSystem.currentMode);
  }
  // ----- Manuelle Pinne -----
  if (autopilotEvents.manualChanged) {
    autopilotEvents.manualChanged = false;
    switch (autopilotSystem.currentPinne) {
      //case 0: navisStepperStop(); break;
      //case 1: pinneEinfahren(); break;
      //case 2: pinneAusfahren(); break;
    }
  }

  unsigned long now = millis();
  if (now - lastControlTime < controlInterval) return;
  lastControlTime = now;

  update_waveAnalyzer_internal();

  float setpoint = getSetpointByMode();
  waveTargetHeading = setpoint;

  adaptiveAutopilot_setTarget(setpoint);
  CoreOutput coreOutput = adaptiveAutopilot_update(autopilotdaten);

  float waveYaw = navis_getPredictedYaw();
  float confidence = navis_getWaveConfidence();

  float waveGain = 0.2f;
  if (currentSea == SEA_MODERATE) waveGain = 0.35f;
  if (currentSea == SEA_HEAVY) waveGain = 0.55f;
  waveGain *= confidence;

  float rudderWithWave = coreOutput.rudderOutput - (waveYaw * waveGain);

  double maxRuderWinkel = fabs(angleDiff(rudderConfig.maxAngle, rudderConfig.minAngle)) / 2.0;
  float ratio = (float)centerPosition / maxRuderWinkel;

  long newTargetSteps = constrain(centerPosition + (long)(rudderWithWave * ratio), 0, maxSteps);
  if (autopilotSystem.invertPinne) newTargetSteps = 2 * centerPosition - newTargetSteps;

  targetSteps = newTargetSteps;
}

// ======================================================
// MOTOR LOOP
// ======================================================
void adaptiveAutopilotMotor_loop() {
  long currentPos = navisStepperGetPosition();
  if (targetSteps == currentPos) return;
  // Schrittweite über FastAccelStepper bewegen
  navisStepperMoveTo(targetSteps);
  // Optional: Motor enable automatisch
  navisStepperEnable(true);
  // Position für Debug oder GUI speichern
  autopilotSystem.stepperPosition = currentPos;
}
