// adaptive_autopilot_navis.h
// -----------------------------------------------------
// NAVIS Autopilot Header
// Enthält: Adaptive PID, WaveAnalyzer, Steppersteuerung
// -----------------------------------------------------

#ifndef ADAPTIVE_AUTOPILOT_NAVIS_H
#define ADAPTIVE_AUTOPILOT_NAVIS_H

#include <Arduino.h>
#include "Autopilot_Data.h"
#include "navis_stepper.h"


// -------------------------
// Interner SeaState Typ
// -------------------------
enum SeaState {
    SEA_CALM = 0,
    SEA_MODERATE = 1,
    SEA_HEAVY = 2
};

// Getter
// ================= WAVE ANALYZER =====================
float navis_getPredictedYaw();
float navis_getWaveConfidence();
SeaState navis_getSeaState();

// ================= ADAPTIVE PID CORE =================
struct CoreOutput {
    float rudderOutput;
    float predictedYaw;
    SeaState seaState;
    float seaConfidence;
    float pGain;
    float iGain;
    float dGain;
};

// Setup & Reset
void setup_adaptiveAutopilot(float P, float I, float D);
void adaptiveAutopilot_setTarget(float heading);
void adaptiveAutopilot_reset();

// Update (Haupt-PID Berechnung)
CoreOutput adaptiveAutopilot_update(const Autopilot_Sensoren& data);

// ================= AUTOPILOT =========================
// Steuerung
void setAutopilotMode(int mode);
void setup_autopilot();
void adaptiveAutopilotNavis_loop();
void adaptiveAutopilotMotor_loop();
// GPS Zielkoordinaten zur berechnung des GPS Steuerkurs
float calculateBearing(float lat1, float lon1, float lat2, float lon2);
#endif // ADAPTIVE_AUTOPILOT_NAVIS_H
