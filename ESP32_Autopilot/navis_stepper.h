#ifndef NAVIS_STEPPER_H
#define NAVIS_STEPPER_H

#include <stdbool.h>
#include <stdint.h>
#include "Config.h"

// ============================================================
// STEPPER MOTOR API
// ============================================================
void navisStepperInit(void);             // Initialisiert Hardware-Pins und Treiber-Parameter
void navisStepperEnable(bool en);        // Aktiviert oder deaktiviert die Motor-Endstufe
void navisStepperMoveTo(int32_t pos);    // Fährt eine absolute Zielposition in Schritten an
void navisStepperStop(void);             // Stoppt die Bewegung unter Einhaltung der Bremsrampe
bool navisStepperIsRunning(void);        // Gibt true zurück, solange der Motor in Bewegung ist
bool navisStepperHasAlarm(void);         // Prüft den Alarm-Pin des Treibers auf Fehlerzustände
int32_t navisStepperGetPosition(void);   // Liefert den aktuellen Stand des Schrittzählers
void navisStepperLoop(void);             // Interne Task für die Echtzeit-Schrittgenerierung
void navisStepperUpdateEndlagen();       // Fragt die physischen Min/Max-Endschalter ab
void navisStepperHome(bool dirForward);  // Startet die Referenzfahrt in die gewählte Richtung

#endif  // NAVIS_STEPPER_H
