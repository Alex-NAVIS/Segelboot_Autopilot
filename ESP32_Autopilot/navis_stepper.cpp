#include "navis_stepper.h"
#include <Arduino.h>
#include <FastAccelStepper.h>

// ==============================
// interne Objekte
// ==============================
static FastAccelStepperEngine engine = FastAccelStepperEngine();
static FastAccelStepper *stepper = NULL;

// ==============================
// Reverenzfahrt homing
// ==============================
enum NavisHomingState {
  NAVIS_HOME_IDLE = 0,
  NAVIS_HOME_RUNNING,
  NAVIS_HOME_FINISH
};

static NavisHomingState homingState = NAVIS_HOME_IDLE;
static bool homingDirForward = false;

// ==============================
// Init
// ==============================
void navisStepperInit(void) {
  // ---------- Pin Setup ----------
  pinMode(NAVIS_PIN_STEPPER_STEP, OUTPUT);
  digitalWrite(NAVIS_PIN_STEPPER_STEP, LOW);
  pinMode(NAVIS_PIN_STEPPER_DIR, OUTPUT);
  digitalWrite(NAVIS_PIN_STEPPER_DIR, LOW);
  pinMode(NAVIS_PIN_STEPPER_EN, OUTPUT);
  digitalWrite(NAVIS_PIN_STEPPER_EN, LOW);  // Motor disabled
  pinMode(NAVIS_PIN_STEPPER_ALARM, INPUT_PULLUP);

  // Endlagenschalter Pins
  pinMode(NAVIS_PIN_STEPPER_END_MIN, INPUT_PULLUP);
  pinMode(NAVIS_PIN_STEPPER_END_MAX, INPUT_PULLUP);

  // Flags und Position initialisieren
  autopilotSystem.endlageMin = false;
  autopilotSystem.endlageMax = false;
  autopilotSystem.stepperPosition = 0;

  // ---------- FastAccelStepper ----------
  engine.init();
  stepper = engine.stepperConnectToPin(NAVIS_PIN_STEPPER_STEP);
  if (stepper) {
    stepper->setDirectionPin(NAVIS_PIN_STEPPER_DIR);
    stepper->setEnablePin(NAVIS_PIN_STEPPER_EN);
    stepper->setAutoEnable(false);
    stepper->setSpeedInHz(autopilotSystem.maxStepperSpeed);
    stepper->setAcceleration(autopilotSystem.maxStepperAccel);
    stepper->setCurrentPosition(0);
  }
  navisStepperHome(false);  // Fahre in die minimale Endlage
}

// ==============================
// Stepper Endlagen prüfen & Position aktualisieren
// ==============================
void navisStepperUpdateEndlagen() {
  if (!stepper) return;  // Stepper nicht initialisiert

  // --- Endlagen prüfen ---
  bool minTriggered = (digitalRead(NAVIS_PIN_STEPPER_END_MIN) == LOW);
  bool maxTriggered = (digitalRead(NAVIS_PIN_STEPPER_END_MAX) == LOW);

  // --- Flags setzen ---
  autopilotSystem.endlageMin = minTriggered;
  autopilotSystem.endlageMax = maxTriggered;

  // --- Stepper stoppen, falls Endlage erreicht ---
  if (autopilotSystem.endlageMin) {
    stepper->stopMove();
    stepper->setCurrentPosition(0);       // interner Zähler auf 0
    autopilotSystem.stepperPosition = 0;  // Autopilot-Variable auf 0
  } else if (autopilotSystem.endlageMax) {
    stepper->stopMove();
    autopilotSystem.stepperPosition = stepper->getCurrentPosition();
  }

  // --- aktuelle Position synchronisieren ---
  // Nur aktualisieren, wenn keine Endlage aktiv ist
  if (!autopilotSystem.endlageMin && !autopilotSystem.endlageMax) {
    autopilotSystem.stepperPosition = stepper->getCurrentPosition();
  }
}


// ==============================
// Enable
// ==============================
void navisStepperEnable(bool en) {
  digitalWrite(NAVIS_PIN_STEPPER_EN, en ? HIGH : LOW);
}

// ==============================
// Move
// ==============================
void navisStepperMoveTo(int32_t positionSteps) {
  if (!stepper) return;

  int32_t currentPos = stepper->getCurrentPosition();

  // --- Softlimits prüfen ---
  if (autopilotSystem.endlageMin && positionSteps < currentPos) {
    // Endlage Minimum erreicht, Bewegung Richtung Minimum blockieren
    return;
  }
  if (autopilotSystem.endlageMax && positionSteps > currentPos) {
    // Endlage Maximum erreicht, Bewegung Richtung Maximum blockieren
    return;
  }

  // --- Ziel innerhalb der Softlimits ---
  stepper->moveTo(positionSteps);
}

// ==============================
// Stop
// ==============================
void navisStepperStop(void) {
  if (!stepper) return;
  stepper->stopMove();
}

// ==============================
// Stepper Homing
// ==============================
void navisStepperHome(bool dirForward) {
  if (!stepper) return;
  homingDirForward = dirForward;
  homingState = NAVIS_HOME_RUNNING;

  navisStepperEnable(true);

  // langsame sichere Homing-Speed
  stepper->setSpeedInHz(autopilotSystem.maxStepperSpeed / 4);

  if (dirForward) stepper->runForward();
  else stepper->runBackward();
}

// ==============================
// Status
// ==============================
bool navisStepperIsRunning(void) {
  if (!stepper) return false;
  return stepper->isRunning();
}

int32_t navisStepperGetPosition(void) {
  if (!stepper) return 0;
  return stepper->getCurrentPosition();
}

bool navisStepperHasAlarm(void) {
  // viele Closed-Loop Treiber ziehen ALARM auf LOW
  return digitalRead(NAVIS_PIN_STEPPER_ALARM) == LOW;
}

// ==============================
// Loop
// ==============================
void navisStepperLoop(void) {
  if (!stepper) return;

  // Endlagen immer aktuell halten
  navisStepperUpdateEndlagen();
  
  // =====================================
  // Manual Jog über currentPinne
  // =====================================
  if (autopilotSystem.currentPinne != 0) {
    navisStepperEnable(true);
    // ----- EINFAHREN -----
    if (autopilotSystem.currentPinne == 1) {
      if (!autopilotSystem.endlageMin) {
        stepper->setSpeedInHz(autopilotSystem.maxStepperSpeed / 2);
        stepper->runBackward();  // ggf. Richtung prüfen
      } else {
        stepper->stopMove();
      }
    }
    // ----- AUSFAHREN -----
    else if (autopilotSystem.currentPinne == 2) {
      if (!autopilotSystem.endlageMax) {
        stepper->setSpeedInHz(autopilotSystem.maxStepperSpeed / 2);
        stepper->runForward();  // ggf. Richtung prüfen
      } else {
        stepper->stopMove();
      }
    }
    return;  // ⭐ Manual hat absolute Priorität
  } else {
    // wenn manual losgelassen wurde → sicher stoppen
    stepper->stopMove();
  }

  switch (homingState) {
    case NAVIS_HOME_IDLE:
      break;

    case NAVIS_HOME_RUNNING:
      {
        bool reached =
          (homingDirForward && autopilotSystem.endlageMax) || (!homingDirForward && autopilotSystem.endlageMin);

        if (reached) {
          stepper->stopMove();  // gewollt: sanft abbremsen
          homingState = NAVIS_HOME_FINISH;
        }
      }
      break;

    case NAVIS_HOME_FINISH:
      {
        if (!stepper->isRunning()) {
          stepper->setCurrentPosition(0);
          autopilotSystem.stepperPosition = 0;

          // Speed zurücksetzen
          stepper->setSpeedInHz(autopilotSystem.maxStepperSpeed);

          navisStepperEnable(false);
          homingState = NAVIS_HOME_IDLE;
        }
      }
      break;
  }
}
