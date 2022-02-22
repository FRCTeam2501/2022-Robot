#include "subsystems/climber/ClimberPin.h"


ClimberPin::ClimberPin() {
    pin = new frc::Solenoid(frc::PneumaticsModuleType::CTREPCM,
            CONSTANTS::SOLENOIDS::CLIMBER_PIN_ID);
}

void ClimberPin::Open() {
    pin->Set(true);
}

void ClimberPin::Close() {
    pin->Set(false);
}

bool ClimberPin::Get() {
    return pin->Get();
}
