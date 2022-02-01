#include "subsystems/ClimberWinches.h"
#include "Constants.h"


ClimberWinches::ClimberWinches() {
    // Create motor controllers
    winch = new rev::CANSparkMax(
        CONSTANTS::MOTORS::CAN::CLIMBER_WINCH_ID,
        rev::CANSparkMax::MotorType::kBrushless
    );

    // Setup current limits, idle modes, and encoder factors
    winch->SetSmartCurrentLimit(
            CONSTANTS::CLIMBER::WINCH::HARD_CURRENT_LIMIT.to<int>());
    winch->SetSecondaryCurrentLimit(
            CONSTANTS::CLIMBER::WINCH::SOFT_CURRENT_LIMIT.to<double>());
}

ClimberWinches::~ClimberWinches() {
    delete winch;
}

void ClimberWinches::SetSpeed(double speed) {
    winch->Set(speed);
}