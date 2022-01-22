#include "subsystems/ClimberWinches.h"
#include "Constants.h"


ClimberWinches::ClimberWinches() {
    // Create motor controllers
    leftWinch = new rev::CANSparkMax(
        CONSTANTS::MOTORS::CAN::CLIMBER_LEFT_WINCH_ID,
        rev::CANSparkMax::MotorType::kBrushless
    );
    rightWinch = new rev::CANSparkMax(
        CONSTANTS::MOTORS::CAN::CLIMBER_RIGHT_WINCH_ID,
        rev::CANSparkMax::MotorType::kBrushless
    );

    winches = new frc::MotorControllerGroup(*leftWinch, *rightWinch);


    // Setup current limits, idle modes, and encoder factors
    leftWinch->SetSmartCurrentLimit(
            CONSTANTS::CLIMBER::WINCH::HARD_CURRENT_LIMIT.to<int>());
    leftWinch->SetSecondaryCurrentLimit(
            CONSTANTS::CLIMBER::WINCH::SOFT_CURRENT_LIMIT.to<double>());

    rightWinch->SetSmartCurrentLimit(
            CONSTANTS::CLIMBER::WINCH::HARD_CURRENT_LIMIT.to<int>());
    rightWinch->SetSecondaryCurrentLimit(
            CONSTANTS::CLIMBER::WINCH::SOFT_CURRENT_LIMIT.to<double>());
}

ClimberWinches::~ClimberWinches() {
    delete leftWinch;
    delete rightWinch;

    delete winches;
}

void ClimberWinches::SetSpeed(double speed) {
    winches->Set(speed);
}