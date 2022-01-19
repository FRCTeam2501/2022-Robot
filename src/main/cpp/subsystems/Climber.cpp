#include "subsystems/Climber.h"
#include "Constants.h"


Climber::Climber() {
    // Create motor controllers
    leftWinch = new rev::CANSparkMax(
        CONSTANTS::MOTORS::CAN::CLIMBER_LEFT_WINCH_ID,
        rev::CANSparkMax::MotorType::kBrushless
    );
    rightWinch = new rev::CANSparkMax(
        CONSTANTS::MOTORS::CAN::CLIMBER_RIGHT_WINCH_ID,
        rev::CANSparkMax::MotorType::kBrushless
    );
    rotation = new rev::CANSparkMax(
        CONSTANTS::MOTORS::CAN::CLIMBER_ROTATION_ID,
        rev::CANSparkMax::MotorType::kBrushless
    );

    winches = new frc::MotorControllerGroup(*leftWinch, *rightWinch);


    // Setup current limits, idle modes, and encoder factors
    leftWinch->SetSmartCurrentLimit(
        CONSTANTS::CLIMBER::HARD_CURRENT_LIMIT.to<int>());
    leftWinch->SetSecondaryCurrentLimit(
        CONSTANTS::CLIMBER::SOFT_CURRENT_LIMIT.to<double>());

    rightWinch->SetSmartCurrentLimit(
        CONSTANTS::CLIMBER::HARD_CURRENT_LIMIT.to<int>());
    rightWinch->SetSecondaryCurrentLimit(
        CONSTANTS::CLIMBER::SOFT_CURRENT_LIMIT.to<double>());

    rotation->SetSmartCurrentLimit(
        CONSTANTS::CLIMBER::HARD_CURRENT_LIMIT.to<int>());
    rotation->SetSecondaryCurrentLimit(
        CONSTANTS::CLIMBER::SOFT_CURRENT_LIMIT.to<double>());
    rotation->GetEncoder().SetPositionConversionFactor(
        CONSTANTS::CLIMBER::TURN_TO_DEGREES.to<double>());


    // Set up PID controllers
    rotation->GetPIDController().SetP(CONSTANTS::CLIMBER::CONTROL::P);
    rotation->GetPIDController().SetI(CONSTANTS::CLIMBER::CONTROL::I);
    rotation->GetPIDController().SetD(CONSTANTS::CLIMBER::CONTROL::D);
    rotation->GetPIDController().SetFF(CONSTANTS::CLIMBER::CONTROL::F);
    rotation->GetPIDController().SetIZone(CONSTANTS::CLIMBER::CONTROL::I_ZONE);
    rotation->GetPIDController().SetOutputRange(
        CONSTANTS::CLIMBER::CONTROL::MIN,
        CONSTANTS::CLIMBER::CONTROL::MAX
    );
}

Climber::~Climber() {
    delete leftWinch;
    delete rightWinch;
    delete rotation;

    delete winches;
}

void Climber::SetWinchSpeed(double speed) {
    winches->Set(speed);
}

void Climber::SetRotationAngle(double angle) {
    rotation->GetPIDController().SetReference(
        angle, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}