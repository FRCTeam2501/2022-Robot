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
            CONSTANTS::CLIMBER::WINCH::HARD_CURRENT_LIMIT.to<int>());
    leftWinch->SetSecondaryCurrentLimit(
            CONSTANTS::CLIMBER::WINCH::SOFT_CURRENT_LIMIT.to<double>());

    rightWinch->SetSmartCurrentLimit(
            CONSTANTS::CLIMBER::WINCH::HARD_CURRENT_LIMIT.to<int>());
    rightWinch->SetSecondaryCurrentLimit(
            CONSTANTS::CLIMBER::WINCH::SOFT_CURRENT_LIMIT.to<double>());

    rotation->SetSmartCurrentLimit(
            CONSTANTS::CLIMBER::ROTATION::HARD_CURRENT_LIMIT.to<int>());
    rotation->SetSecondaryCurrentLimit(
            CONSTANTS::CLIMBER::ROTATION::SOFT_CURRENT_LIMIT.to<double>());
    rotation->GetEncoder().SetPositionConversionFactor(
            CONSTANTS::CLIMBER::ROTATION::TURN_TO_DEGREES.to<double>());


    // Set up PID controllers
    rotation->GetPIDController().SetP(
            CONSTANTS::CLIMBER::ROTATION::PID::P);
    rotation->GetPIDController().SetI(
            CONSTANTS::CLIMBER::ROTATION::PID::I);
    rotation->GetPIDController().SetD(
            CONSTANTS::CLIMBER::ROTATION::PID::D);
    rotation->GetPIDController().SetFF(
            CONSTANTS::CLIMBER::ROTATION::PID::FF);
    rotation->GetPIDController().SetIZone(
            CONSTANTS::CLIMBER::ROTATION::PID::I_ZONE);
    rotation->GetPIDController().SetOutputRange(
            CONSTANTS::CLIMBER::ROTATION::PID::MIN,
            CONSTANTS::CLIMBER::ROTATION::PID::MAX
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

units::degree_t Climber::GetRotationAngle() {
    return angle;
}

void Climber::SetRotationAngle(units::degree_t angle) {
    // Save the angle
    Climber::angle = angle;

    // Update the PID controller
    rotation->GetPIDController().SetReference(
        angle.to<double>(), rev::CANSparkMaxLowLevel::ControlType::kPosition);
}