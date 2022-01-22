#include "subsystems/ClimberRotate.h"
#include "Constants.h"


ClimberRotate::ClimberRotate() {
    // Create motor controllers
    rotation = new rev::CANSparkMax(
        CONSTANTS::MOTORS::CAN::CLIMBER_ROTATION_ID,
        rev::CANSparkMax::MotorType::kBrushless
    );


    // Setup current limit, idle mode, and encoder factor
    rotation->SetSmartCurrentLimit(
            CONSTANTS::CLIMBER::ROTATION::HARD_CURRENT_LIMIT.to<int>());
    rotation->SetSecondaryCurrentLimit(
            CONSTANTS::CLIMBER::ROTATION::SOFT_CURRENT_LIMIT.to<double>());
    rotation->GetEncoder().SetPositionConversionFactor(
            CONSTANTS::CLIMBER::ROTATION::TURN_TO_DEGREES.to<double>());


    // Set up PID controller
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

ClimberRotate::~ClimberRotate() {
    delete rotation;
}

units::degree_t ClimberRotate::GetAngle() {
    return angle;
}

void ClimberRotate::SetAngle(units::degree_t angle) {
    // Save the angle
    ClimberRotate::angle = angle;

    // Update the PID controller
    rotation->GetPIDController().SetReference(
        angle.to<double>(), rev::CANSparkMaxLowLevel::ControlType::kPosition);
}