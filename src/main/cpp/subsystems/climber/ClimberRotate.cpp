#include "subsystems/climber/ClimberRotate.h"


ClimberRotate::ClimberRotate() {
    // Setup current limit, idle mode, and encoder factor
    rotation.SetSmartCurrentLimit(
            CONSTANTS::CLIMBER::ROTATION::HARD_CURRENT_LIMIT.to<int>());
    rotation.SetSecondaryCurrentLimit(
            CONSTANTS::CLIMBER::ROTATION::SOFT_CURRENT_LIMIT.to<double>());
    encoder.SetPositionConversionFactor(
            CONSTANTS::CLIMBER::ROTATION::TURN_TO_DEGREE.to<double>());
            // The encoder is now in units of degrees

    // Set up PID controller
    pid.SetP(CONSTANTS::CLIMBER::ROTATION::PID::P);
    pid.SetI(CONSTANTS::CLIMBER::ROTATION::PID::I);
    pid.SetD(CONSTANTS::CLIMBER::ROTATION::PID::D);
    pid.SetFF(CONSTANTS::CLIMBER::ROTATION::PID::FF);
    pid.SetIZone(CONSTANTS::CLIMBER::ROTATION::PID::I_ZONE);
    pid.SetOutputRange(CONSTANTS::CLIMBER::ROTATION::PID::MIN,
            CONSTANTS::CLIMBER::ROTATION::PID::MAX);
}

units::degree_t ClimberRotate::Get() {
    return angle;
}

void ClimberRotate::Set(units::degree_t angle) {
    // Save the angle
    ClimberRotate::angle = angle;

    // Update the PID controller
    pid.SetReference(angle.to<double>(),
            rev::CANSparkMaxLowLevel::ControlType::kPosition);
}

units::degree_t ClimberRotate::GetActual() {
    return (units::degree_t) encoder.GetPosition();
}

void ClimberRotate::Zero() {
    encoder.SetPosition(0.0);
}
