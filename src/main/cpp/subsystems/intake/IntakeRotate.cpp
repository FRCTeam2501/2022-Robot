#include "subsystems/intake/IntakeRotate.h"


IntakeRotate::IntakeRotate() {
    // Setup current limit, idle mode, and encoder factor
    rotation.SetSmartCurrentLimit(
            CONSTANTS::INTAKE::ROTATION::HARD_CURRENT_LIMIT.to<int>());
    rotation.SetSecondaryCurrentLimit(
            CONSTANTS::INTAKE::ROTATION::SOFT_CURRENT_LIMIT.to<double>());
    encoder.SetPositionConversionFactor(
            CONSTANTS::INTAKE::ROTATION::TURN_TO_DEGREE.to<double>());
            // The encoder is now in units of degrees

    // Set up PID controller
    pid.SetP(CONSTANTS::INTAKE::ROTATION::PID::P);
    pid.SetI(CONSTANTS::INTAKE::ROTATION::PID::I);
    pid.SetD(CONSTANTS::INTAKE::ROTATION::PID::D);
    pid.SetFF(CONSTANTS::INTAKE::ROTATION::PID::FF);
    pid.SetIZone(CONSTANTS::INTAKE::ROTATION::PID::I_ZONE);
    pid.SetOutputRange(CONSTANTS::INTAKE::ROTATION::PID::MIN,
            CONSTANTS::INTAKE::ROTATION::PID::MAX);
}

units::degree_t IntakeRotate::Get() {
    return angle;
}

void IntakeRotate::Set(units::degree_t angle) {
    // Save the angle
    IntakeRotate::angle = angle;

    // Update the PID controller
    pid.SetReference(angle.to<double>(),
            rev::CANSparkMaxLowLevel::ControlType::kPosition);
}

units::degree_t IntakeRotate::GetActual() {
    return (units::degree_t) encoder.GetPosition();
}

void IntakeRotate::Zero() {
    encoder.SetPosition(0.0);
}
