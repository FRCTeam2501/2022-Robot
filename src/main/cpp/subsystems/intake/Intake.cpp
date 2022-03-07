#include "subsystems/intake/Intake.h"


Intake::Intake() {
    motor.SetSmartCurrentLimit(
            CONSTANTS::INTAKE::MOTOR::HARD_CURRENT_LIMIT.to<double>());
    motor.SetSecondaryCurrentLimit(
            CONSTANTS::INTAKE::MOTOR::SOFT_CURRENT_LIMIT.to<double>());
    motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    motor.SetInverted(true);
}

units::degree_t Intake::GetAngle() {
    return rotation.Get();
}

void Intake::SetAngle(units::degree_t angle) {
    rotation.Set(angle);
}

double Intake::GetMotor() {
    return motor.Get();
}

void Intake::SetMotor(double power) {
    motor.Set(power);
}

void Intake::ZeroEncoder(units::degree_t angle) {
    rotation.Zero(angle);
}
