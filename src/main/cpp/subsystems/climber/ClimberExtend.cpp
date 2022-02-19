#include "subsystems/climber/ClimberExtend.h"
#include "Constants.h"


ClimberExtend::ClimberExtend() {
    // Setup current limits, idle modes, and encoder factors
    winch.SetSmartCurrentLimit(
            CONSTANTS::CLIMBER::EXTEND::HARD_CURRENT_LIMIT.to<int>());
    winch.SetSecondaryCurrentLimit(
            CONSTANTS::CLIMBER::EXTEND::SOFT_CURRENT_LIMIT.to<double>());
    winch.GetEncoder().SetPositionConversionFactor(
            CONSTANTS::CLIMBER::EXTEND::GEARBOX_RATIO.to<double>());
            // The encoder is now in units of drum turns

    // Set up PID controller
    pid.SetP(CONSTANTS::CLIMBER::EXTEND::PID::P);
    pid.SetI(CONSTANTS::CLIMBER::EXTEND::PID::I);
    pid.SetD(CONSTANTS::CLIMBER::EXTEND::PID::D);
    pid.SetFF(CONSTANTS::CLIMBER::EXTEND::PID::FF);
    pid.SetIZone(CONSTANTS::CLIMBER::EXTEND::PID::I_ZONE);
    pid.SetOutputRange(CONSTANTS::CLIMBER::EXTEND::PID::MIN,
            CONSTANTS::CLIMBER::EXTEND::PID::MAX);
}

units::meter_t ClimberExtend::Get() {
    return distance;
}

void ClimberExtend::Set(units::meter_t distance) {
    // Save the distance
    ClimberExtend::distance = distance;

    // Update the PID controller
    pid.SetReference(GetTurns(distance).to<double>(),
                rev::CANSparkMaxLowLevel::ControlType::kPosition);
}

units::turn_t ClimberExtend::GetActual() {
    return (units::turn_t) encoder.GetPosition();
}

units::turn_t ClimberExtend::GetTurns(units::meter_t distance) {
    // Note: This only accounts for a single circumference,
    //  not the changing circumference of the pulley system
    return distance * 1_tr / CONSTANTS::CLIMBER::EXTEND::PULLEY_CIRCUMFERENCE;
}

units::meter_t ClimberExtend::GetDistance(units::turn_t turns) {
    // Note: This only accounts for a single circumference,
    //  not the changing circumference of the pulley system
    return turns * CONSTANTS::CLIMBER::EXTEND::PULLEY_CIRCUMFERENCE / 1_tr;
}