#include "subsystems/climber/ClimberExtend.h"
#include "Constants.h"


ClimberExtend::ClimberExtend() {
    // Setup current limits, idle modes, and encoder factors
    winch.SetSmartCurrentLimit(
            CONSTANTS::CLIMBER::EXTEND::HARD_CURRENT_LIMIT.to<int>());
    winch.SetSecondaryCurrentLimit(
            CONSTANTS::CLIMBER::EXTEND::SOFT_CURRENT_LIMIT.to<double>());
    winch.GetEncoder().SetPositionConversionFactor(
            CONSTANTS::CLIMBER::EXTEND::TURN_TO_METER.to<double>());
            // The encoder is now in units of meters

    // Set up PID controller
    pid.SetP(CONSTANTS::CLIMBER::EXTEND::PID::P);
    pid.SetI(CONSTANTS::CLIMBER::EXTEND::PID::I);
    pid.SetD(CONSTANTS::CLIMBER::EXTEND::PID::D);
    pid.SetFF(CONSTANTS::CLIMBER::EXTEND::PID::FF);
    pid.SetIZone(CONSTANTS::CLIMBER::EXTEND::PID::I_ZONE);
    pid.SetOutputRange(CONSTANTS::CLIMBER::EXTEND::PID::MIN,
            CONSTANTS::CLIMBER::EXTEND::PID::MAX);
}

units::meter_t ClimberExtend::GetExtension() {
    return distance;
}

void ClimberExtend::SetExtension(units::meter_t distance) {
    // Save the distance
    ClimberExtend::distance = distance;

    // Update the PID controller
    pid.SetReference(distance.to<double>(),
                rev::CANSparkMaxLowLevel::ControlType::kPosition);
}

units::meter_t ClimberExtend::GetActualExtension() {
    return (units::meter_t) winch.GetEncoder().GetPosition();
}
