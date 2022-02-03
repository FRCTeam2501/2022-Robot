#include "subsystems/climber/ClimberExtend.h"
#include "Constants.h"


ClimberExtend::ClimberExtend() {
    // Create motor controllers
    winch = new rev::CANSparkMax(
        CONSTANTS::MOTORS::CAN::CLIMBER_EXTEND_ID,
        rev::CANSparkMax::MotorType::kBrushless
    );

    // Setup current limits, idle modes, and encoder factors
    winch->SetSmartCurrentLimit(
            CONSTANTS::CLIMBER::EXTEND::HARD_CURRENT_LIMIT.to<int>());
    winch->SetSecondaryCurrentLimit(
            CONSTANTS::CLIMBER::EXTEND::SOFT_CURRENT_LIMIT.to<double>());
    winch->GetEncoder().SetPositionConversionFactor(
            CONSTANTS::CLIMBER::EXTEND::TURN_TO_METER.to<double>());

    // Set up PID controller
    winch->GetPIDController().SetP(
            CONSTANTS::CLIMBER::EXTEND::PID::P);
    winch->GetPIDController().SetI(
            CONSTANTS::CLIMBER::EXTEND::PID::I);
    winch->GetPIDController().SetD(
            CONSTANTS::CLIMBER::EXTEND::PID::D);
    winch->GetPIDController().SetFF(
            CONSTANTS::CLIMBER::EXTEND::PID::FF);
    winch->GetPIDController().SetIZone(
            CONSTANTS::CLIMBER::EXTEND::PID::I_ZONE);
    winch->GetPIDController().SetOutputRange(
            CONSTANTS::CLIMBER::EXTEND::PID::MIN,
            CONSTANTS::CLIMBER::EXTEND::PID::MAX
    );
}

ClimberExtend::~ClimberExtend() {
    delete winch;
}

units::meter_t ClimberExtend::GetExtension() {
    return distance;
}

void ClimberExtend::SetExtension(units::meter_t distance) {
    // Save the distance
    ClimberExtend::distance = distance;

    // Update the PID controller
    winch->GetPIDController().SetReference(
        distance.to<double>(), rev::CANSparkMaxLowLevel::ControlType::kPosition);
}
