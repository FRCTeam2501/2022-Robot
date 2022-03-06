#pragma once
#include "frc2/command/SubsystemBase.h"
#include "subsystems/intake/IntakeRotate.h"


namespace CONSTANTS::INTAKE::MOTOR {
    constexpr units::current::ampere_t
            HARD_CURRENT_LIMIT = 120_A,
            SOFT_CURRENT_LIMIT = 80_A;
}

class Intake : public frc2::SubsystemBase {
private:
    rev::CANSparkMax motor{CONSTANTS::MOTORS::CAN::INTAKE_MOTOR_ID,
            rev::CANSparkMax::MotorType::kBrushed};
    IntakeRotate rotation;

public:
    Intake();

    units::degree_t GetAngle();
    void SetAngle(units::degree_t angle);

    double GetMotor();
    void SetMotor(double voltage);
};
