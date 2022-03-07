#pragma once
#include "rev/CANSparkMax.h"
#include "units/angle.h"
#include "units/constants.h"
#include "units/current.h"
#include "units/dimensionless.h"
#include "units/length.h"
#include "Constants.h"

namespace CONSTANTS::CLIMBER::EXTEND {
    constexpr units::current::ampere_t
            HARD_CURRENT_LIMIT = 60_A,
            SOFT_CURRENT_LIMIT = 70_A;
    constexpr units::scalar_t
            GEARBOX_RATIO = 1 / 100.0; // 100:1 planetary gearbox
    constexpr units::meter_t
            PULLEY_DIAMETER = 1.7887_in,
            PULLEY_CIRCUMFERENCE = PULLEY_DIAMETER * units::constants::pi;
    namespace PID {
        constexpr double 
                P = 2.0,
                I = 0.0,
                D = 0.0,
                FF = 0.0,
                I_ZONE = 0.0,
                MAX = 1.0,
                MIN = -1.0;
    }
}

class ClimberExtend {
private:
    // Individual speed controller
    rev::CANSparkMax winch{CONSTANTS::MOTORS::CAN::CLIMBER_EXTEND_ID,
            rev::CANSparkMax::MotorType::kBrushless};
    // Speed controller objects
    rev::SparkMaxPIDController pid = winch.GetPIDController();
    rev::SparkMaxRelativeEncoder encoder = winch.GetEncoder();
    // State variables
    units::meter_t distance;

    units::turn_t GetTurns(units::meter_t distance);
    units::meter_t GetDistance(units::turn_t turns);

public:
    ClimberExtend();

    units::meter_t Get();
    void Set(units::meter_t distance);

    units::meter_t GetActual();

    void Zero();
};
