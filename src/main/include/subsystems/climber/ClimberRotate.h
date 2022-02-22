#pragma once
#include "rev/CANSparkMax.h"
#include "units/angle.h"
#include "units/current.h"
#include "units/dimensionless.h"
#include "Constants.h"


namespace CONSTANTS::CLIMBER::ROTATION {
    constexpr units::current::ampere_t
            HARD_CURRENT_LIMIT = 120_A,
            SOFT_CURRENT_LIMIT = 80_A;
    constexpr units::scalar_t
            GEARBOX_RATIO = 100.0, // 100:1 planetary gearbox
            PULLEY_RATIO = 48.0 / 22.0; // 65:22 pulley reduction
    constexpr auto
            TURN_TO_DEGREE = 360_deg / (GEARBOX_RATIO * PULLEY_RATIO);
    namespace PID {
        constexpr double 
                P = 0.1,
                I = 0.0,
                D = 0.0,
                FF = 0.0,
                I_ZONE = 0.0,
                MAX = 1.0,
                MIN = -1.0;
    }
}

class ClimberRotate {
  private:
    // Individual speed controller
    rev::CANSparkMax rotation{CONSTANTS::MOTORS::CAN::CLIMBER_ROTATION_ID,
            rev::CANSparkMax::MotorType::kBrushless};
    // Speed controller objects
    rev::SparkMaxPIDController pid = rotation.GetPIDController();
    rev::SparkMaxRelativeEncoder encoder = rotation.GetEncoder();
    // State variables
    units::degree_t angle;

  public:
    ClimberRotate();

    units::degree_t Get();
    void Set(units::degree_t angle);

    units::degree_t GetActual();
};
