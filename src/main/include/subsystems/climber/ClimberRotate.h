#pragma once
#include "rev/CANSparkMax.h"
#include "units/angle.h"
#include "units/current.h"
#include "units/dimensionless.h"


namespace CONSTANTS::CLIMBER::ROTATION {
    constexpr units::current::ampere_t
            HARD_CURRENT_LIMIT = 120_A,
            SOFT_CURRENT_LIMIT = 80_A;
    constexpr units::scalar_t
            GEARBOX_RATIO = 100.0, // 100:1 planetary gearbox
            PULLEY_RATIO = 122.0 / 65.0; // 122:65 pulley reduction
    constexpr units::degree_t
            MIN_ANGLE = 0_deg,
            MAX_ANGLE = 60_deg,
            ADJUSTMENT_ANGLE = 5_deg;
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
    rev::CANSparkMax *rotation;
    // State variables
    units::degree_t angle;

  public:
    ClimberRotate();
    ~ClimberRotate();

    units::degree_t GetAngle();
    void SetAngle(units::degree_t angle);
};
