#pragma once
#include "rev/CANSparkMax.h"
#include "units/current.h"
#include "units/dimensionless.h"
#include "units/length.h"
#include "Constants.h"


namespace CONSTANTS::CLIMBER::EXTEND {
    constexpr units::current::ampere_t
            HARD_CURRENT_LIMIT = 100_A,
            SOFT_CURRENT_LIMIT = 60_A;
    constexpr units::scalar_t
            GEARBOX_RATIO = 100.0, // 100:1 planetary gearbox
            PULLEY_RATIO = 122.0 / 65.0; // 122:65 pulley reduction
    constexpr auto
            TURN_TO_METER = 1_m / (GEARBOX_RATIO * PULLEY_RATIO);
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

class ClimberExtend {
  private:
    // Individual speed controllers
    rev::CANSparkMax winch{CONSTANTS::MOTORS::CAN::CLIMBER_EXTEND_ID,
                rev::CANSparkMax::MotorType::kBrushless};
    // Speed controller PID controllers
    rev::SparkMaxPIDController pid = winch.GetPIDController();
    // State variables
    units::meter_t distance;

  public:
    ClimberExtend();

    units::meter_t GetExtension();
    void SetExtension(units::meter_t distance);

    units::meter_t GetActualExtension();
};
