#pragma once
#include "units/angle.h"
#include "units/constants.h"
#include "units/current.h"
#include "units/dimensionless.h"
#include "units/length.h"


namespace CONSTANTS {
    namespace MOTORS::CAN {
        constexpr int LEFT_FRONT_ID = 1;
        constexpr int RIGHT_FRONT_ID = 2;
        constexpr int LEFT_REAR_ID = 3;
        constexpr int RIGHT_REAR_ID = 4;
    }
    namespace CONTROLLERS::USB {
        constexpr int DRIVESTICK = 0;
    }
    namespace DRIVETRAIN {
        constexpr units::dimensionless::scalar_t
                GEAR_RATIO = 10.71,
                FORWARD_SPEED = -1.0,
                ROTATION_SPEED = 0.6;
        constexpr units::current::ampere_t
                HARD_CURRENT_LIMIT = 100_A,
                SOFT_CURRENT_LIMIT = 60_A;
        constexpr units::length::meter_t
                WHEEL_DIAMETER = 6_in,
                TRACK_WIDTH = 22_in,
                WHEEL_CIR = (WHEEL_DIAMETER * units::constants::pi);
        constexpr auto
					TURN_TO_METER = WHEEL_CIR * GEAR_RATIO
                                    / units::angle::turn_t(1);
    }
}