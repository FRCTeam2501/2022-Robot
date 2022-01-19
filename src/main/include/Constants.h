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
        constexpr int CLIMBER_ROTATION_ID = 5;
        constexpr int CLIMBER_LEFT_WINCH_ID = 6;
        constexpr int CLIMBER_RIGHT_WINCH_ID = 7;
    }
    namespace CONTROLLERS::USB {
        constexpr int DRIVE_STICK = 0;
    }
    namespace CONTROLLERS::BUTTONS {
        constexpr int DRIVE_STICK_REVERSE_DRIVETRAIN = 1;
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
    namespace CLIMBER {
        constexpr units::current::ampere_t
                HARD_CURRENT_LIMIT = 100_A,
                SOFT_CURRENT_LIMIT = 60_A;
        constexpr auto
                TURN_TO_DEGREES = 360.0 / units::angle::turn_t(1);
        namespace CONTROL {
            constexpr double P = 0.1, I = 0.0, D = 0.0;
            constexpr double F = 0.0, I_ZONE = 0.0;
            constexpr double MAX = 1.0, MIN = -1.0;
        }
    }
}