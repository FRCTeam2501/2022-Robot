#pragma once
#include "units/angle.h"
#include "units/constants.h"
#include "units/current.h"
#include "units/dimensionless.h"
#include "units/length.h"


namespace CONSTANTS {
    namespace MOTORS::CAN {
        constexpr int
                LEFT_FRONT_ID = 1,
                RIGHT_FRONT_ID = 2,
                LEFT_REAR_ID = 3,
                RIGHT_REAR_ID = 4,
                CLIMBER_ROTATION_ID = 5,
                CLIMBER_LEFT_WINCH_ID = 6,
                CLIMBER_RIGHT_WINCH_ID = 7;
    }
    namespace CONTROLLERS::USB {
        constexpr int
                DRIVE_STICK = 0,
                CONTROL_STICK = 1;
    }
    namespace CONTROLLERS::BUTTONS {
        namespace DRIVE_STICK {
            constexpr int
                    REVERSE_DRIVETRAIN = 1;
        }
        namespace CONTROL_STICK {
            constexpr int
                    RUN_WINCHES = 1,
                    REVERSE_WINCHES = 2,
                    INCREMENT_CLIMBER_ANGLE = 7,
                    DECREMENT_CLIMBER_ANGLE = 9;
        }
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
                TURN_TO_METER = WHEEL_CIR / GEAR_RATIO;
    }
    namespace CLIMBER::WINCH {
        constexpr units::current::ampere_t
                HARD_CURRENT_LIMIT = 100_A,
                SOFT_CURRENT_LIMIT = 60_A;
        constexpr double
                FORWARD_ADJUSTMENT_SPEED = 0.5, // 50% to 100% speed adjustment
                REVERSE_MIN_SPEED = 0.25,
                REVERSE_MAX_SPEED = 0.75; // 25% to 75% speed adjustment
    }
    namespace CLIMBER::ROTATION {
        constexpr units::current::ampere_t
                HARD_CURRENT_LIMIT = 120_A,
                SOFT_CURRENT_LIMIT = 80_A;
        constexpr units::scalar_t
                GEAR_RATIO = 100 * 4;
        constexpr units::degree_t
                MIN_ANGLE = 0_deg,
                MAX_ANGLE = 60_deg,
                ADJUSTMENT_ANGLE = 5_deg;
        constexpr auto
                TURN_TO_DEGREES = GEAR_RATIO / 360_deg;
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
}