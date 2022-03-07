#pragma once


namespace CONSTANTS {
    namespace MOTORS::CAN {
        constexpr int
                LEFT_FRONT_ID = 1,
                RIGHT_FRONT_ID = 2,
                LEFT_REAR_ID = 3,
                RIGHT_REAR_ID = 4,
                CLIMBER_ROTATION_ID = 5,
                CLIMBER_EXTEND_ID = 6,
                INTAKE_ROTATION_ID = 13,
                INTAKE_MOTOR_ID = 16;
    }
    namespace CONTROLLERS::USB {
        constexpr int
                DRIVE_STICK = 0,
                CONTROL_STICK = 1;
    }
    namespace CONTROLLERS::BUTTONS::DRIVE {
        constexpr int
                RUN_INTAKE = 1,
                REVERSE_INTAKE = 2,
                TOGGLE_INTAKE = 3,
                INTAKE_MIDDLE = 4,
                ZERO_INTAKE_ENCODER_MIN = 7,
                ZERO_INTAKE_ENCODER_MAX = 8,
                REVERSE_DRIVETRAIN = 10,
                VISION_SWITCH_FEED = 12;
    }
    namespace CONTROLLERS::BUTTONS::CONTROL {
        constexpr int
                CLIMBER_EXTEND = 1,
                CLIMBER_RETRACT = 2,
                CLIMBER_IN = 5,
                CLIMBER_OUT = 6,
                CLIMBER_DISLODGE_WRENCH = 8;
    }
}
