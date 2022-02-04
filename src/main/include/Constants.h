#pragma once


namespace CONSTANTS {
    namespace MOTORS::CAN {
        constexpr int
                LEFT_FRONT_ID = 1,
                RIGHT_FRONT_ID = 2,
                LEFT_REAR_ID = 3,
                RIGHT_REAR_ID = 4,
                CLIMBER_ROTATION_ID = 5,
                CLIMBER_EXTEND_ID = 6;
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
}
