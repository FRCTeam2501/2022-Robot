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
    namespace CONTROLLERS::BUTTONS::DRIVE_STICK {
        constexpr int
                REVERSE_DRIVETRAIN = 1;
    }
}
