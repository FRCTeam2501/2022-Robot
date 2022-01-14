#include "subsystems/Drivetrain.h"
#include "Constants.h"


Drivetrain::Drivetrain() :
        leftFront{
            CONSTANTS::MOTORS::CAN::LEFT_FRONT,
            rev::CANSparkMax::MotorType::kBrushless
        },
        rightFront{
            CONSTANTS::MOTORS::CAN::RIGHT_FRONT,
            rev::CANSparkMax::MotorType::kBrushless
        },
        leftRear{
            CONSTANTS::MOTORS::CAN::LEFT_REAR,
            rev::CANSparkMax::MotorType::kBrushless
        },
        rightRear{
            CONSTANTS::MOTORS::CAN::RIGHT_REAR,
            rev::CANSparkMax::MotorType::kBrushless
        } {
}

void Drivetrain::ArcadeDrive(double xSpeed, double zRotation) {
    drive.ArcadeDrive(xSpeed, zRotation);
}