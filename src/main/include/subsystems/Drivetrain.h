#pragma once
#include "frc/drive/DifferentialDrive.h"
#include "frc/motorcontrol/MotorControllerGroup.h"
#include "frc2/command/SubsystemBase.h"
#include "rev/CANSparkMax.h"
#include "units/angle.h"
#include "units/constants.h"
#include "units/current.h"
#include "units/dimensionless.h"
#include "units/length.h"
#include "Constants.h"


namespace CONSTANTS::DRIVETRAIN {
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


class Drivetrain : public frc2::SubsystemBase {
private:
    // Individual speed controllers
    rev::CANSparkMax
        leftFront{CONSTANTS::MOTORS::CAN::LEFT_FRONT_ID,
                rev::CANSparkMax::MotorType::kBrushless},
        rightFront{CONSTANTS::MOTORS::CAN::RIGHT_FRONT_ID,
                rev::CANSparkMax::MotorType::kBrushless},
        leftRear{CONSTANTS::MOTORS::CAN::LEFT_REAR_ID,
                rev::CANSparkMax::MotorType::kBrushless},
        rightRear{CONSTANTS::MOTORS::CAN::RIGHT_REAR_ID,
                rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkMaxRelativeEncoder
            leftFrontEncoder = leftFront.GetEncoder(),
            rightFrontEncoder = rightFront.GetEncoder(),
            leftRearEncoder = leftRear.GetEncoder(),
            rightRearEncoder = rightRear.GetEncoder();
    // Groups of speed controllers
    frc::MotorControllerGroup
            left{leftFront, leftRear},
            right{rightFront, rightRear};
    // Differential drivetrain object
    frc::DifferentialDrive drive{left, right};

    // State variable to indicate if the drivetrain is inverted
    bool isInverted = false;

public:
    void ArcadeDrive(double xSpeed, double zRotation);

    bool IsInverted();
    void SetInverted(bool inverted);
};
