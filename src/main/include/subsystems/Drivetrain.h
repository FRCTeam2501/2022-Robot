#pragma once
#include "frc/drive/DifferentialDrive.h"
#include "frc/motorcontrol/MotorControllerGroup.h"
#include "frc2/command/SubsystemBase.h"
#include "rev/CANSparkMax.h"


class Drivetrain : public frc2::SubsystemBase {
  private:
    // Individual speed controllers
    rev::CANSparkMax leftFront, rightFront, leftRear, rightRear;
    // Groups of speed controllers
    frc::MotorControllerGroup left{leftFront, leftRear};
    frc::MotorControllerGroup right{rightFront, rightRear};
    // Differential drivetrain object
    frc::DifferentialDrive drive{left, right};

  public:
	Drivetrain();

    void ArcadeDrive(double xSpeed, double zRotation);
};