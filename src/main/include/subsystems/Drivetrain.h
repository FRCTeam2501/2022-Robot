#pragma once
#include "frc/drive/DifferentialDrive.h"
#include "frc/motorcontrol/MotorControllerGroup.h"
#include "frc2/command/SubsystemBase.h"
#include "rev/CANSparkMax.h"


class Drivetrain : public frc2::SubsystemBase {
  private:
    // Individual speed controllers
    rev::CANSparkMax *leftFront, *rightFront, *leftRear, *rightRear;
    // Groups of speed controllers
    frc::MotorControllerGroup *left, *right;
    // Differential drivetrain object
    frc::DifferentialDrive *drive;

    // State variable to indicate if the drivetrain is inverted
    bool *isInverted;

  public:
    Drivetrain();
    ~Drivetrain();

    void ArcadeDrive(double xSpeed, double zRotation);

    bool IsInverted();
    void SetInverted(bool inverted);
};