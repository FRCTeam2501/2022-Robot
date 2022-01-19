#pragma once
#include "frc/motorcontrol/MotorControllerGroup.h"
#include "frc2/command/SubsystemBase.h"
#include "rev/CANSparkMax.h"


class Climber : public frc2::SubsystemBase {
  private:
    // Individual speed controllers
    rev::CANSparkMax *leftWinch, *rightWinch, *rotation;
    // Groups of speed controllers
    frc::MotorControllerGroup *winches;

  public:
    Climber();
    ~Climber();

    void SetWinchSpeed(double speed);
    void SetRotationAngle(double angle);
};