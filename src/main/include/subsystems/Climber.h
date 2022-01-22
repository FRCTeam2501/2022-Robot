#pragma once
#include "frc/motorcontrol/MotorControllerGroup.h"
#include "frc2/command/SubsystemBase.h"
#include "rev/CANSparkMax.h"
#include "units/angle.h"


class Climber : public frc2::SubsystemBase {
  private:
    // Individual speed controllers
    rev::CANSparkMax *leftWinch, *rightWinch, *rotation;
    // Groups of speed controllers
    frc::MotorControllerGroup *winches;
    // State variables
    units::degree_t angle;

  public:
    Climber();
    ~Climber();

    void SetWinchSpeed(double speed);

    units::degree_t GetRotationAngle();
    void SetRotationAngle(units::degree_t angle);
};