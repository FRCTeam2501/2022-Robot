#pragma once
#include "frc/motorcontrol/MotorControllerGroup.h"
#include "frc2/command/SubsystemBase.h"
#include "rev/CANSparkMax.h"
#include "units/angle.h"


class ClimberWinches : public frc2::SubsystemBase {
  private:
    // Individual speed controllers
    rev::CANSparkMax *leftWinch, *rightWinch;
    // Groups of speed controllers
    frc::MotorControllerGroup *winches;

  public:
    ClimberWinches();
    ~ClimberWinches();

    void SetSpeed(double speed);
};