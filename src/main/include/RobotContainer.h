#pragma once
#include "frc/Joystick.h"
#include "subsystems/ClimberRotate.h"
#include "subsystems/ClimberWinches.h"
#include "subsystems/Drivetrain.h"


class RobotContainer {
  private:
    Drivetrain *drivetrain;
    ClimberRotate *climberRotate;
    ClimberWinches *climberWinches;
    frc::Joystick *driveStick, *controlStick;

    void ConfigureButtonBindings();

  public:
    RobotContainer();
};
