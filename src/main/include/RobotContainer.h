#pragma once
#include "frc/Joystick.h"
#include "subsystems/Drivetrain.h"


class RobotContainer {
  private:
    Drivetrain *drivetrain;
    frc::Joystick *driveStick, *controlStick;

    void ConfigureButtonBindings();

  public:
    RobotContainer();
};
