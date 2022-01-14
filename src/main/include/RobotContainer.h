#pragma once
#include "frc/Joystick.h"
#include "subsystems/Drivetrain.h"


class RobotContainer {
  private:
    Drivetrain *drivetrain;
    frc::Joystick *driveStick;

    void ConfigureButtonBindings();

  public:
    RobotContainer();
};
