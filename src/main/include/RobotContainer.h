#pragma once
#include "frc/Joystick.h"
#include "subsystems/ClimberRotate.h"
#include "subsystems/ClimberExtend.h"
#include "subsystems/Drivetrain.h"


class RobotContainer {
  private:
    Drivetrain *drivetrain;
    ClimberRotate *climberRotate;
    ClimberExtend *climberExtend;
    frc::Joystick *driveStick, *controlStick;

    void ConfigureButtonBindings();

  public:
    RobotContainer();
};
