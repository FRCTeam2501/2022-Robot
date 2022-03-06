#pragma once
#include "frc/Joystick.h"
#include "frc2/command/Command.h"
#include "subsystems/Climber.h"
#include "subsystems/Drivetrain.h"


class RobotContainer {
private:
    Drivetrain *drivetrain;
    Climber *climber;
    frc::Joystick *driveStick, *controlStick;

    void ConfigureButtonBindings();

public:
    RobotContainer();

    frc2::Command* GetAutonomousCommand();
};
