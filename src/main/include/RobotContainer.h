#pragma once
#include "Constants.h"
#include "frc/Joystick.h"
#include "frc2/command/Command.h"
#include "subsystems/climber/Climber.h"
#include "subsystems/intake/Intake.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Vision.h"


class RobotContainer {
private:
    Drivetrain drivetrain;
    Climber climber;
    Intake intake;
    Vision vision;
    frc::Joystick driveStick{CONSTANTS::CONTROLLERS::USB::DRIVE_STICK},
            controlStick{CONSTANTS::CONTROLLERS::USB::CONTROL_STICK};

    void ConfigureButtonBindings();

public:
    RobotContainer();

    frc2::Command* GetAutonomousCommand();
};
