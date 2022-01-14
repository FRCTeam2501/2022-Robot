#include "RobotContainer.h"
#include "frc2/command/RunCommand.h"
#include "Constants.h"


RobotContainer::RobotContainer() :
        driveStick{CONSTANTS::CONTROLLERS::DRIVESTICK} {
    ConfigureButtonBindings();

    drivetrain.SetDefaultCommand(frc2::RunCommand(
            [this] {
                drivetrain.ArcadeDrive(
                    driveStick.GetY(),
                    driveStick.GetX()
                );
            },
            { &drivetrain }
    ));
}

void RobotContainer::ConfigureButtonBindings() {
    
}
