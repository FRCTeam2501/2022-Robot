#include "RobotContainer.h"
#include "frc2/command/RunCommand.h"
#include "frc2/command/button/JoystickButton.h"
#include "Constants.h"


RobotContainer::RobotContainer() {
    driveStick = new frc::Joystick(CONSTANTS::CONTROLLERS::USB::DRIVE_STICK);

    ConfigureButtonBindings();

    drivetrain->SetDefaultCommand(frc2::RunCommand(
        [this] {
            drivetrain->ArcadeDrive(
                driveStick->GetY(),
                driveStick->GetX()
            );
        },
        { drivetrain }
    ));
}

void RobotContainer::ConfigureButtonBindings() {
    frc2::JoystickButton *temp = new frc2::JoystickButton(
        driveStick,
        CONSTANTS::CONTROLLERS::BUTTONS::DRIVE_STICK_REVERSE_DRIVETRAIN
    );
    temp->WhenPressed(
        [this] {
            drivetrain->SetInverted(!drivetrain->IsInverted());
        }
    );
}
