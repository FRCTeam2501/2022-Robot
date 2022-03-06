#include "RobotContainer.h"
#include "frc2/command/button/JoystickButton.h"
#include "frc2/command/InstantCommand.h"
#include "frc2/command/RunCommand.h"
#include "frc2/command/SequentialCommandGroup.h"
#include "frc2/command/WaitCommand.h"
#include "frc2/command/ParallelCommandGroup.h"
#include "frc2/command/ParallelRaceGroup.h"
#include "frc2/command/StartEndCommand.h"
#include "Constants.h"


RobotContainer::RobotContainer() {
    driveStick = new frc::Joystick(
        CONSTANTS::CONTROLLERS::USB::DRIVE_STICK);
    controlStick = new frc::Joystick(
        CONSTANTS::CONTROLLERS::USB::CONTROL_STICK);

	drivetrain = new Drivetrain();
    climber = new Climber();

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

void RobotContainer::Init() {
    climber->ExtendPin();
}

void RobotContainer::ConfigureButtonBindings() {
    frc2::JoystickButton *temp;

    temp = new frc2::JoystickButton(
        driveStick,
        CONSTANTS::CONTROLLERS::BUTTONS::DRIVE_STICK::REVERSE_DRIVETRAIN
    );
    temp->WhenPressed(
        [this] {
            drivetrain->SetInverted(!drivetrain->IsInverted());
        },
		{ drivetrain }
    );
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return new frc2::SequentialCommandGroup(
        frc2::RunCommand{
            [this] {
                drivetrain->ArcadeDrive(0.5, 0.0);
            },
            { drivetrain }
        }.WithTimeout(
            1_s
        ),
        frc2::InstantCommand{
            [this] {
                drivetrain->ArcadeDrive(0.0, 0.0);
            },
            { drivetrain }
        }
    );
}
