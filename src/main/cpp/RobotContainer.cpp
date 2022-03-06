#include "RobotContainer.h"
#include "frc2/command/button/JoystickButton.h"
#include "frc2/command/ConditionalCommand.h"
#include "frc2/command/InstantCommand.h"
#include "frc2/command/RunCommand.h"
#include "frc2/command/ParallelRaceGroup.h"
#include "frc2/command/WaitCommand.h"
#include "frc2/command/SequentialCommandGroup.h"


RobotContainer::RobotContainer() {
    drivetrain.SetDefaultCommand(frc2::RunCommand([this] {
        if(driveStick.GetThrottle() <= 0.0)
            drivetrain.ArcadeDrive(
                -0.6 * driveStick.GetY(),
                0.45 * driveStick.GetX()
            );
        else
            drivetrain.ArcadeDrive(
                -1.0 * driveStick.GetY(),
                0.6 * driveStick.GetX()
            );
    }, { &drivetrain } ));

    climber.SetDefaultCommand(frc2::RunCommand([this] {
        if(abs(controlStick.GetY()) > 0.1)
            climber.Set(
                (1.0_in * controlStick.GetY())
                    + climber.GetExtension(),
                climber.GetAngle()
            );
    }, { &climber } ));

    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
    frc2::JoystickButton *temp;

    frc2::JoystickButton(&driveStick,
            CONSTANTS::CONTROLLERS::BUTTONS::DRIVE::REVERSE_DRIVETRAIN
    ).WhenPressed([this] {
        drivetrain.SetInverted(!drivetrain.IsInverted());
    }, { &drivetrain } );

    frc2::JoystickButton(&controlStick,
            CONSTANTS::CONTROLLERS::BUTTONS::CONTROL::CLIMBER_DISLODGE_WRENCH
    ).WhenPressed(frc2::SequentialCommandGroup{
        frc2::RunCommand([this] {
            climber.Set(-0.5_in, 0_deg);
        }, { &climber }).WithTimeout(
            0.5_s
        ),
        frc2::InstantCommand([this] {
            climber.Set(0_in, 0_deg);
        }, { &climber })
    });
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return new frc2::SequentialCommandGroup(
        frc2::RunCommand{[this] {
            drivetrain.ArcadeDrive(0.5, 0.0);
        }, { &drivetrain } }.WithTimeout(
            1_s
        ),
        frc2::InstantCommand{[this] {
            drivetrain.ArcadeDrive(0.0, 0.0);
        }, { &drivetrain } }
    );
}
