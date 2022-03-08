#include "RobotContainer.h"
#include "frc2/command/button/JoystickButton.h"
#include "frc2/command/ConditionalCommand.h"
#include "frc2/command/InstantCommand.h"
#include "frc2/command/ParallelRaceGroup.h"
#include "frc2/command/RunCommand.h"
#include "frc2/command/StartEndCommand.h"
#include "frc2/command/SequentialCommandGroup.h"
#include "frc2/command/WaitCommand.h"


RobotContainer::RobotContainer() {
    drivetrain.SetDefaultCommand(frc2::RunCommand([this] {
        if(driveStick.GetThrottle() >= 0.0)
            drivetrain.ArcadeDrive(
                -1.0 * driveStick.GetY(),
                0.6 * driveStick.GetX()
            );
        else
            drivetrain.ArcadeDrive(
                -0.6 * driveStick.GetY(),
                0.45 * driveStick.GetX()
            );
    }, { &drivetrain } ));

    climber.SetDefaultCommand(frc2::RunCommand([this] {
        if(abs(controlStick.GetY()) > 0.1)
            climber.Set(
                climber.GetExtension(),
                (1.0_deg * controlStick.GetY())
                    + climber.GetAngle()
            );
    }, { &climber } ));

    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
    // Reverse drivetrain direction
    frc2::JoystickButton(&driveStick,
            CONSTANTS::CONTROLLERS::BUTTONS::DRIVE::REVERSE_DRIVETRAIN
    ).WhenPressed([this] {
        drivetrain.SetInverted(!drivetrain.IsInverted());
    }, { &drivetrain } );


    // Run the intake inwards
    frc2::JoystickButton(&driveStick,
            CONSTANTS::CONTROLLERS::BUTTONS::DRIVE::RUN_INTAKE
    ).WhileHeld(frc2::StartEndCommand{
        [this] {
            intake.SetMotor(0.8);
        },
        [this] {
            intake.SetMotor(0.0);
        },
        { &intake }
    });

    // Reverse the intake
    frc2::JoystickButton(&driveStick,
            CONSTANTS::CONTROLLERS::BUTTONS::DRIVE::REVERSE_INTAKE
    ).WhileHeld(frc2::StartEndCommand{
        [this] {
            intake.SetMotor(-0.8);
        },
        [this] {
            intake.SetMotor(0.0);
        },
        { &intake }
    });

    // Toggle the intake
    frc2::JoystickButton(&driveStick,
            CONSTANTS::CONTROLLERS::BUTTONS::DRIVE::TOGGLE_INTAKE
    ).WhenPressed([this] {
        if(intake.GetAngle() > -50.0_deg)
            intake.SetAngle(-90.0_deg);
        else
            intake.SetAngle(0.0_deg);
    }, { &intake } );

    // Set the intake to the middle
    frc2::JoystickButton(&driveStick,
            CONSTANTS::CONTROLLERS::BUTTONS::DRIVE::INTAKE_MIDDLE
    ).WhenPressed([this] {
        intake.SetAngle(-30.0_deg);
    }, { &intake } );

    // Zero the intake encoder at min
    frc2::JoystickButton(&driveStick,
            CONSTANTS::CONTROLLERS::BUTTONS::DRIVE::ZERO_INTAKE_ENCODER_MIN
    ).WhenPressed([this] {
        intake.ZeroEncoder(-1 * 90.0_deg);
    }, { &intake } );

    // Zero the intake encoder at max
    frc2::JoystickButton(&driveStick,
            CONSTANTS::CONTROLLERS::BUTTONS::DRIVE::ZERO_INTAKE_ENCODER_MAX
    ).WhenPressed([this] {
        intake.ZeroEncoder(0.0_deg);
    }, { &intake } );


    // Run the climber upwards
    frc2::JoystickButton(&controlStick,
            CONSTANTS::CONTROLLERS::BUTTONS::CONTROL::CLIMBER_EXTEND
    ).WhileHeld(frc2::RunCommand{
        [this] {
            if(controlStick.GetThrottle() >= 0.0)
                climber.Set(
                    climber.GetExtension() + 2.5_in,
                    climber.GetAngle()
                );
            else
                climber.Set(
                    climber.GetExtension() + 0.5_in,
                    climber.GetAngle()
                );
        },
        { &climber }
    } );

    // Run the climber downwards
    frc2::JoystickButton(&controlStick,
            CONSTANTS::CONTROLLERS::BUTTONS::CONTROL::CLIMBER_RETRACT
    ).WhileHeld(frc2::RunCommand{
        [this] {
            if(controlStick.GetThrottle() >= 0.0)
                climber.Set(
                    climber.GetExtension() - 2.5_in,
                    climber.GetAngle()
                );
            else
                climber.Set(
                    climber.GetExtension() - 0.5_in,
                    climber.GetAngle()
                );
        },
        { &climber }
    } );

    // Bring the climber in over the course of five seconds
    frc2::JoystickButton(&controlStick,
            CONSTANTS::CONTROLLERS::BUTTONS::CONTROL::CLIMBER_IN
    ).WhenPressed(frc2::SequentialCommandGroup{
        frc2::RunCommand{[this] {
            climber.Set(20_in, 70_deg); // Retract until off the bar
        }, { &climber } }.WithTimeout(
            2_s
        ),
        frc2::RunCommand{[this] {
            climber.Set(9_in, 0_deg); // Quickly retract and go vertical
        }, { &climber } }.WithTimeout(
            2_s
        ),
        frc2::RunCommand{[this] {
            climber.Set(0_in, 0_deg); // Retract fully once vertical
        }, { &climber } }.WithTimeout(
            1_s
        )
    });

    // Set the climber to the out position
    frc2::JoystickButton(&controlStick,
            CONSTANTS::CONTROLLERS::BUTTONS::CONTROL::CLIMBER_OUT
    ).WhenPressed([this] {
        climber.Set(27_in, 80_deg);
    }, { &climber } );

    // Run the climber down and back up to dislodge the wrench
    frc2::JoystickButton(&controlStick,
            CONSTANTS::CONTROLLERS::BUTTONS::CONTROL::CLIMBER_DISLODGE_WRENCH
    ).WhenPressed(frc2::SequentialCommandGroup{
        frc2::RunCommand([this] {
            climber.Set(-0.5_in, 0_deg);
        }, { &climber } ).WithTimeout(
            0.5_s
        ),
        frc2::InstantCommand([this] {
            climber.Set(0_in, 0_deg);
        }, { &climber } )
    });


    // Switch camera feed
    frc2::JoystickButton(&driveStick,
            CONSTANTS::CONTROLLERS::BUTTONS::DRIVE::VISION_SWITCH_FEED
    ).WhenPressed([this] {
        vision.SwitchFeed();
    }, { &vision } );
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return new frc2::SequentialCommandGroup(
        // Zero encoders
        frc2::InstantCommand{[this] {
            intake.ZeroEncoder(0.0_deg);
            climber.ZeroEncoders();
        }, { &intake, &climber } },
        // Run the intake for 2 seconds
        frc2::RunCommand([this] {
            intake.SetMotor(-0.8);
        }, { &intake } ).WithTimeout(
            2_s
        ),
        // Stop the intake
        frc2::InstantCommand([this] {
            intake.SetMotor(0.0);
        }, { &intake } ),
        // Drive backwards for 1.2 seconds
        frc2::RunCommand{[this] {
            drivetrain.ArcadeDrive(-0.6, 0.0);
        }, { &drivetrain } }.WithTimeout(
            1.2_s
        ),
        // Stop the drivetrain
        frc2::InstantCommand{[this] {
            drivetrain.ArcadeDrive(0.0, 0.0);
        }, { &drivetrain } }
        // Do nothing for the remaining 8.8 seconds
    );
}
