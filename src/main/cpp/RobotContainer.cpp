#include "RobotContainer.h"
#include "frc2/command/RunCommand.h"
#include "frc2/command/button/JoystickButton.h"
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

    temp = new frc2::JoystickButton(
        controlStick,
        CONSTANTS::CONTROLLERS::BUTTONS::CONTROL_STICK::INCREMENT_CLIMBER_ANGLE
    );
    temp->WhenPressed(
        [this] {
            units::degree_t angle = climber->GetRotationAngle();
            angle += CONSTANTS::CLIMBER::ROTATION::ADJUSTMENT_ANGLE;
            if(angle > CONSTANTS::CLIMBER::ROTATION::MAX_ANGLE) {
                angle = CONSTANTS::CLIMBER::ROTATION::MAX_ANGLE;
            }
            climber->SetRotationAngle(angle);
        },
		{ climber }
    );

	temp = new frc2::JoystickButton(
		controlStick,
		CONSTANTS::CONTROLLERS::BUTTONS::CONTROL_STICK::DECREMENT_CLIMBER_ANGLE
	);
	temp->WhenPressed(
		[this] {
			units::degree_t angle = climber->GetRotationAngle();
            angle -= CONSTANTS::CLIMBER::ROTATION::ADJUSTMENT_ANGLE;
            if(angle < CONSTANTS::CLIMBER::ROTATION::MIN_ANGLE) {
                angle = CONSTANTS::CLIMBER::ROTATION::MIN_ANGLE;
            }
            climber->SetRotationAngle(angle);
		},
		{ climber }
	);
}
