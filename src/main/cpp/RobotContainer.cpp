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
	climberRotate = new ClimberRotate();
    climberExtend = new ClimberExtend();

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
        CONSTANTS::CONTROLLERS::BUTTONS::CONTROL_STICK::RUN_WINCHES
    );
    temp->WhileHeld(
        [this] {
            double speed = controlStick->GetThrottle();
            // Scale the speed to be from 100% down to FORWARD_ADJUSTMENT_SPEED
            speed *= CONSTANTS::CLIMBER::WINCH::FORWARD_ADJUSTMENT_SPEED;
            speed += 1.0;
            speed -= CONSTANTS::CLIMBER::WINCH::FORWARD_ADJUSTMENT_SPEED;
            climberExtend->SetSpeed(speed);
        },
        { climberExtend }
    );

    temp = new frc2::JoystickButton(
        controlStick,
        CONSTANTS::CONTROLLERS::BUTTONS::CONTROL_STICK::REVERSE_WINCHES
    );
    temp->WhileHeld(
        [this] {
            double speed = controlStick->GetThrottle();
            // Scale the speed to be from REVERSE_MAX_SPEED down to REVERSE_MIN_SPEED
            speed += 1.0;
            speed /= -2.0;
            speed *= CONSTANTS::CLIMBER::WINCH::REVERSE_MAX_SPEED - CONSTANTS::CLIMBER::WINCH::REVERSE_MIN_SPEED;
            speed -= CONSTANTS::CLIMBER::WINCH::REVERSE_MIN_SPEED;
            climberExtend->SetSpeed(speed);
        },
        { climberExtend }
    );

    temp = new frc2::JoystickButton(
        controlStick,
        CONSTANTS::CONTROLLERS::BUTTONS::CONTROL_STICK::INCREMENT_CLIMBER_ANGLE
    );
    temp->WhenPressed(
        [this] {
            units::degree_t angle = climberRotate->GetAngle();
            angle += CONSTANTS::CLIMBER::ROTATION::ADJUSTMENT_ANGLE;
            if(angle > CONSTANTS::CLIMBER::ROTATION::MAX_ANGLE) {
                angle = CONSTANTS::CLIMBER::ROTATION::MAX_ANGLE;
            }
            climberRotate->SetAngle(angle);
        },
		{ climberRotate }
    );

	temp = new frc2::JoystickButton(
		controlStick,
		CONSTANTS::CONTROLLERS::BUTTONS::CONTROL_STICK::DECREMENT_CLIMBER_ANGLE
	);
	temp->WhenPressed(
		[this] {
			units::degree_t angle = climberRotate->GetAngle();
            angle -= CONSTANTS::CLIMBER::ROTATION::ADJUSTMENT_ANGLE;
            if(angle < CONSTANTS::CLIMBER::ROTATION::MIN_ANGLE) {
                angle = CONSTANTS::CLIMBER::ROTATION::MIN_ANGLE;
            }
            climberRotate->SetAngle(angle);
		},
		{ climberRotate }
	);
}
