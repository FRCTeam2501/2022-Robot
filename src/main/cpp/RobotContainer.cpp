// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer()
{
	intake = new Intake();
	drive = new DrivetrainDrive();
	driveStick = new Joystick(0);//usb port of 0
	camera = new Camera();
	controlStick = new Joystick(1);//usb port of 1
	climber = new Climber();
	drive->SetDefaultCommand(frc2::RunCommand(
		[this]
		{
			//	remove the constants from the coppy pasted code
			drive->ArcadeDrive(-1.0 * driveStick->GetRawAxis(
										  JOYSTICK::AXIS::Y),
							   0.6 * driveStick->GetRawAxis(JOYSTICK::AXIS::X));
		},
		{drive}));

	climber->SetDefaultCommand(frc2::RunCommand(
		[this]
		{
			if (controlStick->GetRawAxis(JOYSTICK::AXIS::Y) > 0.1)
			{
				climber->ClimbControl((controlStick->GetRawAxis(JOYSTICK::AXIS::Y) / 50) + climber->GetAngle(), (climber->GetLength()));
				// 50 means that it will adjust the angle to one degree per seccond at full speed on the joystick
			}
		},
		{climber}));

	armExtend = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_9);
	armExtend->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			climber->ClimbControl(80, 28); // 80 degrees, 28 inches
		},
		{climber}));

	swingAndClampBar = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_10);
	swingAndClampBar->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			climber->ClimbControl(1, 1); // 1 degree, 1 inch
		},
		{climber}));

	winchUp = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_7);
	winchUp->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			climber->ClimbControl((climber->GetAngle()), ((climber->GetLength()) + 0.5));
		},
		{climber}));
	winchDown = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_11);
	winchDown->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			climber->ClimbControl((climber->GetAngle()), ((climber->GetLength()) - 0.5));
		},
		{climber}));

	feedSwitch = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_12);
	feedSwitch->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			camera->SwitchFeed();
		},
		{camera}));

	rollerControl = new frc2::JoystickButton(driveStick, JOYSTICK::BUTTON::TRIGGER);
	rollerControl->WhileHeld(new frc2::StartEndCommand(
		[this]
		{
			if((driveStick->GetRawButton(2)) == true){
				intake->RollerControl(-0.3);
			}else{
				intake->RollerControl(0.3);	
			}

		},
		[this]
		{
			intake->RollerControl(0);
		},
		{intake}));
		
	
	liftControl = new frc2::JoystickButton(driveStick, JOYSTICK::BUTTON::BUTTON_10);
	liftControl->WhenPressed(new frc2::InstantCommand(
		[this]
		{
		
			switch (liftPosition)
			{
			case 1:
			intake->LiftControl(2);//2 degrees
			liftPosition = 2;
				break;
			case 2:
			intake->LiftControl(30);//30 degrees
			liftPosition = 1;
				break;
			default:
				break;
			}

		},
		{intake}));
}

RobotContainer::~RobotContainer()
{
	delete drive;
	delete driveStick;
	delete controlStick;
	delete climber;
	delete camera;
}

frc2::Command* GetAutoCommand(){
/*

	autoCommand = new frc2::SequentialCommandGroup(
				frc2::InstantCommand(
					[this] {
						intake->LiftControl(30);
					},
					{ intake }
				),
				frc2::WaitCommand(CONSTANTS::AUTO::SIMPLE_SHOOT::SPIN_UP_TIME),
				frc2::InstantCommand(
					[this] {
						hopper->Set(CONSTANTS::HOPPER::ON);
						hopper->TogglePin();
					},
					{ hopper }
				),
				frc2::WaitCommand(CONSTANTS::AUTO::SIMPLE_SHOOT::SHOOT_TIME),
				frc2::InstantCommand(
					[this] {
						shooter->Set(CONSTANTS::SHOOTER::OFF_SPEED);
						hopper->Set(CONSTANTS::HOPPER::OFF);
						hopper->TogglePin();
					},
					{ shooter, hopper }
				),
				frc2::ParallelRaceGroup(
					frc2::StartEndCommand(
						[this] {
							drive->ArcadeDrive(CONSTANTS::AUTO::SIMPLE_DRIVE::SPEED, 0.0);
						},
						[this] {
							drive->Stop();
						},
						{ drive }
					),
					frc2::WaitCommand(CONSTANTS::AUTO::SIMPLE_DRIVE::TIME)
				)
			);
			*/
			
}

void RobotContainer::Periodic()
{
}
