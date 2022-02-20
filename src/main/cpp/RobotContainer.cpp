// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer()
{
	drive = new DrivetrainDrive();
	driveStick = new Joystick(0);
	camera = new Camera();
	controlStick = new Joystick(1);
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
			if(controlStick->GetRawAxis(JOYSTICK::AXIS::Y)>0.1){
			climber->ClimbControl((controlStick->GetRawAxis(JOYSTICK::AXIS::Y) / 50) + climber->GetAngle(), (climber->GetLength()));
			// 50 means that it will adjust the angle to one degree per seccond at full speed on the joystick
			}
		},
	
		{climber}));
	
	armExtend = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_9);
	armExtend->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			climber->ArmHorizontal();
		},
		{climber}));

	swingAndClampBar = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_10);
	swingAndClampBar->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			climber->SwingAndClamp();
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

	feedSwitch = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_8);
	feedSwitch->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			camera->SwitchFeed();
		},
		{camera}));
}

RobotContainer::~RobotContainer()
{
	delete drive;
	delete driveStick;
	delete controlStick;
	delete climber;
	delete camera;
}

void RobotContainer::Periodic()
{
}
