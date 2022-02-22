// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer()
{
	drive = new DrivetrainDrive();
	Stick = new Joystick(0);
	Stick2 = new Joystick(1);
	intakem = new Intake();
	climberm = new Climber();

	drive->SetDefaultCommand(frc2::RunCommand(
		[this]
		{
			drive->ArcadeDrive(Stick->GetRawAxis(JOYSTICK::AXIS::Y), Stick->GetRawAxis(JOYSTICK::AXIS::X));
		},
		{drive}));

{	up = new frc2::JoystickButton(Stick, JOYSTICK::BUTTON::BUTTON_4);
	down = new frc2::JoystickButton(Stick, JOYSTICK::BUTTON::BUTTON_5);
	in = new frc2::JoystickButton(Stick, JOYSTICK::BUTTON::BUTTON_6);
	out = new frc2::JoystickButton(Stick, JOYSTICK::BUTTON::BUTTON_7);
	off = new frc2::JoystickButton(Stick, JOYSTICK::BUTTON::BUTTON_8);

	down->WhenPressed(new frc2::RunCommand(
		[this]
		{
			intakem->UpDown(updownp);
		},
		{intakem}));
	up->WhenPressed(new frc2::RunCommand(
		[this]
		{
			intakem->UpDown(-updownp);
		},
		{intakem}));
	in->WhenPressed(new frc2::RunCommand(
		[this]
		{
			intakem->Power(inp);
		
		},
		{intakem}));
	out->WhenPressed(new frc2::RunCommand(
		[this]
		{
			intakem->Power(-outp);
		
		},
		{intakem}));
	off->WhenPressed(new frc2::RunCommand(
		[this]
		{
			intakem->Power(0);
		
		},
		{intakem}));



}
{	///*
	one = new frc2::JoystickButton(Stick, JOYSTICK::BUTTON::BUTTON_9);
	two = new frc2::JoystickButton(Stick, JOYSTICK::BUTTON::BUTTON_10);

one->WhenPressed(new frc2::RunCommand(
		[this]
		{ counter++;
			climberm->clim(counter);
		},
		{climberm}));



//*/
}
}
RobotContainer::~RobotContainer()
{
	delete drive;
	delete Stick;
	delete intakem;
	delete Stick2;
	delete climberm;
}
void RobotContainer::Periodic() {}
