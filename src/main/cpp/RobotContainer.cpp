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



{	up = new frc2::JoystickButton(Stick, JOYSTICK::BUTTON::BUTTON_4);
	down = new frc2::JoystickButton(Stick, JOYSTICK::BUTTON::BUTTON_5);
	in = new frc2::JoystickButton(Stick, JOYSTICK::BUTTON::BUTTON_6);
	out = new frc2::JoystickButton(Stick, JOYSTICK::BUTTON::BUTTON_7);
	off = new frc2::JoystickButton(Stick, JOYSTICK::BUTTON::BUTTON_8);
//	off2 = new frc2::JoystickButton(Stick, JOYSTICK::BUTTON::BUTTON_9);
	down->WhenPressed(new frc2::RunCommand(
		[this]
		{
			intakem->UpDown(IntakeConstants::down);
		},
		{intakem}));
	up->WhenPressed(new frc2::RunCommand(
		[this]
		{
			intakem->UpDown(IntakeConstants::up);
		},
		{intakem}));
	in->WhenPressed(new frc2::RunCommand(
		[this]
		{
			intakem->Power(IntakeConstants::inp);
		
		},
		{intakem}));
	out->WhenPressed(new frc2::RunCommand(
		[this]
		{
			intakem->Power(-IntakeConstants::outp);
		
		},
		{intakem}));
	off->WhenPressed(new frc2::RunCommand(
		[this]
		{
			intakem->Power(0);
		
		},
		{intakem}));
	/*off2->WhenPressed(new frc2::RunCommand(
		[this]
		{
			intakem->UpDown(IntakeConstants::start);
		
		},
		{intakem}));
*/
}


	drive->SetDefaultCommand(frc2::RunCommand(
		[this]
		{
			drive->ArcadeDrive(Stick->GetRawAxis(JOYSTICK::AXIS::Y), Stick->GetRawAxis(JOYSTICK::AXIS::X));
		},
		{drive}));

		



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
