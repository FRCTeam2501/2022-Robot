// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer()
{
	drive = new DrivetrainDrive();
	driveStick = new Joystick(0);
	
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
		{drive}
	));


	hardAngle = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::TRIGGER);
	hardAngle->WhenPressed(new frc2::InstantCommand(
		[this]
		{
				if (testing < 40){
					testing = 60;			
					}else{
						testing = 30;
					}
			

			
			// frc::SmartDashboard::PutNumber("hi", 1);
			climber->HardAngle(testing);
		},
		{climber}));

	climber->SetDefaultCommand(frc2::RunCommand(
		[this]
		{
			if(abs(controlStick->GetRawAxis(JOYSTICK::AXIS::Y))<0.1){

			}else{
			//	remove the constants from the coppy pasted code
			climber->HardLength(
					(-0.05 * controlStick->GetRawAxis(JOYSTICK::AXIS::Y))
			);
			}
		},
		{ climber }
	));
}

RobotContainer::~RobotContainer()
{
	delete drive;
	delete driveStick;
	delete controlStick;
	delete climber;
}

void RobotContainer::Periodic()
{
}
