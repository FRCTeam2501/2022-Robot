// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"


RobotContainer::RobotContainer()  {
  drive = new DrivetrainDrive();
  driveStick = new Joystick(0);
  controlStick = new Joystick(1);
  climber = new Climber();
 drive->SetDefaultCommand(frc2::RunCommand(
		[this] {
			//	remove the constants from the coppy pasted code
			drive->ArcadeDrive(-1.0 * driveStick->GetRawAxis(
        JOYSTICK::AXIS::Y), 0.6 * driveStick->GetRawAxis(JOYSTICK::AXIS::X));
		},
		{ drive }
	));
    
	climber->SetDefaultCommand(frc2::RunCommand(
		[this] {
			climber->ControlPivot(controlStick->GetRawAxis(
        JOYSTICK::AXIS::Y) * 0.3);
		},
		{ climber }
	));

	winchUp = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_7);
	winchUp->WhenPressed(new frc2::InstantCommand(
		[this] {
			climber->WinchesUp(0.48);
			
		},
		{ climber }
	));
	winchDown = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_11);
	winchDown->WhenPressed(new frc2::InstantCommand(
		[this] {
			climber->WinchesDown(-0.48);
		},
		{ climber }
	));
	winchOff = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_9);
	winchOff->WhenPressed(new frc2::InstantCommand(
		[this] {
			climber->WinchesOff(0.0);
		},
		{ climber }
	)); 


}

RobotContainer::~RobotContainer()  {
  delete drive;
  delete driveStick;
  delete controlStick;
  delete climber;
}



void RobotContainer::Periodic(){

}
