// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "iostream"

using namespace std;

RobotContainer::RobotContainer()
{
	intake = new Intake();
	drive = new DrivetrainDrive();
	driveStick = new Joystick(0); // usb port of 0
	camera = new Camera();
	controlStick = new Joystick(1); // usb port of 1
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
			if (abs(controlStick->GetRawAxis(JOYSTICK::AXIS::Y)) > 0.1)
			{
			//	cout<<"Y-Axis plus angle: "<<((controlStick->GetRawAxis(JOYSTICK::AXIS::Y) / 50) + climber->GetAngle())<<endl;
				cout<<"Get Angle: "<<(climber->GetAngle())<<endl;
				cout<<"Get Length: "<<(climber->GetLength())<<endl;
				angleAdd = ((controlStick->GetRawAxis(JOYSTICK::AXIS::Y) / 5) + climber->GetAngle());
				lengthAdd = (climber->GetLength());
				climber->ClimbControl(((controlStick->GetRawAxis(JOYSTICK::AXIS::Y) / 50) + climber->GetAngle()), (climber->GetLength()));
				//climber->ClimbControl(angleAdd, lengthAdd);
				// 50 means that it will adjust the angle to one degree per seccond at full speed on the joystick
			}
		},
		{climber}));
	
	zeroEncoders = new frc2::JoystickButton(driveStick, JOYSTICK::BUTTON::BUTTON_5);
	zeroEncoders->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			intake->SetLiftEncoder(0.0);
			climber->ClimbPivotSetEncoder(0.0);
			climber->ClimbWinchSetEncoder(0.0);
			
		},
		{climber, intake}));

	pinControl = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_4);
	pinControl->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			if(climber->PinStatus() == true){
			climber->PinOut(); //removes pin
			}
		},
		{climber}));

	armExtend = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_6);
	armExtend->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			climber->ClimbControl(80, 28); // 80 degrees, 28 inches
			cout<<"ClimbControl set to 80 deg, 28 in"<<endl;
		},
		{climber}));

	swingAndClampBar = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_10);
	swingAndClampBar->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			climber->ClimbControl(1, 1); // 1 degree, 1 inch
			cout<<"ClimbControl set to 1 deg, 1 in"<<endl;
		},
		{climber}));

	winchUp = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_9);
	winchUp->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			climber->ClimbControl((climber->GetAngle()), ((climber->GetLength()) + 0.5));
			cout<<"ClimbControl set to current angle, current length + 0.5"<<endl;
		},
		{climber}));
	winchDown = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_11);
	winchDown->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			climber->ClimbControl((climber->GetAngle()), ((climber->GetLength()) - 0.5));
			cout<<"ClimbControl set to current angle, current length - 0.5"<<endl;
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
			if ((driveStick->GetRawButton(2)) == true)
			{
				intake->RollerControl(-6.0);
			}
			else
			{
				intake->RollerControl(0.8);
			}
		},
		[this]
		{
			intake->RollerControl(0);
		},
		{intake}));

	liftControl = new frc2::JoystickButton(driveStick, JOYSTICK::BUTTON::BUTTON_3);
	liftControl->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			switch (liftPosition)
			{
			case 1:
				//intake->SetP(0.03);
				intake->LiftControl(0.0); // 2 degrees
				liftPosition = 2;
				break;
			case 2:
				//intake->SetP(0.07);
				intake->LiftControl(14); // 30 degrees
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


void RobotContainer::Autonmous()
{

	if (timeLimet > timeTracker)
	{
		timeTracker++;
	}
	else
	{
		timeTracker = 0;

		switch (autoSwitch)
		{
		case 1:
			autoSwitch = 2;
			timeLimet = 100;
			intake->LiftControl(30);
			break;
		case 2:
			autoSwitch = 3;
			timeLimet = 100;
			drive->DriveControl(0.8, 0.8);

			break;
		case 3:
			autoSwitch = 4;
			timeLimet = 50;
			drive->DriveControl(0.0, 0.0);

			break;
		case 4:
			autoSwitch = 5;
			timeLimet = 300;
			intake->RollerControl(0.5);

			break;
		case 5:
			autoSwitch = 6;
			timeLimet = 200;
			intake->RollerControl(0.0);
			drive->DriveControl(-0.5, -0.5);
			intake->LiftControl(1);
			break;
		case 6:
			drive->DriveControl(0.0, 0.0);
			break;
		default:
			break;
		}
	}
}

void RobotContainer::Periodic()
{
}
