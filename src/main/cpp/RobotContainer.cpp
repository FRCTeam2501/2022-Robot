

/****************************************** CREDITS: **********************************************************************************************************************

		This code was written by Jacob Hagberg with a lot of help from Tyler Seiford. 
		Other members of the programming team who helped a lot were Austin and Eddie. 
		This year was a success as far as programming is consirned, as well as the building and electronic aspect. 
		I enjoyed this year a lot working with these guys to built a great robot. We had a great season and I hope for many more to come. 


		I decided to heavily comment this code in the hopes that it will be a good programming resource for the next robotics season, and maybe even beyond that.
		I hope that this is helpful and good luck on programming to whoever is reading this. 
**************************************************************************************************************************************************************************
*/

#include "RobotContainer.h"
#include "iostream"
#include "frc2/command/WaitCommand.h"
#include "frc2/command/SequentialCommandGroup.h"
#include "frc2/command/ParallelCommandGroup.h"

using namespace std;

/*
Ok so this is the robot container. This is where we take all the functions and code from the subsystems and make them respond to control
inputs as well as to get them to work together. We mainly do this through commands 

*/

RobotContainer::RobotContainer()
{
	//This is where we creat a new instance of each subsystem that we can do things with. We also create the joystick objects
	//We will be using these objects to interact with each subsystem in the robot container file. 
	intake = new Intake();
	drive = new DrivetrainDrive();
	driveStick = new Joystick(0); // usb port of 0
	camera = new Camera();
	controlStick = new Joystick(1); // usb port of 1
	climber = new Climber();

	//The WPILIB has a lot of useful things such as commands. Here we are setting the default command to take the 
	//value from the driverstick joystick and putting them into the arcade drive function in the drivetrainDrive subsystem.
	//The values that are passed to that function are multiplied by certian values to make the robot drive the right way. 
	//We then multiply the value of drive reverse to make sure that the drive control will be reversed if we need it to be
	//I never tested the reverse code though I hope it works. 
	drive->SetDefaultCommand(frc2::RunCommand(
		[this]
		{
			//Here is where we get the value of teh throttle to decide to do fast or slow drivetrain mode. 
			if (driveStick->GetRawAxis(JOYSTICK::AXIS::Z) <= 0.0)
			{
			//	cout << "Fast Drive Mode" << endl;
				drive->ArcadeDrive(
					driveReverse * -1.0 * driveStick->GetRawAxis(JOYSTICK::AXIS::Y),
					driveReverse * 0.6 * driveStick->GetRawAxis(JOYSTICK::AXIS::X));
			}
			else
			{
			//	cout << "Slow Drive Mode" << endl;

				drive->ArcadeDrive(driveReverse * -0.6 * driveStick->GetRawAxis(
											  JOYSTICK::AXIS::Y),
								   driveReverse * 0.45 * driveStick->GetRawAxis(JOYSTICK::AXIS::X));
			}
		},
		{drive}));

	climber->SetDefaultCommand(frc2::RunCommand(
		[this]
		{
			//This is very similar to the drivetrain default command, every 20 ms it will read the value of the y axis on the control stick
			//it will then pass that value to the climb control function so that it can be checked that the climb angle is legal. 
			//the if function is to make a dead zone on the controller so that the noise from the joystick wont move the climber. 
			if (abs(controlStick->GetRawAxis(JOYSTICK::AXIS::Y)) > 0.1)
			{
			//This will take the angle that the climber is currently at and add the value of the y control stick axis, (-1 to 1) and pass it to the
			//climb conntrol function along with current angle.
				climber->ClimbControl(((controlStick->GetRawAxis(JOYSTICK::AXIS::Y)) + climber->GetAngle()), (climber->GetLength()));
			}
		},
		{climber}));

	disclodgedWrench = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_8);
	disclodgedWrench->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			//This command connected to button 8 on the  control stick will run the DislodgeWrench function in the climber subsystem
			climber->DislodgeWrench();
		},
		{climber}));

	reverseDrivetrain = new frc2::JoystickButton(driveStick, JOYSTICK::BUTTON::BUTTON_10);
	reverseDrivetrain->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			//This will change the drive reverse value to reverse the drivetrain. 
			if(driveReverse < 0){
				driveReverse = 1;
			}else{
				driveReverse = -1;
			}
		},
		{climber}));

	zeroEncoders = new frc2::JoystickButton(driveStick, JOYSTICK::BUTTON::BUTTON_8);
	zeroEncoders->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			//When you press button eight on the drive stick it will zero the encoders on the lift by useing the function SetLiftEncoder in the 
			//intake subsystem and passing the value of 0.0 to it. 
			intake->SetLiftEncoder(0.0);
		},
		{intake}));
	minEcoders = new frc2::JoystickButton(driveStick, JOYSTICK::BUTTON::BUTTON_7);
	minEcoders->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			//Does the same thing except this button should be used when you want to set the lift to its down postition
			intake->SetLiftEncoder(-13.0);
		},
		{intake}));

	armExtend = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_6);
	armExtend->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			//This function sets the climber to 80 degrees and 27 inches. This can be done at any position of the climber and it will avoid the battery area
			climber->ClimbControl(80, 27); // 80 degrees, 28 inches
			cout << "ClimbControl set to 80 deg, 28 in" << endl;
		},
		{climber}));

	swingAndClampBar = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_5);
	swingAndClampBar->WhenPressed(new frc2::InstantCommand(
		[this]
		{

			//This gets the climber to near its neutral position
			climber->ClimbControl(0, 0); // 1 degree, 1 inch
			cout << "ClimbControl set to 1 deg, 1 in" << endl;
		},
		{climber}));

	winchUp = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::TRIGGER);
	winchUp->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			//This takes the throttle value and decides wether to do fast or slow mode for the climber. 
			if (controlStick->GetRawAxis(JOYSTICK::AXIS::Z) <= 0.0)
			{
				//passes the current angle so that it will stay at the current angle and also adds 2.5 inches or 0.5 in depending weather it is on fast or slow mode
				climber->ClimbControl((climber->GetAngle()), ((climber->GetLength()) + 2.5));
			//fast mode
			}else{
				climber->ClimbControl((climber->GetAngle()), ((climber->GetLength()) + 0.5));
			//slow mode
			}
		},
		{climber}));
	winchDown = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::THUMB);
	winchDown->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			//does the same thing as the last function but it subtracts length
			if (controlStick->GetRawAxis(JOYSTICK::AXIS::Z) <= 0.0)
			{
				climber->ClimbControl((climber->GetAngle()), ((climber->GetLength()) - 2.5));
			}else{
				climber->ClimbControl((climber->GetAngle()), ((climber->GetLength()) - 0.5));
			}
		},
		{climber}));

	feedSwitch = new frc2::JoystickButton(controlStick, JOYSTICK::BUTTON::BUTTON_12);
	feedSwitch->WhenPressed(new frc2::InstantCommand(
		[this]
		{
			//Runs the feed switch function in the camera subsystem to switch to the other camera
			camera->SwitchFeed();
		},
		{camera}));

	rollerIn = new frc2::JoystickButton(driveStick, JOYSTICK::BUTTON::TRIGGER);
	rollerIn->WhileHeld(new frc2::StartEndCommand(
		[this]
		{
		//This is a start end command, this will start the roller motor and stop it when you release the trigger
			intake->RollerControl(0.8);
		},
		[this]
		{
			intake->RollerControl(0);
		},
		{intake}));
	rollerOut = new frc2::JoystickButton(driveStick, JOYSTICK::BUTTON::BUTTON_11);
	rollerOut->WhileHeld(new frc2::StartEndCommand(
		[this]
		{
			//Same thing as before except reverse
			intake->RollerControl(-0.8);
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
				//This will set the position of the lift to up and down, this button allows you to toggle it
			case 1:
				intake->LiftControl(2.9);
				liftPosition = 2;
				break;
			case 2:

				intake->LiftControl(-13);
				liftPosition = 1;
				break;
			default:
				break;
			}
		},
		{intake}));
	
	liftMid = new frc2::JoystickButton(driveStick, JOYSTICK::BUTTON::BUTTON_4);
	liftMid->WhenPressed(new frc2::InstantCommand(
		[this]
		{
		//This was to put the intake up half way in order to be at the right spot for climbing
			intake->LiftControl(-3);
		},
		{intake}));
}

RobotContainer::~RobotContainer()
{
	//This is the function where we delete the pointers 
	delete drive;
	delete driveStick;
	delete controlStick;
	delete climber;
	delete camera;
}


//Here is the autonomous command, it needs some code in the robot.cpp so check that out if you want to do it
//This is simple, it just ejects the ball into the low goal and then backs off the tarmach. 

frc2::Command *RobotContainer::Autonmous()
{
	return new frc2::SequentialCommandGroup(
//This is a sequential command group, it is a WPILIB thing that will run commands one after the other, this is how we will do the auto
		frc2::InstantCommand{
			[this]
			{
				//This zeros all the encoders to make sure that their current position is right to start the match so we can move it as we want
				intake->SetLiftEncoder(0.0);
				intake->LiftControl(2.9);
				climber->ClimbPivotSetEncoder(0.0);
				climber->ClimbWinchSetEncoder(0.0);
			},
			{intake, climber}},
		frc2::WaitCommand{
			400_ms},
			//this part will wait for 400 mili seconds
		frc2::InstantCommand{
			[this]
			{
				intake->RollerControl(-0.8);
			},
			{intake}},
		frc2::WaitCommand{
			2_s},

		frc2::RunCommand{
			[this]{
			//This one is a run command so that the drive default command doesn't set the drivetrain to zero 2 ms after the drivetrain is set to go backwards
				drive->ArcadeDrive(-0.6,0.0);
				intake->RollerControl(0.0);
			},
			{intake, drive}
		}.WithTimeout(
			1.2_s
		),
		//back to an instant command to stop the drive
		frc2::InstantCommand{
			[this]
			{
				drive->ArcadeDrive(0.0, 0.0);
			},
			{drive}}

	);
}

void RobotContainer::Periodic()
{
}

/*
OK, so here is the deal. Right now I am on the coach buss driving home from duluth after we did not get picked in the 2022 year. 
I am planning on writing comments to help whoever is the next programming captian. Right now it is looking like Austin but who knows.
It was a bummer that we did not go on but I think we did overall well and we got a high climb with a solid ball intake. 
The reason that I am commenting so heavenly is to give the next programming captian as many recources as I can. I also dont want to look
like a one foot out the door kinda guy. I hope this really helps because I probably wont be able to help much next year. 
But as long as we still have Tyler we should be fine.  
*/

