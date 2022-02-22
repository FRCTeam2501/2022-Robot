//done


#pragma once

#include <frc2/command/Command.h>
#include "subsystems/Intake.h"
#include "subsystems/DrivetrainDrive.h"
#include "subsystems/Climber.h"
/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
using namespace frc;
#include <frc2/command/Command.h>
#include "include.h"
#include "subsystems/DrivetrainDrive.h"
#include <frc2/command/Subsystem.h>
#include "Constants.h"
#include "ButtonMap.h"
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/RunCommand.h>
#include "subsystems/Intake.h"
#include "subsystems/Climber.h"




class RobotContainer {
 public:
  RobotContainer();
~RobotContainer();
  frc2::Command* GetAutonomousCommand();
void Periodic();

 private:
 int counter = -2;
  // The robot's subsystems and commands are defined here...
DrivetrainDrive *drive;
Intake *intakem;
Climber *climberm;
Joystick *Stick,*Stick2;
frc2::JoystickButton *up,*down,*in,*out,*off,*upC,*downC,*one,*two,*off2; //put button names here 
  void ConfigureButtonBindings();
};
