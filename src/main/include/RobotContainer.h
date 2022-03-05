// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>

#include "subsystems/DrivetrainDrive.h"

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
#include "subsystems/Climber.h"
#include <frc2/command/Subsystem.h>
#include "Constants.h"
#include "ButtonMap.h"
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/RunCommand.h>
#include "subsystems/Camera.h"
#include "frc2/command/InstantCommand.h"
#include "subsystems/Intake.h"
#include "frc/smartdashboard/SmartDashboard.h"

class RobotContainer
{
public:
//These RobotContainer functions are the constructor and the destructor
//They will bassicly construct the pointer objects and delete them when we are done with them
  RobotContainer();
  ~RobotContainer();

  void Periodic();
//The periodic function is the function that will run ever 20 ms run of the code
  frc2::Command *Autonmous();
  //This is the auto command
private:

//These values are for keeping track of difrent values for the robot container
  int driveReverse = 1;
  int liftPosition = 2;

//This is where we make a pointer object to creat a new class from each subsystem to interact with that subsystem
  DrivetrainDrive *drive;
  Climber *climber;
  Camera *camera;
  Intake *intake;
  //We make he joystick pointers
  Joystick *driveStick, *controlStick;
  //We create the bottons on the joysticks
  frc2::JoystickButton *winchUp, *winchDown, *feedSwitch, *armExtend, *swingAndClampBar, 
  *rollerControl, *liftControl, *zeroEncoders, *minEcoders, *disclodgedWrench, *rollerIn, *rollerOut,
  *liftMid, *reverseDrivetrain;
};
