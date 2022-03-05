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
  RobotContainer();
  ~RobotContainer();

  void Periodic();

  frc2::Command *Autonmous();
  // frc2::Command* GetAutoCommand();

private:
  double angleAdd = 0;
  double lengthAdd = 0;

  bool driveReverse = false;

  int liftPosition = 2;
  // The robot's subsystems and commands are defined here...
  DrivetrainDrive *drive;
  Climber *climber;
  Camera *camera;
  Intake *intake;
  Joystick *driveStick, *controlStick;
  frc2::JoystickButton *winchUp, *winchDown, *feedSwitch, *armExtend, *swingAndClampBar, 
  *rollerControl, *liftControl, *zeroEncoders, *minEcoders, *disclodgedWrench, *rollerIn, *rollerOut,
  *liftMid, *reverseDrivetrain;
};
