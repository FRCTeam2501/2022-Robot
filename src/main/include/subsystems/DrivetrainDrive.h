
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "frc2/command/SubsystemBase.h"
#include "frc/drive/DifferentialDrive.h"
#include "rev/CANSparkMax.h"
#include "frc/Motorcontrol/MotorControllerGroup.h"

namespace Drivetrain
{
  //constexpr needed in a namespace to not get pointer error
  constexpr int frontRight = 15;
  constexpr int frontLeft = 6;
  constexpr int rearRight = 8;
  constexpr int rearLeft = 12;

  constexpr double driveSmartCurrentLimet = 60.0;
  constexpr double driveSeccondaryCurrentLimet = 70.0;
}

class DrivetrainDrive : public frc2::SubsystemBase
{
  //This public means that anything in here can be acessed anywhere in the code
public:

//Drivetrain constructor and destrucotr functions
  DrivetrainDrive();
  ~DrivetrainDrive();

//drivectontrol that was never used 
  void DriveControl(double l, double r);
  //Arcade drive with the two double values that we will be passing it to control the drivetrain
  void ArcadeDrive(double x, double z);

  void Periodic();
  

private:
//The private means that only this sybsystem can acess this stuff

  //Create drive object
  frc::DifferentialDrive *drive;

  //Morot controller group pointers
  frc::MotorControllerGroup *LeftSide, *RightSide;

  //The can spark max pointers that we will use on the drivetrain
  rev::CANSparkMax *frontLeft, *rearLeft, *frontRight, *rearRight;

  //never used this function either
  void InitDefaultCommand();
};
