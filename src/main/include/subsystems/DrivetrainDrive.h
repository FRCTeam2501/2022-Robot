
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "frc2/command/SubsystemBase.h"
#include "frc/drive/DifferentialDrive.h"
#include "rev/CANSparkMax.h"
#include "frc/Motorcontrol/MotorControllerGroup.h"


namespace Drivetrain{
constexpr int frontRight = 2;
constexpr int frontLeft = 1;
constexpr int rearRight = 3;
constexpr int rearLeft = 5;

constexpr double driveSmartCurrentLimet = 60.0;
constexpr double driveSeccondaryCurrentLimet = 70.0;
}


class DrivetrainDrive : public frc2::SubsystemBase {
 public:
 DrivetrainDrive();
 ~DrivetrainDrive();


  void DriveControl(double l, double r);
 void ArcadeDrive(double x, double z);

	void Periodic();
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

 private:
  
  frc::DifferentialDrive *drive;
  frc::MotorControllerGroup  *LeftSide, *RightSide;
	rev::CANSparkMax *frontLeft, *rearLeft, *frontRight, *rearRight;
  void InitDefaultCommand();
};
