// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "subsystems/DrivetrainDrive.h"
#include "Constants.h"
using namespace frc;

void DrivetrainDrive::Periodic() {}

DrivetrainDrive::DrivetrainDrive()
{
  frontLeft = new rev::CANSparkMax(Drivetrain::frontLeft, rev::CANSparkMax::MotorType::kBrushless);
  frontRight = new rev::CANSparkMax(Drivetrain::frontRight, rev::CANSparkMax::MotorType::kBrushless);
  rearLeft = new rev::CANSparkMax(Drivetrain::rearLeft, rev::CANSparkMax::MotorType::kBrushless);
  rearRight = new rev::CANSparkMax(Drivetrain::rearRight, rev::CANSparkMax::MotorType::kBrushless);

  frontLeft->SetSmartCurrentLimit(Drivetrain::driveSmartCurrentLimet);
  frontLeft->SetSecondaryCurrentLimit(Drivetrain::driveSeccondaryCurrentLimet);
  frontLeft->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  frontRight->SetSmartCurrentLimit(Drivetrain::driveSmartCurrentLimet);
  frontRight->SetSecondaryCurrentLimit(Drivetrain::driveSeccondaryCurrentLimet);
  frontRight->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  rearLeft->SetSmartCurrentLimit(Drivetrain::driveSmartCurrentLimet);
  rearLeft->SetSecondaryCurrentLimit(Drivetrain::driveSeccondaryCurrentLimet);
  rearLeft->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  rearRight->SetSmartCurrentLimit(Drivetrain::driveSmartCurrentLimet);
  rearRight->SetSecondaryCurrentLimit(Drivetrain::driveSeccondaryCurrentLimet);
  rearRight->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  frontLeft->SetInverted(false);
  frontRight->SetInverted(true);

  LeftSide = new frc::MotorControllerGroup(*frontLeft, *rearLeft);
  RightSide = new frc::MotorControllerGroup(*frontRight, *rearRight);

  drive = new DifferentialDrive(*LeftSide, *RightSide);
  RightSide->SetInverted(false);
}
DrivetrainDrive::~DrivetrainDrive()
{
  delete frontLeft;
  delete frontRight;
  delete rearLeft;
  delete rearRight;
  delete drive;
  delete LeftSide;
  delete RightSide;
}

void DrivetrainDrive::DriveControl(double l, double r)
{
  LeftSide->Set(l);
  RightSide->Set(r);
}

void DrivetrainDrive::ArcadeDrive(double x, double z)
{

  drive->ArcadeDrive(x, z);
}
void DrivetrainDrive::InitDefaultCommand()
{
}
// Put methods for controlling this subsystem
// here. Call these from Commands.